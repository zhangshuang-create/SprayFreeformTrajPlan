#include "RibbonWidget/RibbonWidget.h"
#include "ui_RibbonWidget.h"
#include <QMenu>
#include <QAction>
#include <QGroupBox>
#include <QLayoutItem>
#include <QMessageBox>
#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QSaveFile>
#include <QStandardPaths>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QPainter>
#include <QPalette>
#include <QProgressDialog>
#include <QStringList>
#include <QPaintEvent>
#include <QEvent>
#include <QTimer>
#include <QMetaObject>
#include <QToolTip>
#include <QTextStream>
#include <QRegularExpression>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <cmath>
#include <limits>
#include <numeric>
#include <data_path.h>

#include <PropertyPanel/PropertyPanel.h>
#include <ThicknessAnalyzer/ThicknessAnalyzer.h>
#include <GaussianModel/GaussianModel.h>
#include <LinearInterpolator/LinearInterpolator.h>
#include "ThicknessGpuCompute.h"
#include "GpuDiagnostics.h"
#include <TrajModule/TrajPlanPipeline.h>
#include <TrajModule/UvPlanningSession.h>
#include <TrajModule/GeometryUtils.h>
#include <TrajOptimization/TrajOptimization.h>

using namespace ModelTrajPlan;

namespace
{
    class ThicknessOverlay final : public QWidget
    {
    public:
        explicit ThicknessOverlay(QWidget* parent) : QWidget(parent)
        {
            setAttribute(Qt::WA_TransparentForMouseEvents);
            setAttribute(Qt::WA_TranslucentBackground);
            parent->installEventFilter(this);
            reposition();
        }

        void setResult(double minimum, double maximum, double average, double sd)
        {
            min_ = minimum;
            max_ = maximum;
            average_ = average;
            sd_ = sd;
            variance_ = sd * sd;
            range_ = maximum - minimum;
            cv_ = std::abs(average) > 1e-12 ? sd / std::abs(average) * 100.0 : 0.0;
            update();
        }

    protected:
        bool eventFilter(QObject* watched, QEvent* event) override
        {
            if (watched == parentWidget() && (event->type() == QEvent::Resize ||
                event->type() == QEvent::Show || event->type() == QEvent::PaletteChange)) {
                reposition();
                update();
            }
            return QWidget::eventFilter(watched, event);
        }

        void paintEvent(QPaintEvent*) override
        {
            QPainter p(this);
            p.setRenderHint(QPainter::Antialiasing);
            const int margin = 10;
            const int barWidth = 20;
            const QRect bar(width() - 150, margin, barWidth, height() - margin * 2);
            QLinearGradient gradient(bar.bottomLeft(), bar.topLeft());
            const auto jet = [](double t) {
                const auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };
                return QColor::fromRgbF(clamp01(1.5 - std::abs(4.0 * t - 3.0)),
                                        clamp01(1.5 - std::abs(4.0 * t - 2.0)),
                                        clamp01(1.5 - std::abs(4.0 * t - 1.0)));
            };
            for (int i = 0; i <= 256; ++i) gradient.setColorAt(double(i) / 256.0, jet(double(i) / 256.0));
            p.fillRect(bar, gradient);
            const QColor background = parentWidget()
                ? parentWidget()->palette().color(QPalette::Window)
                : QColor(Qt::black);
            const bool lightBackground = background.lightnessF() >= 0.5;
            const QColor textColor = lightBackground ? QColor(Qt::black) : QColor(Qt::white);
            p.setPen(textColor);
            constexpr int tickCount = 8;
            for (int i = 0; i < tickCount; ++i) {
                const double t = double(i) / double(tickCount - 1);
                const int y = bar.bottom() - int(std::lround(t * bar.height())) + 5;
                const double value = min_ + (max_ - min_) * t;
                p.drawText(bar.right() + 10, y, QString::number(value, 'f', 4));
            }

            constexpr int statsWidth = 300;
            constexpr int statsHeight = 102;
            constexpr int statsGap = 12;
            const int statsX = std::max(margin, bar.left() - statsGap - statsWidth);
            const int statsY = std::max(margin, bar.bottom() - statsHeight + 1);
            const QRect statsRect(statsX, statsY, statsWidth, statsHeight);
            p.fillRect(statsRect, lightBackground
                ? QColor(255, 255, 255, 190)
                : QColor(0, 0, 0, 170));
            p.setPen(textColor);
            const QStringList stats = {
                QStringLiteral("Thickness statistics"),
                QStringLiteral("Average: %1").arg(average_, 0, 'f', 6),
                QStringLiteral("Variance: %1").arg(variance_, 0, 'f', 6),
                QStringLiteral("Std deviation: %1").arg(sd_, 0, 'f', 6),
                QStringLiteral("CV: %1%").arg(cv_, 0, 'f', 2)
            };
            int textY = statsRect.top() + 18;
            for (const QString& line : stats) {
                p.drawText(statsRect.left() + 8, textY, line);
                textY += 18;
            }
        }

    private:
        void reposition()
        {
            if (parentWidget()) {
                resize(parentWidget()->size());
                move(0, 0);
            }
        }
        double min_ = 0.0, max_ = 0.0, average_ = 0.0, sd_ = 0.0;
        double variance_ = 0.0, range_ = 0.0, cv_ = 0.0;
    };

    void syncConnectedTrajectoryToPublic(
        const ModelTrajPlan::GeneralData& generalData,
        PublicData::TrajData* publicTrajData)
    {
        if (!publicTrajData) {
            return;
        }

        const auto& connected = generalData.trajData_.connectionData_.connectedTrajectory_;
        if (connected.empty()) {
            return;
        }

        auto& publicPoints = publicTrajData->getTrajectoryPoints();
        publicPoints.clear();
        publicTrajData->getSpiralTrajectoryPoints().clear();

        // Debug: Check trajectory_index distribution
        std::map<int, int> indexCounts;
        for (const auto& src : connected) {
            indexCounts[src.trajectory_index]++;
        }
        qInfo() << "[SYNC] Trajectory points:" << connected.size();
        qInfo() << "[SYNC] trajectory_index distribution:";
        for (const auto& kv : indexCounts) {
            qInfo() << "  index" << kv.first << ":" << kv.second << "points";
        }

        double timestamp = 0.0;
        for (const auto& src : connected) {
            PublicData::TrajectoryPoint dst;
            dst.pose = src.pose;
            dst.spray_distance = src.spray_distance;
            dst.velocity = src.velocity;
            // Preserve scan-line ownership. Thickness interpolation must not
            // generate spraying samples across a line-change travel move.
            dst.spray_pass_id = src.trajectory_index;
            publicPoints[timestamp] = dst;
            timestamp += 1.0;
        }
    }

    QString writeThicknessReport(
        const ModelTrajPlan::ThicknessResultData& result,
        QOpenGLWidget* view,
        QString reportPath = QString())
    {
        if (!result.hasValidResult || result.thicknessValues.empty()) {
            return QString();
        }

        if (reportPath.isEmpty()) {
            const QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
            const QString directory = QDir(documents.isEmpty() ? QCoreApplication::applicationDirPath() : documents)
                .filePath(QStringLiteral("ThicknessReports"));
            if (!QDir().mkpath(directory)) {
                return QString();
            }
            reportPath = QDir(directory).filePath(QStringLiteral("thickness_report_%1.html")
                .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
        }
        if (QFileInfo(reportPath).suffix().isEmpty()) {
            reportPath += QStringLiteral(".html");
        }

        std::vector<double> values = result.thicknessValues;
        std::sort(values.begin(), values.end());
        const double p05 = values[static_cast<size_t>((values.size() - 1) * 0.05)];
        const double p95 = values[static_cast<size_t>((values.size() - 1) * 0.95)];
        const double cv = result.coefficientOfVariation;
        const QString uniformity = cv <= 10.0 ? QStringLiteral("Good")
            : cv <= 20.0 ? QStringLiteral("Needs attention") : QStringLiteral("Poor");

        const QString imageName = QFileInfo(reportPath).completeBaseName() + QStringLiteral("_screenshot.png");
        const QString imagePath = QDir(QFileInfo(reportPath).absolutePath()).filePath(imageName);
        QCoreApplication::processEvents();
        const QPixmap screenshot = view ? view->grab() : QPixmap();
        const bool screenshotSaved = !screenshot.isNull() && screenshot.save(imagePath, "PNG");
        const QString imageHtml = screenshotSaved
            ? QStringLiteral("<img src=\"%1\" alt=\"Thickness heatmap screenshot\">").arg(imageName)
            : QStringLiteral("<p class=\"notice\">The viewport was unavailable, so no screenshot was captured.</p>");

        const QString html = QStringLiteral(R"HTML(<!doctype html><html><head><meta charset="utf-8"><title>Thickness Simulation Report</title><style>body{font-family:"Microsoft YaHei",Arial,sans-serif;color:#172033;margin:36px;line-height:1.6}h1{color:#0f766e;border-bottom:2px solid #99f6e4;padding-bottom:10px}h2{color:#134e4a;margin-top:30px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #cbd5e1;padding:8px;text-align:left}th{background:#f0fdfa}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.metric{background:#f8fafc;border-left:4px solid #0f766e;padding:10px}img{max-width:100%;border:1px solid #cbd5e1}.notice{background:#fff7ed;border-left:4px solid #f97316;padding:10px}.small{color:#475569;font-size:.9em}</style></head><body><h1>Thickness Simulation Analysis Report</h1><p class="small">Generated: %1</p><h2>1. Summary</h2><div class="grid"><div class="metric"><b>Uniformity</b><br>%2</div><div class="metric"><b>Coefficient of variation</b><br>%3%</div><div class="metric"><b>Samples</b><br>%4</div></div><table><tr><th>Minimum</th><th>Maximum</th><th>Mean</th><th>Std. deviation</th><th>P05</th><th>P95</th></tr><tr><td>%5</td><td>%6</td><td>%7</td><td>%8</td><td>%9</td><td>%10</td></tr></table><table><tr><th>Below tolerance</th><th>Above tolerance</th></tr><tr><td>%11</td><td>%12</td></tr></table><h2>2. Thickness Screenshot</h2>%13<h2>3. Cause Assessment</h2><p>These are data-based indicators, not confirmed physical root causes. Validate them against the trajectory, spray distance, incidence angle, flow rate and robot speed.</p><ul><li><b>High-thickness risk:</b> %12 samples exceed the upper tolerance. Inspect path overlap, low speed or dwell, and locally short spray distance in the yellow/red heatmap areas.</li><li><b>Low-thickness risk:</b> %11 samples fall below the lower tolerance. Inspect edge coverage, excessive path spacing, large spray distance, oblique incidence and self-occlusion in blue areas.</li><li><b>Overall variation:</b> CV is %3% (%2). Prioritize consistent overlap and speed, then verify distance and normal-angle consistency.</li></ul><p class="small">Units follow the active thickness simulation settings.</p></body></html>)HTML")
            .arg(QDateTime::currentDateTime().toString(Qt::ISODate), uniformity)
            .arg(cv, 0, 'f', 2).arg(values.size()).arg(result.minThickness, 0, 'f', 6)
            .arg(result.maxThickness, 0, 'f', 6).arg(result.averageThickness, 0, 'f', 6)
            .arg(result.stdDeviation, 0, 'f', 6).arg(p05, 0, 'f', 6).arg(p95, 0, 'f', 6)
            .arg(result.underToleranceCount).arg(result.overToleranceCount).arg(imageHtml);

        QSaveFile reportFile(reportPath);
        if (!reportFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return QString();
        }
        reportFile.write(html.toUtf8());
        return reportFile.commit() ? reportPath : QString();
    }
}

RibbonWidget::RibbonWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RibbonWidget)
    , m_currentGenerationMethod("Slicing")
    , m_currentSlicingMethod("AA Slicing")
    , m_currentThicknessMethod("Gaussian")
{
    ui->setupUi(this);
    configureRibbonSizing();
    setupConnections();
    setupMenus();
    createUvToolPanel();
}

RibbonWidget::~RibbonWidget()
{
    // The optimizer owns no Qt objects, but it does operate on the pipeline
    // owned by this widget.  Join before member destruction makes shutdown
    // deterministic and prevents a worker from accessing released geometry.
    if (m_uvOptimizationThread.joinable()) m_uvOptimizationThread.join();
    delete ui;
}

int RibbonWidget::getCurrentTabIndex() const
{
    return ui->ribbonTabWidget->currentIndex();
}

void RibbonWidget::setCurrentTab(int index)
{
    ui->ribbonTabWidget->setCurrentIndex(index);
}

void RibbonWidget::setupConnections()
{
    // Connect tab change signal
    connect(ui->ribbonTabWidget, &QTabWidget::currentChanged,
            this, &RibbonWidget::tabChanged);

    // Trajectory generation tab connections
    connect(ui->btnTrajectoryMethod, &QToolButton::clicked,
            this, &RibbonWidget::trajectoryMethodClicked);
    connect(ui->btnSlicingMethod, &QToolButton::clicked,
            this, &RibbonWidget::slicingMethodClicked);
    connect(ui->btnConfirmSlicing, &QToolButton::clicked,
            this, &RibbonWidget::onConfirmSlicingClicked);
    connect(ui->btnGenerateTraj, &QToolButton::clicked,
            this, &RibbonWidget::onGenerateTrajectoryClicked);
    connect(ui->btnConnectTraj, &QToolButton::clicked,
            this, &RibbonWidget::onConnectTrajectoryClicked);
    connect(ui->btnCancelSlicing, &QToolButton::clicked,
            this, &RibbonWidget::onCancelSlicingClicked);

    // Trajectory generation clear connections
    connect(ui->btnClearModel, &QToolButton::clicked,
            this, &RibbonWidget::clearModel);
    connect(ui->btnClearTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::clearTrajectory);
    connect(ui->btnClearConnection, &QToolButton::clicked,
            this, &RibbonWidget::clearConnection);

    // Trajectory generation Import/Export connections
    connect(ui->btnImportModel, &QToolButton::clicked,
            this, &RibbonWidget::onImportModelClicked);
    connect(ui->btnExportTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onExportTrajectoryClicked);

    // Collision optimization tab connections
    connect(ui->btnImportWorkpiece, &QToolButton::clicked,
            this, &RibbonWidget::importCollisionWorkpiece);
    connect(ui->btnImportRobot, &QToolButton::clicked,
            this, &RibbonWidget::importRobot);
    connect(ui->btnImportCollisionModel, &QToolButton::clicked,
            this, &RibbonWidget::importCollisionModel);
    connect(ui->btnSelectOptimizationAlgorithm, &QToolButton::clicked, this, [this]() {
        if (!trajOptimizationData_) {
            QMessageBox::warning(this, tr("Select Algorithm"),
                tr("Collision optimization data is not initialized."));
            return;
        }

        QMenu menu(this);
        QAction* defaultGa = menu.addAction(tr("Genetic Algorithm"));
        QAction* islandGaRrt = menu.addAction(tr("Island GA + RRT"));
        QAction* aco = menu.addAction(tr("Ant Colony Optimization"));
        QAction* selected = menu.exec(ui->btnSelectOptimizationAlgorithm->mapToGlobal(
            QPoint(0, ui->btnSelectOptimizationAlgorithm->height())));
        if (!selected) return;

        trajOptimizationData_->useParallelGAMethod_ = selected == islandGaRrt;
        trajOptimizationData_->useACOMethod_ = selected == aco;
        m_currentCollisionMethod = selected == defaultGa
            ? QStringLiteral("GA")
            : (selected == islandGaRrt ? QStringLiteral("Island GA + RRT") : QStringLiteral("ACO"));
        ui->btnSelectOptimizationAlgorithm->setToolTip(
            tr("Current algorithm: %1").arg(m_currentCollisionMethod));
        emit selectOptimizationAlgorithmClicked();
        emit collisionOptimizationMethodSelected();
    });
    connect(ui->btnRunOptimization, &QToolButton::clicked,
            this, &RibbonWidget::runCollisionOptimization);
    connect(ui->btnClearOptimizationResult, &QToolButton::clicked,
            this, &RibbonWidget::clearOptimizationResult);

    // Collision optimization clear connections
    connect(ui->btnClearCollisionWorkpiece, &QToolButton::clicked,
            this, &RibbonWidget::clearCollisionWorkpiece);
    connect(ui->btnClearCollisionRobot, &QToolButton::clicked,
            this, &RibbonWidget::clearCollisionRobot);
    connect(ui->btnClearCollisionModel, &QToolButton::clicked,
            this, &RibbonWidget::clearCollisionModel);

    // Collision optimization additional Import/Export connections
    connect(ui->btnImportCollisionTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onImportCollisionTrajectoryClicked);
    connect(ui->btnExportOptimizedTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onExportOptimizedTrajectoryClicked);
    connect(ui->btnExportCollisionReport, &QToolButton::clicked,
            this, &RibbonWidget::onExportCollisionReportClicked);

    // Pose transformation tab connections
    connect(ui->btnImportMatrixJson, &QToolButton::clicked,
            this, &RibbonWidget::importTransformJson);
    connect(ui->btnLoadExtraModel, &QToolButton::clicked,
            this, &RibbonWidget::loadExtraTransformModel);
    connect(ui->btnApplyTransform, &QToolButton::clicked,
            this, &RibbonWidget::applyTransform);

    // Pose transformation additional Import/Export connections
    connect(ui->btnImportTransformTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onImportTransformTrajectoryClicked);
    connect(ui->btnExportMatrixJson, &QToolButton::clicked,
            this, &RibbonWidget::exportTransformJson);
    connect(ui->btnExportTransformedModel, &QToolButton::clicked,
            this, &RibbonWidget::onExportTransformedModelClicked);
    connect(ui->btnExportTransformedTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onExportTransformedTrajectoryClicked);
    connect(ui->btnResetTransform, &QToolButton::clicked,
            this, &RibbonWidget::resetTransform);
    connect(ui->btnClearExtraModel, &QToolButton::clicked,
            this, &RibbonWidget::onClearExtraModelClicked);
    connect(ui->btnClearTransformResult, &QToolButton::clicked,
            this, &RibbonWidget::onClearTransformResultClicked);
    connect(ui->btnClearMatrix, &QToolButton::clicked,
            this, &RibbonWidget::onClearMatrixClicked);

    // Simulation tab connections
    connect(ui->btnImportTrajectoryFile, &QToolButton::clicked, this, [this]() {
        onImportTrajectoryClicked();
        emit importTrajectoryFileClicked();
    });
    connect(ui->btnUseExistingTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::useExistingTrajectoryClicked);
    connect(ui->btnPlay, &QToolButton::clicked,
            this, &RibbonWidget::playClicked);
    connect(ui->btnPause, &QToolButton::clicked,
            this, &RibbonWidget::pauseClicked);
    connect(ui->btnStop, &QToolButton::clicked,
            this, &RibbonWidget::stopClicked);
    connect(ui->btnReset, &QToolButton::clicked,
            this, &RibbonWidget::resetClicked);
    connect(ui->btnPlaybackSpeed, &QToolButton::clicked,
            this, &RibbonWidget::playbackSpeedClicked);

    // Thickness prediction tab connections
    connect(ui->btnImportDepositionModel, &QToolButton::clicked,
            this, &RibbonWidget::importDepositionModel);
    connect(ui->btnStartThicknessSimulation, &QToolButton::clicked,
            this, &RibbonWidget::runThicknessSimulation);
    connect(ui->btnClearThicknessResult, &QToolButton::clicked,
            this, &RibbonWidget::clearThicknessResult);

    // Thickness prediction clear connections
    connect(ui->btnClearDepositionModel, &QToolButton::clicked,
            this, &RibbonWidget::clearDepositionModel);
    connect(ui->btnClearThicknessInputs, &QToolButton::clicked,
            this, &RibbonWidget::clearThicknessInputs);

    // Thickness prediction additional Import/Export connections
    connect(ui->btnImportThicknessWorkpiece, &QToolButton::clicked,
            this, &RibbonWidget::onImportThicknessWorkpieceClicked);
    connect(ui->btnImportPointCloud, &QToolButton::clicked,
            this, &RibbonWidget::onImportPointCloudClicked);
    connect(ui->btnImportThicknessTrajectory, &QToolButton::clicked,
            this, &RibbonWidget::onImportThicknessTrajectoryClicked);
    connect(ui->btnExportThicknessResult, &QToolButton::clicked,
            this, &RibbonWidget::onExportThicknessResultClicked);
    connect(ui->btnExportHeatmap, &QToolButton::clicked,
            this, &RibbonWidget::onExportHeatmapClicked);
    connect(ui->btnExportThicknessReport, &QToolButton::clicked,
            this, &RibbonWidget::onExportThicknessReportClicked);
}

void RibbonWidget::setupMenus()
{
    // Setup trajectory generation method menu
    QMenu* trajectoryMenu = new QMenu(this);

    QAction* slicingAction = trajectoryMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_slicing.png"),
                                                       tr("Slicing"));
    connect(slicingAction, &QAction::triggered, [this]() {
        m_currentGenerationMethod = "Slicing";
        ui->btnTrajectoryMethod->setText(tr("Trajectory\nMethod: Slicing"));
        setGenerationToolPanel(tr("Slicing"), true);
        emit slicingGenerationSelected();
    });

    QAction* uvMappingAction = trajectoryMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_generate_trajectory.png"),
                                                         tr("UV Mapping"));
    connect(uvMappingAction, &QAction::triggered, [this]() {
        m_currentGenerationMethod = "UV Mapping";
        ui->btnTrajectoryMethod->setText(tr("Trajectory\nMethod: UV"));
        setGenerationToolPanel(tr("UV Mapping"), false);
        updateUvCommandState();
        emit uvMappingGenerationSelected();
    });

    QAction* meshDecompositionAction = trajectoryMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_generate_trajectory.png"),
                                                                 tr("Mesh Decomposition"));
    connect(meshDecompositionAction, &QAction::triggered, [this]() {
        m_currentGenerationMethod = "Mesh Decomposition";
        ui->btnTrajectoryMethod->setText(tr("Trajectory\nMethod: Mesh"));
        setGenerationToolPanel(tr("Mesh Decomposition"), false);
        emit meshDecompositionGenerationSelected();
    });

    ui->btnTrajectoryMethod->setMenu(trajectoryMenu);
    ui->btnTrajectoryMethod->setText(tr("Trajectory\nMethod: Slicing"));

    // Setup Slicing Method Menu
    QMenu* slicingMenu = new QMenu(this);

    QAction* aaSlicingAction = slicingMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_slicing.png"),
                                                       tr("AA Slicing"));
    connect(aaSlicingAction, &QAction::triggered, [this]() {
        m_currentSlicingMethod = "AA Slicing";
        ui->btnSlicingMethod->setText(tr("Slicing\nMethod: AA"));
        if (generalData_) {
            generalData_->slicingData_.managedParams_.mode = ModelTrajPlan::SlicingMode::AA;
        }
        emit aaSlicingSelected();
    });

    QAction* parallelAction = slicingMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_slicing_method.png"),
                                                      tr("Parallel Slicing"));
    connect(parallelAction, &QAction::triggered, [this]() {
        m_currentSlicingMethod = "Parallel";
        ui->btnSlicingMethod->setText(tr("Slicing\nMethod: Parallel"));
        if (generalData_) {
            generalData_->slicingData_.managedParams_.mode = ModelTrajPlan::SlicingMode::Parallel;
        }
        emit parallelSlicingSelected();
    });

    QAction* adaptiveSlicingAction = slicingMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_slicing_method.png"),
                                                             tr("Adaptive Slicing"));
    connect(adaptiveSlicingAction, &QAction::triggered, [this]() {
        m_currentSlicingMethod = "Adaptive";
        ui->btnSlicingMethod->setText(tr("Slicing\nMethod: Adaptive"));
        // Adaptive/Contour do not have separate execution paths yet; retain
        // the historical AA backend instead of inheriting a prior Parallel mode.
        if (generalData_) {
            generalData_->slicingData_.managedParams_.mode = ModelTrajPlan::SlicingMode::AA;
        }
        emit adaptiveSlicingSelected();
    });

    QAction* contourAction = slicingMenu->addAction(QIcon(":/RibbonWidget/images/icons/icon_slicing_method.png"),
                                                     tr("Contour Slicing"));
    connect(contourAction, &QAction::triggered, [this]() {
        m_currentSlicingMethod = "Contour";
        ui->btnSlicingMethod->setText(tr("Slicing\nMethod: Contour"));
        if (generalData_) {
            generalData_->slicingData_.managedParams_.mode = ModelTrajPlan::SlicingMode::AA;
        }
        emit contourSlicingSelected();
    });

    ui->btnSlicingMethod->setMenu(slicingMenu);
    ui->btnSlicingMethod->setText(tr("Slicing\nMethod: AA"));
    setGenerationToolPanel(tr("Slicing"), true);

    // Thickness deposition model menu. The selected value is consumed by the
    // CPU analyzer when Start Simulation is pressed.
    QMenu* thicknessModelMenu = new QMenu(ui->btnThicknessMethod);
    QAction* gaussianModelAction = thicknessModelMenu->addAction(tr("Gaussian model"));
    QAction* doubleBetaModelAction = thicknessModelMenu->addAction(tr("Double Beta model"));
    QAction* skewedDoubleBetaModelAction = thicknessModelMenu->addAction(tr("Skewed Double Beta model"));
    QAction* zigongModelAction = thicknessModelMenu->addAction(tr("zigong model"));
    QAction* model420Action = thicknessModelMenu->addAction(tr("420 model"));

    connect(gaussianModelAction, &QAction::triggered, this, [this]() {
        m_currentThicknessMethod = QStringLiteral("Gaussian");
        ui->btnThicknessMethod->setText(tr("Thickness\nModel: Gaussian"));
        emit gaussianModelSelected();
    });
    connect(doubleBetaModelAction, &QAction::triggered, this, [this]() {
        m_currentThicknessMethod = QStringLiteral("DoubleBeta");
        ui->btnThicknessMethod->setText(tr("Thickness\nModel: Double Beta"));
        emit betaModelSelected();
    });
    connect(skewedDoubleBetaModelAction, &QAction::triggered, this, [this]() {
        m_currentThicknessMethod = QStringLiteral("SkewedDoubleBeta");
        ui->btnThicknessMethod->setText(tr("Thickness\nModel: Skewed Double Beta"));
        emit customModelSelected();
    });
    connect(zigongModelAction, &QAction::triggered, this, [this]() {
        m_currentThicknessMethod = QStringLiteral("zigong");
        ui->btnThicknessMethod->setText(tr("Thickness\nModel: zigong"));
        emit zigongModelSelected();
    });
    connect(model420Action, &QAction::triggered, this, [this]() {
        m_currentThicknessMethod = QStringLiteral("420");
        ui->btnThicknessMethod->setText(tr("Thickness\nModel: 420"));
        emit model420Selected();
    });
    ui->btnThicknessMethod->setMenu(thicknessModelMenu);
    ui->btnThicknessMethod->setText(tr("Thickness\nModel: Gaussian"));
    setupClearMenus();
}

void RibbonWidget::setupClearMenus()
{
    const auto prepareClearButton = [this](QToolButton* button, const QString& text) {
        button->setIcon(QIcon(":/RibbonWidget/images/icons/icon_clear.png"));
        button->setIconSize(QSize(28, 28));
        button->setMinimumSize(QSize(90, 82));
        button->setText(text);
        button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        button->setPopupMode(QToolButton::InstantPopup);
    };

    ui->btnClearModel->setVisible(false);
    ui->btnClearTrajectory->setVisible(false);
    ui->btnClearConnection->setVisible(false);

    QToolButton* trajectoryClearButton = new QToolButton(ui->groupTrajectoryClear);
    trajectoryClearButton->setObjectName("btnTrajectoryClearMenu");
    prepareClearButton(trajectoryClearButton, tr("Clear\nData"));

    QMenu* trajectoryClearMenu = new QMenu(trajectoryClearButton);
    QAction* clearModelAction = trajectoryClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Model"));
    QAction* clearTrajectoryAction = trajectoryClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Trajectory"));
    QAction* clearConnectionAction = trajectoryClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Connection"));
    connect(clearModelAction, &QAction::triggered, this, &RibbonWidget::clearModel);
    connect(clearTrajectoryAction, &QAction::triggered, this, &RibbonWidget::clearTrajectory);
    connect(clearConnectionAction, &QAction::triggered, this, &RibbonWidget::clearConnection);

    trajectoryClearButton->setMenu(trajectoryClearMenu);
    ui->groupTrajectoryClear->layout()->addWidget(trajectoryClearButton);

    ui->btnClearCollisionWorkpiece->setVisible(false);
    ui->btnClearCollisionRobot->setVisible(false);
    ui->btnClearCollisionModel->setVisible(false);
    ui->btnClearOptimizationResult->setVisible(false);

    QToolButton* collisionClearButton = new QToolButton(ui->groupCollisionClear);
    collisionClearButton->setObjectName("btnCollisionClearMenu");
    prepareClearButton(collisionClearButton, tr("Clear\nData"));

    QMenu* collisionClearMenu = new QMenu(collisionClearButton);
    QAction* clearCollisionWorkpieceAction = collisionClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Workpiece"));
    QAction* clearCollisionRobotAction = collisionClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Robot"));
    QAction* clearCollisionModelAction = collisionClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Collision Model"));
    collisionClearMenu->addSeparator();
    QAction* clearOptimizationResultAction = collisionClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Optimization Result"));
    connect(clearCollisionWorkpieceAction, &QAction::triggered,
            this, &RibbonWidget::clearCollisionWorkpiece);
    connect(clearCollisionRobotAction, &QAction::triggered,
            this, &RibbonWidget::clearCollisionRobot);
    connect(clearCollisionModelAction, &QAction::triggered,
            this, &RibbonWidget::clearCollisionModel);
    connect(clearOptimizationResultAction, &QAction::triggered,
            this, &RibbonWidget::clearOptimizationResult);

    collisionClearButton->setMenu(collisionClearMenu);
    ui->groupCollisionClear->layout()->addWidget(collisionClearButton);

    // Transform Clear Menu
    ui->btnClearExtraModel->setVisible(false);
    ui->btnClearTransformResult->setVisible(false);
    ui->btnClearMatrix->setVisible(false);

    QToolButton* transformClearButton = new QToolButton(ui->groupTransformClear);
    transformClearButton->setObjectName("btnTransformClearMenu");
    prepareClearButton(transformClearButton, tr("Clear\nData"));

    QMenu* transformClearMenu = new QMenu(transformClearButton);
    QAction* clearExtraModelAction = transformClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Extra Model"));
    QAction* clearTransformResultAction = transformClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Transform Result"));
    QAction* clearMatrixAction = transformClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Matrix"));
    connect(clearExtraModelAction, &QAction::triggered,
            this, &RibbonWidget::clearExtraModel);
    connect(clearTransformResultAction, &QAction::triggered,
            this, &RibbonWidget::clearTransformResult);
    connect(clearMatrixAction, &QAction::triggered,
            this, &RibbonWidget::clearMatrix);

    transformClearButton->setMenu(transformClearMenu);
    ui->groupTransformClear->layout()->addWidget(transformClearButton);

    // Thickness Clear Menu
    ui->btnClearDepositionModel->setVisible(false);
    ui->btnClearThicknessResult->setVisible(false);
    ui->btnClearThicknessInputs->setVisible(false);

    QToolButton* thicknessClearButton = new QToolButton(ui->groupThicknessClear);
    thicknessClearButton->setObjectName("btnThicknessClearMenu");
    prepareClearButton(thicknessClearButton, tr("Clear\nData"));

    QMenu* thicknessClearMenu = new QMenu(thicknessClearButton);
    QAction* clearDepositionModelAction = thicknessClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Deposition Model"));
    QAction* clearThicknessInputsAction = thicknessClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear.png"), tr("Thickness Inputs"));
    QAction* clearThicknessResultAction = thicknessClearMenu->addAction(
        QIcon(":/RibbonWidget/images/icons/icon_clear_thickness.png"), tr("Thickness Result"));
    connect(clearDepositionModelAction, &QAction::triggered,
            this, &RibbonWidget::clearDepositionModel);
    connect(clearThicknessInputsAction, &QAction::triggered,
            this, &RibbonWidget::clearThicknessInputs);
    connect(clearThicknessResultAction, &QAction::triggered,
            this, &RibbonWidget::clearThicknessResult);

    thicknessClearButton->setMenu(thicknessClearMenu);
    ui->groupThicknessClear->layout()->addWidget(thicknessClearButton);
}

void RibbonWidget::configureRibbonSizing()
{
    constexpr int kRibbonHeight = 176;
    constexpr int kRibbonMaxHeight = 240;
    constexpr int kButtonHeight = 82;
    constexpr int kGroupMinHeight = 112;

    setMinimumHeight(kRibbonHeight);
    setMaximumHeight(kRibbonMaxHeight);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    ui->ribbonTabWidget->setMinimumHeight(kRibbonHeight);
    ui->ribbonTabWidget->setMaximumHeight(kRibbonMaxHeight);
    ui->ribbonTabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    const QList<QToolButton*> buttons = findChildren<QToolButton*>();
    for (QToolButton* button : buttons) {
        button->setMinimumHeight(kButtonHeight);
        button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    }

    const QList<QGroupBox*> groups = findChildren<QGroupBox*>();
    for (QGroupBox* group : groups) {
        group->setMinimumHeight(kGroupMinHeight);
        group->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    }

    for (int i = 0; i < ui->ribbonTabWidget->count(); ++i) {
        QWidget* page = ui->ribbonTabWidget->widget(i);
        if (page) {
            page->setMinimumHeight(kGroupMinHeight + 10);
            page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        }
    }
}

void RibbonWidget::setGenerationToolPanel(const QString& title, bool showSlicingControls)
{
    ui->groupSlicing->setTitle(title);

    const QList<QWidget*> slicingWidgets = {
        ui->btnSlicingMethod,
        ui->slicingSeparator,
        ui->btnConfirmSlicing,
        ui->btnGenerateTraj,
        ui->btnConnectTraj,
        ui->btnCancelSlicing
    };

    for (QWidget* widget : slicingWidgets) {
        widget->setVisible(showSlicingControls);
    }
    if (m_uvToolGroup) m_uvToolGroup->setVisible(!showSlicingControls && title == tr("UV Mapping"));
}

void RibbonWidget::createUvToolPanel()
{
    m_uvPipeline = std::make_unique<SprayTrajPlan::TrajPlanPipeline>();
    m_uvSession = std::make_unique<SprayTrajPlan::UvPlanningSession>();
    m_uvPipeline->initialize();
    m_uvToolGroup = new QGroupBox(tr("UV Mapping"), ui->tabTrajectoryGeneration);
    auto* layout = new QHBoxLayout(m_uvToolGroup);
    const auto addButton = [this, layout](const QString& name, const QString& label) {
        auto* button = new QToolButton(m_uvToolGroup);
        button->setObjectName(name);
        button->setText(label);
        button->setIcon(QIcon(":/RibbonWidget/images/icons/icon_generate_trajectory.png"));
        button->setIconSize(QSize(28, 28));
        button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        button->setMinimumSize(70, 60);
        layout->addWidget(button);
        return button;
    };
    m_startUvMapButton = addButton("StartUVmap", tr("Start UV Map"));
    m_startUvTrajectoryButton = addButton("Startgetraj", tr("Start Get Traj"));
    m_startUvOptimizeButton = addButton("Startopt", tr("Start Optimize"));
    m_exportUvButton = addButton("ExportUVTrajectory", tr("Export Trajectory"));
    ui->trajectoryGenerationLayout->insertWidget(4, m_uvToolGroup);
    connect(m_startUvMapButton, &QToolButton::clicked, this, &RibbonWidget::onStartUvMapClicked);
    connect(m_startUvTrajectoryButton, &QToolButton::clicked, this, &RibbonWidget::onStartUvTrajectoryClicked);
    connect(m_startUvOptimizeButton, &QToolButton::clicked, this, &RibbonWidget::onStartUvOptimizeClicked);
    connect(m_exportUvButton, &QToolButton::clicked, this, &RibbonWidget::onExportUvTrajectoryClicked);
    m_uvToolGroup->hide();
    updateUvCommandState();
}

void RibbonWidget::updateUvCommandState()
{
    const bool active = m_hasActiveWorkspace && m_uvSession && !m_uvOptimizationRunning.load();
    m_startUvMapButton->setEnabled(active && m_uvSession->canMap());
    m_startUvTrajectoryButton->setEnabled(active && m_uvSession->canGenerateTrajectory());
    m_startUvOptimizeButton->setEnabled(active && m_uvSession->canOptimize());
    m_exportUvButton->setEnabled(active && m_uvSession->canExport());
    if (auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_)) {
        panel->setUvControlsEnabled(active && !m_uvSession->isRunning());
        panel->setUvTrajectoryGenerationEnabled(active && m_uvSession->canGenerateTrajectory());
    }
}

SprayTrajPlan::TrajectoryParams RibbonWidget::uvParameters() const
{
    SprayTrajPlan::TrajectoryParams params;
    const auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_);
    if (!panel) return params;
    params.target_spacing = panel->uvTargetSpacing();
    params.sample_step = params.target_spacing / 4.0;
    params.trajectory_speed = panel->uvTrajectorySpeed();
    params.spray_distance = panel->uvSprayDistance();
    params.max_seam_spray_distance = panel->uvMaxSeamSprayDistance();
    params.a_max = panel->uvMaximumAcceleration();
    params.j_max = panel->uvMaximumJerk();
    params.arc_lock_guard_d = panel->uvArcLockGuardD();
    params.arc_speed_loss_pct = panel->uvArcSpeedLossPct();
    return params;
}

bool RibbonWidget::displayUvSourceModel()
{
    if (!m_uvPipeline || !publicModelData_ || !publicDataRender_) return false;
    const QString meshPath = QString::fromStdString(m_uvPipeline->getOptCutsResult().mesh_path);
    if (meshPath.isEmpty() || !QFileInfo::exists(meshPath)) return false;
    if (QFileInfo(meshPath).suffix().compare("stl", Qt::CaseInsensitive) != 0) {
        qWarning() << "UV source mesh is not renderable by the public STL renderer:" << meshPath;
        return false;
    }
    if (!publicModelData_->loadModel(meshPath.toStdString())) {
        qWarning() << "Failed to load UV source model:" << meshPath;
        return false;
    }

    publicDataRender_->m_PublicSTLRenderData.needsUpdate = true;
    publicDataRender_->m_PublicSTLRenderData.isVisible = true;
    if (openGLView_) {
        openGLView_->makeCurrent();
        PublicData::PublicDataRender::drawPublicModel(
            publicDataRender_->m_PublicSTLRenderData, *publicModelData_);
        openGLView_->doneCurrent();
    }
    refreshDataDisplay();
    return true;
}

bool RibbonWidget::renderUvTrajectory()
{
    if (!m_uvPipeline || !publicDataRender_ || !openGLView_) return false;
    return renderUvTrajectory(m_uvPipeline->getTrajectory());
}

bool RibbonWidget::renderUvTrajectory(const std::vector<SprayTrajPlan::TrajectorySegment>& trajectory)
{
    if (!publicDataRender_ || !openGLView_) return false;
    auto& render = publicDataRender_->m_PublicTrajectoryRenderData;
    if (!render.trajectoryModel) return false;

    // Surface scans and the physical seam lie on the workpiece. Lift only
    // those segments along their outward normal to avoid z-fighting. Travel
    // segments deliberately retain their true 3D geometry: rendering them as
    // surface-projected would conceal an off-surface or through-model move.
    std::vector<Eigen::Vector3d> sourcePoints;
    std::vector<size_t> segmentOffsets;
    std::vector<bool> shouldLift;
    segmentOffsets.reserve(trajectory.size());
    shouldLift.reserve(trajectory.size());
    for (const auto& segment : trajectory) {
        segmentOffsets.push_back(sourcePoints.size());
        shouldLift.push_back(segment.kind == "SURFACE_SCAN" || segment.kind == "SEAM_TRANSITION");
        if (segment.xyz.size() >= 2) sourcePoints.insert(sourcePoints.end(), segment.xyz.begin(), segment.xyz.end());
    }

    std::vector<Eigen::Vector3d> displayPoints = sourcePoints;
    if (m_uvPipeline && !sourcePoints.empty()) {
        const auto& result = m_uvPipeline->getOptCutsResult();
        if (!result.mesh_vertices.empty() && !result.mesh_faces.empty()) {
            std::vector<Eigen::Vector3d> closestPoints;
            std::vector<double> distances;
            std::vector<int> faceIndices;
            SprayTrajPlan::GeometryUtils::closestPointsRobust(
                result.mesh_vertices, result.mesh_faces, sourcePoints, closestPoints, distances, faceIndices);

            Eigen::Vector3d meshCenter = Eigen::Vector3d::Zero();
            Eigen::Vector3d boundsMin = result.mesh_vertices.front();
            Eigen::Vector3d boundsMax = result.mesh_vertices.front();
            for (const auto& vertex : result.mesh_vertices) {
                meshCenter += vertex;
                boundsMin = boundsMin.cwiseMin(vertex);
                boundsMax = boundsMax.cwiseMax(vertex);
            }
            meshCenter /= static_cast<double>(result.mesh_vertices.size());
            const double lift = std::clamp((boundsMax - boundsMin).norm() * 5.0e-4, 0.2, 2.0);
            for (size_t segmentIndex = 0; segmentIndex < trajectory.size(); ++segmentIndex) {
                if (!shouldLift[segmentIndex] || trajectory[segmentIndex].xyz.size() < 2) continue;
                const size_t begin = segmentOffsets[segmentIndex];
                const size_t end = begin + trajectory[segmentIndex].xyz.size();
                for (size_t index = begin; index < end && index < displayPoints.size() && index < faceIndices.size(); ++index) {
                    const int faceIndex = faceIndices[index];
                    if (faceIndex < 0 || faceIndex >= static_cast<int>(result.mesh_faces.size())) continue;
                    const auto& face = result.mesh_faces[faceIndex];
                    if (face.minCoeff() < 0 || face.maxCoeff() >= static_cast<int>(result.mesh_vertices.size())) continue;
                    Eigen::Vector3d normal = (result.mesh_vertices[face[1]] - result.mesh_vertices[face[0]])
                        .cross(result.mesh_vertices[face[2]] - result.mesh_vertices[face[0]]);
                    if (normal.squaredNorm() < 1.0e-12) continue;
                    normal.normalize();
                    const Eigen::Vector3d surfacePoint = index < closestPoints.size() ? closestPoints[index] : sourcePoints[index];
                    if (normal.dot(surfacePoint - meshCenter) < 0.0) normal = -normal;
                    displayPoints[index] = surfacePoint + normal * lift;
                }
            }
        }
    }

    PublicData::PublicDataRender::clearMeshData(render.trajectoryData);
    const auto appendVertex = [&render](const Eigen::Vector3d& point, float r, float g, float b) {
        const unsigned int index = static_cast<unsigned int>(render.trajectoryData.m_vertices.size() / 7);
        render.trajectoryData.m_vertices.insert(render.trajectoryData.m_vertices.end(), {
            static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()), r, g, b, 1.0f });
        return index;
    };
    const auto segmentColor = [](const SprayTrajPlan::TrajectorySegment& segment) {
        // Scan regions use distinct cool/warm colors.  Non-scan motion is
        // intentionally high contrast so it cannot be mistaken for spraying.
        if (segment.kind == "SEAM_TRANSITION") return Eigen::Vector3f(0.80f, 0.10f, 0.90f); // magenta
        if (segment.kind == "OVERTRAVEL") return Eigen::Vector3f(1.00f, 0.82f, 0.00f);      // yellow
        if (segment.kind == "CONNECTION" || segment.kind == "TRANSITION")
            return Eigen::Vector3f(0.68f, 0.35f, 0.95f);                                     // violet
        if (segment.region == "LOWER") return Eigen::Vector3f(1.00f, 0.20f, 0.08f);          // red-orange
        if (segment.region == "UPPER") return Eigen::Vector3f(0.08f, 0.72f, 1.00f);          // cyan
        if (segment.region == "CENTER") return Eigen::Vector3f(0.15f, 0.95f, 0.38f);         // green
        return Eigen::Vector3f(1.00f, 1.00f, 1.00f);                                          // unknown scan
    };
    for (const auto& segment : trajectory) {
        if (segment.xyz.size() < 2) continue;
        const size_t offset = segmentOffsets[&segment - trajectory.data()];
        const Eigen::Vector3f color = segmentColor(segment);
        for (size_t i = 1; i < segment.xyz.size(); ++i) {
            const unsigned int a = appendVertex(displayPoints[offset + i - 1], color.x(), color.y(), color.z());
            const unsigned int bIndex = appendVertex(displayPoints[offset + i], color.x(), color.y(), color.z());
            render.trajectoryData.m_indices.push_back(a);
            render.trajectoryData.m_indices.push_back(bIndex);
        }
    }
    if (render.trajectoryData.m_indices.empty()) return false;
    render.isVisible = true;
    render.needsUpdate = false;
    openGLView_->makeCurrent();
    render.trajectoryModel->import(render.trajectoryData);
    openGLView_->doneCurrent();
    refreshDataDisplay();
    return true;
}

void RibbonWidget::onStartUvMapClicked()
{
    const QString path = QFileDialog::getOpenFileName(this, tr("Select UV mesh or OptCuts cache"), QString(),
        tr("UV input (*.stl *.obj *.ply *.off *.json)"));
    if (path.isEmpty()) return;
    const SprayTrajPlan::TrajectoryParams params = uvParameters();
    if (!m_uvSession->startUvMap(*m_uvPipeline, path.toStdString(), params)) {
        QMessageBox::critical(this, tr("UV Mapping"), tr("Start UV Map failed. Check the input topology and OptCuts configuration."));
    } else {
        displayUvSourceModel();
        emit uvStartMapRequested();
    }
    updateUvCommandState();
}

void RibbonWidget::onStartUvTrajectoryClicked()
{
    const SprayTrajPlan::TrajectoryParams params = uvParameters();
    if (!m_uvSession->startGetTraj(*m_uvPipeline, params)) QMessageBox::critical(this, tr("UV Mapping"), tr("Start Get Traj failed."));
    else { renderUvTrajectory(); emit uvStartTrajectoryRequested(); }
    updateUvCommandState();
}

void RibbonWidget::onStartUvOptimizeClicked()
{
    if (m_uvOptimizationRunning.exchange(true)) return;
    if (m_uvOptimizationThread.joinable()) m_uvOptimizationThread.join();
    const SprayTrajPlan::TrajectoryParams params = uvParameters();
    auto* progress = new QProgressDialog(tr("Preparing trajectory"), QString(), 0, 100, this);
    m_uvOptimizationProgress = progress;
    progress->setWindowTitle(tr("UV trajectory optimization"));
    progress->setWindowModality(Qt::NonModal);
    progress->setCancelButton(nullptr);
    progress->setMinimumDuration(0);
    progress->setAutoClose(false);
    progress->show();
    updateUvCommandState();

    // All geometry optimization stays off the GUI thread.  The callback only
    // copies a snapshot and posts rendering work back to this QObject's thread.
    m_uvOptimizationThread = std::thread([this, params]() {
        m_uvPipeline->setOptimizationProgressCallback([this](int percent, const std::string& stage,
            const std::vector<SprayTrajPlan::TrajectorySegment>& trajectory) {
            const QString label = QString::fromStdString(stage);
            QMetaObject::invokeMethod(this, [this, percent, label, trajectory]() {
                if (!m_uvOptimizationRunning.load()) return;
                if (m_uvOptimizationProgress) {
                    m_uvOptimizationProgress->setLabelText(label);
                    m_uvOptimizationProgress->setValue(percent);
                }
                m_uvPreviewTrajectory = trajectory;
                renderUvTrajectory(trajectory);
                if (openGLView_) openGLView_->update();
                if (forceRefreshHook_) forceRefreshHook_();
                emit uvOptimizationProgress(percent);
                emit uvOptimizationTelemetry(label);
            }, Qt::QueuedConnection);
        });
        const bool optimized = m_uvSession->startOptimize(*m_uvPipeline, params);
        m_uvPipeline->setOptimizationProgressCallback({});
        QMetaObject::invokeMethod(this, [this, optimized]() {
            m_uvOptimizationRunning.store(false);
            if (m_uvOptimizationProgress) {
                m_uvOptimizationProgress->setValue(100);
                m_uvOptimizationProgress->close();
                m_uvOptimizationProgress->deleteLater();
            }
            if (!optimized) {
                QMessageBox::critical(this, tr("UV Mapping"), tr("Start Optimize failed."));
            } else {
                m_uvPreviewTrajectory = m_uvPipeline->getTrajectory();
                renderUvTrajectory();
                emit uvStartOptimizeRequested();
            }
            updateUvCommandState();
        }, Qt::QueuedConnection);
    });
}

void RibbonWidget::onExportUvTrajectoryClicked()
{
    const QString path = QFileDialog::getSaveFileName(this, tr("Export PAQ trajectory"), QString(), tr("PAQ files (*.txt)"));
    if (path.isEmpty()) return;
    const SprayTrajPlan::TrajectoryParams params = uvParameters();
    if (!m_uvSession->exportPaq(*m_uvPipeline, path.toStdString(), params)) QMessageBox::critical(this, tr("UV Mapping"), tr("Export Trajectory failed.")); else emit uvExportRequested();
    updateUvCommandState();
}

// ================== Data Setters ==================
void RibbonWidget::setTrajPlanModel(ModelTrajPlan::IBurTrajPlan_spointer model)
{
    trajPlanModel_ = model;
}

void RibbonWidget::setGeneralData(ModelTrajPlan::GeneralData* generalData)
{
    generalData_ = generalData;
}

void RibbonWidget::setRenderData(ModelTrajPlan::RenderData* renderData)
{
    renderData_ = renderData;
}

void RibbonWidget::setTrajOptimizationData(ModelTrajPlan::TrajOptimizationData* trajOptimizationData)
{
    trajOptimizationData_ = trajOptimizationData;
}

void RibbonWidget::setPublicModelData(PublicData::PublicModelData* publicModelData)
{
    publicModelData_ = publicModelData;
}

void RibbonWidget::setPublicTrajData(PublicData::TrajData* publicTrajData)
{
    publicTrajData_ = publicTrajData;
}

void RibbonWidget::setPublicDataRender(PublicData::PublicDataRender* publicDataRender)
{
    publicDataRender_ = publicDataRender;
}

void RibbonWidget::setOpenGLView(QOpenGLWidget* openGLView)
{
    openGLView_ = openGLView;
    if (openGLView_ && !thicknessOverlay_) {
        thicknessOverlay_ = new ThicknessOverlay(openGLView_);
        thicknessOverlay_->hide();
    }
}

void RibbonWidget::setForceRefreshHook(const std::function<void()>& cb)
{
    forceRefreshHook_ = cb;
}

void RibbonWidget::setPropertyPanel(QWidget* propertyPanel)
{
    propertyPanel_ = propertyPanel;
}

void RibbonWidget::setWorkspaceActive(bool active)
{
    m_hasActiveWorkspace = active;
    if (m_uvSession) updateUvCommandState();
}

void RibbonWidget::generateUvTrajectory()
{
    onStartUvTrajectoryClicked();
}

const SprayTrajPlan::ObjUVData* RibbonWidget::uvMapData() const
{
    return m_uvPipeline ? m_uvPipeline->getOptCutsResult().uv_data.get() : nullptr;
}

const std::vector<SprayTrajPlan::TrajectorySegment>& RibbonWidget::uvTrajectory() const
{
    static const std::vector<SprayTrajPlan::TrajectorySegment> empty;
    if (!m_uvPreviewTrajectory.empty()) return m_uvPreviewTrajectory;
    return m_uvPipeline ? m_uvPipeline->getTrajectory() : empty;
}

void RibbonWidget::importModel()
{
    onImportModelClicked();
}

void RibbonWidget::importTrajectory()
{
    onImportTrajectoryClicked();
}

void RibbonWidget::importRobot()
{
    onImportRobotClicked();
}

void RibbonWidget::importTool()
{
    onImportCollisionModelClicked();
}

void RibbonWidget::clearModel()
{
    if (m_currentGenerationMethod == "UV Mapping" && m_uvSession) {
        m_uvSession->clearModel(*m_uvPipeline);
        updateUvCommandState();
    }
    onClearModelClicked();
    emit clearModelClicked();
}

void RibbonWidget::clearTrajectory()
{
    if (m_currentGenerationMethod == "UV Mapping" && m_uvSession) {
        m_uvSession->clearTrajectory(*m_uvPipeline);
        updateUvCommandState();
    }
    onClearTrajectoryClicked();
    emit clearTrajectoryClicked();
}

void RibbonWidget::clearConnection()
{
    if (m_currentGenerationMethod == "UV Mapping" && m_uvSession) {
        m_uvSession->clearConnection(*m_uvPipeline);
        updateUvCommandState();
    }
    onClearConnectionClicked();
    emit clearConnectionClicked();
}

void RibbonWidget::importCollisionWorkpiece()
{
    onImportWorkpieceClicked();
    emit importWorkpieceClicked();
}

void RibbonWidget::importCollisionModel()
{
    onImportCollisionModelClicked();
    emit importCollisionModelClicked();
}

void RibbonWidget::runCollisionOptimization()
{
    onRunCollisionOptimizationClicked();
    emit runOptimizationClicked();
}

void RibbonWidget::clearCollisionWorkpiece()
{
    QMessageBox::information(this, "Info", "Clear collision workpiece - To be implemented");
    emit clearCollisionWorkpieceClicked();
}

void RibbonWidget::clearCollisionRobot()
{
    QMessageBox::information(this, "Info", "Clear collision robot - To be implemented");
    emit clearCollisionRobotClicked();
}

void RibbonWidget::clearCollisionModel()
{
    onClearCollisionModelClicked();
    emit clearCollisionModelClicked();
}

void RibbonWidget::clearOptimizationResult()
{
    QMessageBox::information(this, "Info", "Clear optimization result - To be implemented");
    emit clearOptimizationResultClicked();
}

void RibbonWidget::importTransformJson()
{
    onImportTransformJsonClicked();
    emit importMatrixJsonClicked();
}

void RibbonWidget::exportTransformJson()
{
    onExportTransformJsonClicked();
    emit exportTransformSettingsClicked();
}

void RibbonWidget::loadExtraTransformModel()
{
    QMessageBox::information(this, "Info", "Load extra model - To be implemented");
    emit loadExtraModelClicked();
}

void RibbonWidget::applyTransform()
{
    onApplyTransformClicked();
    emit applyTransformClicked();
}

void RibbonWidget::resetTransform()
{
    onResetTransformClicked();
    emit resetTransformClicked();
}

void RibbonWidget::importDepositionModel()
{
    QMessageBox::information(this, "Info", "Import deposition model - To be implemented");
    emit importDepositionModelClicked();
}

void RibbonWidget::runThicknessSimulation()
{
    onStartThicknessSimulationClicked();
}

void RibbonWidget::clearThicknessResult()
{
    onClearThicknessResultClicked();
}

void RibbonWidget::clearDepositionModel()
{
    QMessageBox::information(this, "Info", "Clear deposition model - To be implemented");
    emit clearDepositionModelClicked();
}

void RibbonWidget::clearThicknessInputs()
{
    QMessageBox::information(this, "Info", "Clear thickness inputs - To be implemented");
    emit clearThicknessInputsClicked();
}

// ================== Helper Methods ==================
void RibbonWidget::refreshDataDisplay()
{
    if (openGLView_) {
        openGLView_->update();
    }

    if (forceRefreshHook_) {
        forceRefreshHook_();
    }
}

// ================== Trajectory Operation Implementations ==================

void RibbonWidget::onConfirmSlicingClicked()
{
    // Implement slicing confirmation logic
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }

    if (!generalData_) {
        QMessageBox::warning(this, "Warning", "General data is not initialized.");
        return;
    }

    // Check if STL file has been processed
    if (!trajPlanModel_->isSTLProcessed(*generalData_)) {
        QMessageBox::warning(this, "Warning", "STL file not processed yet. Please load an STL file first.");
        return;
    }

    // Clear any custom Z positions (use uniform intervals instead)
    std::vector<double> emptyZs;
    trajPlanModel_->setCustomZPositions(*generalData_, emptyZs);

    // Keep original data visible during slicing
    trajPlanModel_->setKeepOriginalVisibleAfterPreprocess(true);
    if (renderData_) {
        trajPlanModel_->setOriginalDataVisibility(*renderData_, true);
    }

    // Clear previous cutting planes before generating new ones
    trajPlanModel_->clearCuttingPlanes(*generalData_);

    // Confirming a new slice configuration invalidates every downstream
    // artifact. Clear the public cache and hide the old private render models
    // before rebuilding, otherwise frames/connection segments from the old
    // plane set remain visible until a new trajectory is generated.
    if (publicTrajData_) {
        publicTrajData_->clear();
    }
    if (publicDataRender_) {
        auto& publicTrajectoryRender = publicDataRender_->m_PublicTrajectoryRenderData;
        publicTrajectoryRender.isVisible = false;
        publicTrajectoryRender.needsUpdate = true;
        PublicData::PublicDataRender::clearMeshData(publicTrajectoryRender.trajectoryData);
    }
    if (renderData_) {
        trajPlanModel_->setTrajectoryVisibility(*renderData_, false);
        trajPlanModel_->setConnectionVisibility(*renderData_, false);
        renderData_->slicing().needsUpdate = true;
        renderData_->trajectory().needsUpdate = true;
        renderData_->connection().needsUpdate = true;
    }

    // Generate new cutting planes
    if (trajPlanModel_->generateCuttingPlanes(*generalData_, renderData_)) {
        std::vector<ModelTrajPlan::CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

        // Show cutting planes and boundaries in render view
        if (renderData_) {
            trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);

            // Show boundary if detected during generateCuttingPlanes
            if (trajPlanModel_->isBoundaryDetected(*generalData_)) {
                trajPlanModel_->setBoundaryVisibility(*renderData_, true);
            }
        }

        // Refresh OpenGL display
        refreshDataDisplay();

        QMessageBox::information(this, "Success",
            QString("Slicing completed! Generated %1 cutting planes.\nYou can now generate trajectory.")
            .arg(cuttingPlanes.size()));

        emit confirmSlicingClicked();
    } else {
        QMessageBox::critical(this, "Error",
            "Uniform slicing failed! Please check if the file is loaded correctly and parameters are valid.");
    }
}

void RibbonWidget::onCancelSlicingClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }

    if (!generalData_) {
        QMessageBox::warning(this, "Warning", "General data is not initialized.");
        return;
    }

    const QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        tr("Confirm Clear Data"),
        tr("Are you sure you want to clear all loaded data?\n\n"
           "This will remove:\n"
           "- All loaded files\n"
           "- Generated slicing planes\n"
           "- Trajectory data\n"
           "- All rendered models\n\n"
           "You will need to reload files to start over."),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        return;
    }

    trajPlanModel_->clear(*generalData_);

    if (renderData_) {
        trajPlanModel_->setConnectionVisibility(*renderData_, false);
        trajPlanModel_->setTrajectoryVisibility(*renderData_, false);
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, false);
        trajPlanModel_->setBoundaryVisibility(*renderData_, false);
        trajPlanModel_->setOriginalDataVisibility(*renderData_, false);

        renderData_->slicing().needsUpdate = true;
        renderData_->trajectory().needsUpdate = true;
        renderData_->boundary().needsUpdate = true;
        renderData_->connection().needsUpdate = true;
        renderData_->stl().needsUpdate = true;
    }

    refreshDataDisplay();
    QMessageBox::information(
        this,
        tr("Data Cleared"),
        tr("All data has been cleared successfully.\n\nPlease reload files to start over."));
    emit cancelSlicingClicked();
}

// Generate Trajectory button clicked
void RibbonWidget::onGenerateTrajectoryClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }

    if (!generalData_) {
        QMessageBox::warning(this, "Warning", "General data is not initialized.");
        return;
    }

    if (!publicTrajData_) {
        QMessageBox::warning(this, "Warning", "Public trajectory data is not initialized.");
        return;
    }

    try {
        if (auto* propPanel = qobject_cast<PropertyPanel*>(propertyPanel_)) {
            const double optimalSpeed = propPanel->getTrajectoryOptimalSpeed();
            if (std::isfinite(optimalSpeed) && optimalSpeed > 0.0) {
                trajPlanModel_->setnormalVelocity_(*generalData_, optimalSpeed);
                trajPlanModel_->settransitionVelocity_(*generalData_, optimalSpeed);
                qInfo() << "[TRAJECTORY] Applying Optimal Speed:" << optimalSpeed << "mm/s";
                propPanel->logRequested(QString("[Trajectory] Applying Optimal Speed: %1 mm/s")
                    .arg(optimalSpeed, 0, 'f', 3));
            }
        }

        std::vector<ModelTrajPlan::CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
        if (cuttingPlanes.empty()) {
            QMessageBox::warning(this, "Warning", "No slice data! Please confirm slicing first.");
            return;
        }

        int sizeN = 0;
        double z1Offset = 0.0;
        double z2Offset = 0.0;
        double offsetDistance = 0.0;
        trajPlanModel_->getSlicingSize(*generalData_, sizeN);
        trajPlanModel_->getZ1Offset(*generalData_, z1Offset);
        trajPlanModel_->getZ2Offset(*generalData_, z2Offset);
        trajPlanModel_->getOffsetDistance(*generalData_, offsetDistance);

        if (!trajPlanModel_->generateMATLABTrajectory(*generalData_, sizeN, z1Offset, z2Offset, offsetDistance, *publicTrajData_)) {
            QMessageBox::critical(this, "Error", "Trajectory generation failed. Please check slicing parameters and debug output.");
            return;
        }

        std::vector<ModelTrajPlan::TrajectoryPoint> trajectoryPoints;
        trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
        if (trajectoryPoints.empty()) {
            QMessageBox::critical(this, "Error", "Trajectory generation completed but no trajectory points were created.");
            return;
        }

        if (publicDataRender_ && openGLView_) {
            publicDataRender_->m_PublicTrajectoryRenderData.isVisible = true;
            publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;
            openGLView_->makeCurrent();
            PublicData::PublicDataRender::drawPublicTrajectory(
                publicDataRender_->m_PublicTrajectoryRenderData, *publicTrajData_);
            openGLView_->doneCurrent();
        }

        if (renderData_) {
            trajPlanModel_->setTrajectoryVisibility(*renderData_, true);
        }

        refreshDataDisplay();
        QMessageBox::information(this, "Success",
            QString("Trajectory generated successfully. Total points: %1").arg(trajectoryPoints.size()));
        emit generateTrajectoryClicked();
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Trajectory generation failed: %1").arg(e.what()));
    }
}

// Connect Trajectory button clicked
void RibbonWidget::onConnectTrajectoryClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }

    if (!generalData_) {
        QMessageBox::warning(this, "Warning", "General data is not initialized.");
        return;
    }

    std::vector<ModelTrajPlan::TrajectoryPoint> trajectoryPoints;
    trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
    if (trajectoryPoints.empty()) {
        QMessageBox::warning(this, "Warning", "No trajectory points. Please generate trajectory first.");
        return;
    }

    if (!trajPlanModel_->connectTrajectoryPoints(*generalData_, ModelTrajPlan::ConnectionStrategy::BowPath)) {
        QMessageBox::critical(this, "Error", "Trajectory connection failed.");
        return;
    }

    syncConnectedTrajectoryToPublic(*generalData_, publicTrajData_);

    if (renderData_) {
        trajPlanModel_->initializeRenderer(*renderData_);
        trajPlanModel_->setConnectionVisibility(*renderData_, true);
    }

    refreshDataDisplay();
    QMessageBox::information(this, "Success", "Trajectory connected successfully.");
    emit connectTrajectoryClicked();
}
// Thickness prediction implementation for RibbonWidget
// This file will be appended to RibbonWidget.cpp

// ================== Thickness Prediction Implementations ==================

void RibbonWidget::onStartThicknessSimulationClicked()
{
    PropertyPanel* propPanel = qobject_cast<PropertyPanel*>(propertyPanel_);
    if (!propPanel) {
        QMessageBox::warning(this, "Error", "PropertyPanel not available");
        return;
    }

    // Validate data availability
    if (!publicModelData_ || !publicTrajData_ || !publicDataRender_ || !thicknessData_) {
        QMessageBox::warning(this, "Error", "Thickness data not initialized");
        return;
    }

    if (!publicModelData_->hasValidData()) {
        QMessageBox::warning(this, "Error", "Please import a workpiece model first");
        return;
    }

    const PublicData::PublicModel& publicModel = publicModelData_->getModel();
    const std::vector<float>* modelVertices = &publicModel.m_vertices;
    const std::vector<float>* modelNormals = &publicModel.m_normals;
    const std::vector<unsigned int>* modelIndices = &publicModel.m_indices;

    if (!modelVertices || modelVertices->empty()) {
        QMessageBox::warning(this, "Error", "Workpiece model has no vertices");
        return;
    }

    const PublicData::TrajectoryPoints& normalTrajectory = publicTrajData_->getTrajectoryPoints();
    const PublicData::TrajectoryPoints& spiralTrajectory = publicTrajData_->getSpiralTrajectoryPoints();
    const bool useSpiralTrajectory = normalTrajectory.empty() && !spiralTrajectory.empty();
    const PublicData::TrajectoryPoints& publicTrajectory =
        useSpiralTrajectory ? spiralTrajectory : normalTrajectory;

    if (publicTrajectory.size() < 2) {
        QMessageBox::warning(this, "Error", "Please load or generate at least two trajectory points");
        return;
    }

    const QMessageBox::StandardButton confirmation = QMessageBox::question(
        this,
        tr("Confirm Thickness Model"),
        tr("是否采用“%1”模型进行仿真？").arg(m_currentThicknessMethod),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::Yes);
    if (confirmation != QMessageBox::Yes) {
        return;
    }

    const size_t vertexCount = modelVertices->size() / 3;
    if (vertexCount == 0) {
        QMessageBox::warning(this, "Error", "Workpiece model has no vertices");
        return;
    }

    {
        Eigen::Vector3d modelMin((*modelVertices)[0], (*modelVertices)[1], (*modelVertices)[2]);
        Eigen::Vector3d modelMax = modelMin;
        for (size_t i = 1; i < vertexCount; ++i) {
            const Eigen::Vector3d p((*modelVertices)[i * 3], (*modelVertices)[i * 3 + 1], (*modelVertices)[i * 3 + 2]);
            modelMin = modelMin.cwiseMin(p);
            modelMax = modelMax.cwiseMax(p);
        }
        printf("[THICKNESS INPUT] model vertices=%zu bounds min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)\n",
               vertexCount, modelMin.x(), modelMin.y(), modelMin.z(), modelMax.x(), modelMax.y(), modelMax.z());
    }

    {
        Eigen::Vector3d trajMin = publicTrajectory.begin()->second.pose.block<3, 1>(0, 3);
        Eigen::Vector3d trajMax = trajMin;
        double minDistance = std::numeric_limits<double>::max();
        double maxDistance = 0.0;
        double minVelocity = std::numeric_limits<double>::max();
        double maxVelocity = 0.0;
        for (const auto& item : publicTrajectory) {
            const Eigen::Vector3d p = item.second.pose.block<3, 1>(0, 3);
            trajMin = trajMin.cwiseMin(p);
            trajMax = trajMax.cwiseMax(p);
            minDistance = std::min(minDistance, item.second.spray_distance);
            maxDistance = std::max(maxDistance, item.second.spray_distance);
            minVelocity = std::min(minVelocity, item.second.velocity);
            maxVelocity = std::max(maxVelocity, item.second.velocity);
        }
        const Eigen::Matrix4d& firstPose = publicTrajectory.begin()->second.pose;
        printf("[THICKNESS INPUT] trajectory points=%zu bounds min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f) sprayDistance=(%.3f..%.3f) velocity=(%.3f..%.3f) mm/s\n",
               publicTrajectory.size(), trajMin.x(), trajMin.y(), trajMin.z(), trajMax.x(), trajMax.y(), trajMax.z(), minDistance, maxDistance, minVelocity, maxVelocity);
        propPanel->logRequested(QString("[Thickness] Input trajectory speed: %1..%2 mm/s")
            .arg(minVelocity, 0, 'f', 3)
            .arg(maxVelocity, 0, 'f', 3));
        printf("[THICKNESS INPUT] first axes X=(%.3f, %.3f, %.3f) Y=(%.3f, %.3f, %.3f) Z=(%.3f, %.3f, %.3f)\n",
               firstPose(0, 0), firstPose(1, 0), firstPose(2, 0), firstPose(0, 1), firstPose(1, 1), firstPose(2, 1), firstPose(0, 2), firstPose(1, 2), firstPose(2, 2));
    }

    // Read UI parameters from PropertyPanel. Execution still belongs to RibbonWidget.
    double sprayWidth = propPanel->getThicknessSprayWidth();
    double depositionEfficiency = propPanel->getThicknessDepositionEfficiency() / 100.0;
    double targetThickness = propPanel->getThicknessTargetThickness();
    double qualityFactor = propPanel->getThicknessQualityFactor();
    double defaultVelocity = propPanel->getThicknessDefaultVelocity();
    (void)propPanel->getThicknessDepositionFunctionIndex();

    // The Gaussian menu item deliberately selects the empirical Gaussian
    // implemented in GaussianModel.cpp, rather than the legacy normalized
    // width-only distribution.
    ModelTrajPlan::DepositionModelType selectedModel = ModelTrajPlan::DepositionModelType::GaussianCalibrated;
    if (m_currentThicknessMethod == QStringLiteral("zigong")) {
        selectedModel = ModelTrajPlan::DepositionModelType::Zigong;
    } else if (m_currentThicknessMethod == QStringLiteral("420")) {
        selectedModel = ModelTrajPlan::DepositionModelType::Model420;
    } else if (m_currentThicknessMethod == QStringLiteral("DoubleBeta")) {
        selectedModel = ModelTrajPlan::DepositionModelType::DoubleBeta;
    } else if (m_currentThicknessMethod == QStringLiteral("SkewedDoubleBeta")) {
        selectedModel = ModelTrajPlan::DepositionModelType::SkewedDoubleBeta;
    }

    auto& params = thicknessData_->getInputParams();
    params.sprayWidth = sprayWidth;
    params.depositionEfficiency = depositionEfficiency;
    params.targetThickness = targetThickness;
    params.qualityFactor = qualityFactor;
    params.defaultVelocity = defaultVelocity;
    params.lowerTolerance = -0.1 * targetThickness;
    params.upperTolerance = 0.1 * targetThickness;
    params.modelType = selectedModel;

    // Prepare analysis input
    ModelTrajPlan::ThicknessAnalysisInput input;
    input.gaussian = ModelTrajPlan::ThicknessAnalyzer::makeCalibratedGaussianParameters(
        sprayWidth,
        targetThickness,
        depositionEfficiency);
    input.modelType = selectedModel;
    input.deposition.h = params.sprayDistance;
    input.deposition.theta = params.incidenceAngle;
    input.deposition.gamma = params.gunOrientationAngle;
    input.deposition.alpha_x = params.sprayAngleX;
    input.deposition.alpha_y = params.sprayAngleY;
    input.deposition.v = defaultVelocity;
    input.deposition.beta1 = params.beta1;
    input.deposition.beta2 = params.beta2;
    input.deposition.beta3 = params.beta3;
    input.deposition.k_epsilon = params.k_epsilon;
    input.deposition.T_max = params.T_max;
    input.deposition.delta = params.skewness;

    // Debug: Print Gaussian parameters
    const char* selectedModelName = "Unknown";
    switch (selectedModel) {
    case ModelTrajPlan::DepositionModelType::DoubleBeta: selectedModelName = "DoubleBeta"; break;
    case ModelTrajPlan::DepositionModelType::GaussianCalibrated: selectedModelName = "GaussianCalibrated"; break;
    case ModelTrajPlan::DepositionModelType::SkewedDoubleBeta: selectedModelName = "SkewedDoubleBeta"; break;
    case ModelTrajPlan::DepositionModelType::Zigong: selectedModelName = "Zigong"; break;
    case ModelTrajPlan::DepositionModelType::Model420: selectedModelName = "Model420"; break;
    default: break;
    }
    qInfo() << "[THICKNESS] Model:" << selectedModelName
            << "GPU modelType=" << static_cast<int>(selectedModel);
    qInfo() << "[THICKNESS] Input parameters:";
    qInfo() << "  sprayWidth:" << sprayWidth << "mm";
    qInfo() << "  targetThickness:" << targetThickness << "mm";
    qInfo() << "  depositionEfficiency:" << depositionEfficiency;
    qInfo() << "  qualityFactor:" << qualityFactor;
    qInfo() << "  defaultVelocity:" << defaultVelocity << "mm/s";
    qInfo() << "[THICKNESS] Gaussian parameters:";
    qInfo() << "  peakHeight:" << input.gaussian.peakHeight;
    qInfo() << "  effectiveRadius:" << input.gaussian.effectiveRadius << "mm";
    qInfo() << "  sigma:" << input.gaussian.sigma;
    qInfo() << "  efficiency:" << input.gaussian.efficiency;
    qInfo() << "  sprayWidth:" << input.gaussian.sprayWidth << "mm";

    // The fitted 420/zigong footprints are angular distributions.  At the
    // trajectory's 20 mm working distance their useful lateral support is
    // only roughly 4-12 mm, so using a 50 mm Gaussian width here would leave
    // interpolation gaps and produce visible scan-line banding. Keep the
    // Gaussian sampling unchanged and use a 10 mm control width for the
    // calibrated models. With qualityFactor=1 this gives a 5 mm step, which
    // is sufficient for the current 5 mm raster pitch without creating
    // millions of unnecessary samples on long trajectories.
    input.interpolation.sprayWidth = selectedModel == ModelTrajPlan::DepositionModelType::Gaussian
        ? input.gaussian.sprayWidth
        : std::min(input.gaussian.sprayWidth, 10.0);
    input.interpolation.qualityFactor = qualityFactor;
    input.interpolation.defaultVelocity = defaultVelocity;
    input.targetThickness = targetThickness;
    input.lowerTolerance = -0.1 * targetThickness;
    input.upperTolerance = 0.1 * targetThickness;

    bool waitCursorActive = false;
    auto restoreWaitCursor = [&waitCursorActive]() {
        if (waitCursorActive) {
            QApplication::restoreOverrideCursor();
            waitCursorActive = false;
        }
    };

    try {
        // This path builds a render mesh on the GUI thread, so give immediate
        // visual feedback while the event loop is occupied.
        QApplication::setOverrideCursor(Qt::WaitCursor);
        waitCursorActive = true;
        QElapsedTimer totalTimer;
        totalTimer.start();
        // Build surface samples from vertices
        input.surfaceSamples.reserve(vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            ModelTrajPlan::SurfaceSample sample;
            sample.position = Eigen::Vector3d(
                (*modelVertices)[i * 3 + 0],
                (*modelVertices)[i * 3 + 1],
                (*modelVertices)[i * 3 + 2]);

            if (modelNormals && modelNormals->size() >= (i + 1) * 3) {
                sample.normal = Eigen::Vector3d(
                    (*modelNormals)[i * 3 + 0],
                    (*modelNormals)[i * 3 + 1],
                    (*modelNormals)[i * 3 + 2]);
                if (sample.normal.norm() < 1e-9) {
                    sample.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
                } else {
                    sample.normal.normalize();
                }
            } else {
                sample.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
            }
            sample.sourceVertexIndex = static_cast<int>(i);
            input.surfaceSamples.push_back(sample);
        }
        if (modelIndices && modelIndices->size() >= 3) {
            input.meshIndices = *modelIndices;
            printf("[THICKNESS INPUT] mesh triangles=%zu; first-hit visibility enabled\n",
                   input.meshIndices.size() / 3);
        } else {
            printf("[THICKNESS INPUT] mesh indices unavailable; using legacy one-sided incidence\n");
        }

        // Build trajectory from public trajectory data
        input.trajectory.reserve(publicTrajectory.size());
        double lastTimestamp = 0.0;
        bool firstPoint = true;
        int skippedInvalidPoints = 0;

        for (const auto& kv : publicTrajectory) {
            Eigen::Matrix4d pose = kv.second.pose;
            bool poseValid = true;

            // Validate pose matrix
            if (std::abs(pose(3, 0)) > 0.01 || std::abs(pose(3, 1)) > 0.01 ||
                std::abs(pose(3, 2)) > 0.01 || std::abs(pose(3, 3) - 1.0) > 0.01) {
                poseValid = false;
            }

            if (poseValid) {
                for (int i = 0; i < 3; ++i) {
                    if (!std::isfinite(pose(i, 3))) {
                        poseValid = false;
                        break;
                    }
                }
            }

            if (poseValid) {
                for (int row = 0; row < 3; ++row) {
                    for (int col = 0; col < 3; ++col) {
                        if (!std::isfinite(pose(row, col))) {
                            poseValid = false;
                            break;
                        }
                    }
                    if (!poseValid) {
                        break;
                    }
                }
            }

            if (poseValid && !std::isfinite(kv.second.spray_distance)) {
                poseValid = false;
            }

            if (!poseValid) {
                ++skippedInvalidPoints;
                continue;
            }

            ModelTrajPlan::TrajectoryPoint point;
            point.pose = Eigen::Matrix4d::Identity();
            point.pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0);
            point.pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3);
            point.spray_distance = kv.second.spray_distance;
            point.trajectory_index = kv.second.spray_pass_id;
            point.timestamp = firstPoint ? 0.0 : kv.first;
            point.velocity = kv.second.velocity > input.interpolation.minVelocity
                ? kv.second.velocity
                : input.interpolation.defaultVelocity;
            point.acceleration = 0.0;
            point.point_type = firstPoint ? 1 : 0;

            if (!firstPoint && point.timestamp <= lastTimestamp) {
                point.timestamp = lastTimestamp + 1.0;
            }
            lastTimestamp = point.timestamp;
            firstPoint = false;
            input.trajectory.push_back(point);
        }

        if (input.trajectory.size() < 2) {
            restoreWaitCursor();
            QMessageBox::warning(this, "Error", "Not enough valid trajectory points after filtering");
            return;
        }

        // Calculate path length and estimate dense points
        double pathLength = 0.0;
        for (size_t i = 1; i < input.trajectory.size(); ++i) {
            // Travel between scan passes is not interpolated for thickness,
            // so it must not be included in the dense-point estimate.
            if (input.trajectory[i - 1].trajectory_index != input.trajectory[i].trajectory_index) {
                continue;
            }
            const Eigen::Vector3d p0 = input.trajectory[i - 1].pose.block<3, 1>(0, 3);
            const Eigen::Vector3d p1 = input.trajectory[i].pose.block<3, 1>(0, 3);
            pathLength += (p1 - p0).norm();
        }
        constexpr double kMaxSafeInterpolationPoints = 4000000.0;
        // Leave room for the original points and per-segment end points. This
        // makes the estimate conservative when ceil() is applied per segment.
        const double kTargetDensePoints = kMaxSafeInterpolationPoints * 0.85;
        const double interpolationPointBudget = kTargetDensePoints -
            2.0 * static_cast<double>(input.trajectory.size());
        if (pathLength > 0.0 && interpolationPointBudget > 0.0) {
            const double requiredMinStep = pathLength / interpolationPointBudget;
            if (std::isfinite(requiredMinStep) &&
                requiredMinStep > input.interpolation.minStep) {
                qInfo() << "[THICKNESS] Raising interpolation minimum step from"
                        << input.interpolation.minStep << "to" << requiredMinStep
                        << "mm to keep dense trajectory below" << kTargetDensePoints
                        << "points.";
                input.interpolation.minStep = requiredMinStep;
                input.interpolation.maxStep = std::max(
                    input.interpolation.maxStep, requiredMinStep);
            }
        }

        const double interpolationStep = ModelTrajPlan::LinearInterpolator::computeAdaptiveStepSize(
            input.interpolation.sprayWidth,
            input.interpolation.qualityFactor,
            input.interpolation.minStep,
            input.interpolation.maxStep);
        const double estimatedDensePoints = interpolationStep > 0.0
            ? std::ceil(pathLength / interpolationStep) + static_cast<double>(input.trajectory.size())
            : 0.0;

        if (estimatedDensePoints > kMaxSafeInterpolationPoints * 0.9) {
            qWarning() << "[THICKNESS] Dense-point estimate remains high:"
                       << estimatedDensePoints << "of" << kMaxSafeInterpolationPoints;
        }

        // Run thickness analysis
        ModelTrajPlan::ThicknessHeatmapResult result;
        if (!thicknessAnalyzer_) {
            restoreWaitCursor();
            QMessageBox::warning(this, "Error", "Thickness analyzer is not initialized");
            return;
        }
        QElapsedTimer computeTimer;
        computeTimer.start();
        bool gpuComputed = false;

        // GPU 诊断：第一次计算时输出详细信息
        static bool diagnosedOnce = false;
        if (!diagnosedOnce && openGLView_) {
            ModelTrajPlan::diagnoseGpuCapabilities(openGLView_);
            diagnosedOnce = true;
        }

        if (openGLView_) {
            openGLView_->makeCurrent();
            gpuComputed = ModelTrajPlan::computeThicknessOnGpu(openGLView_, input, result);
            openGLView_->doneCurrent();
        } else {
            qWarning() << "[THICKNESS] openGLView_ 为空，无法使用 GPU 加速";
        }

        qInfo() << "[THICKNESS] backend=" << (gpuComputed ? "GPU" : "CPU fallback")
                << "computeMs=" << computeTimer.elapsed()
                << "surfaces=" << input.surfaceSamples.size()
                << "trajectoryPoints=" << input.trajectory.size();
        if (!gpuComputed && !thicknessAnalyzer_->analyze(input, result)) {
            const QString interpolationFailure = QString::fromStdString(
                ModelTrajPlan::LinearInterpolator::lastThicknessFailureReason());
            QString errorMsg = "Thickness simulation failed";
            if (!interpolationFailure.isEmpty()) {
                errorMsg += "\nReason: " + interpolationFailure;
            }
            restoreWaitCursor();
            QMessageBox::critical(this, "Error", errorMsg);
            return;
        }
        qInfo() << "[THICKNESS] completed model=" << selectedModelName
                << "modelType=" << static_cast<int>(selectedModel)
                << "backend=" << (gpuComputed ? "GPU" : "CPU")
                << "range=" << result.statistics.minThickness
                << ".." << result.statistics.maxThickness;
        propPanel->logRequested(QString("[Thickness] Result average=%1, range=%2..%3")
            .arg(result.statistics.averageThickness, 0, 'f', 6)
            .arg(result.statistics.minThickness, 0, 'f', 6)
            .arg(result.statistics.maxThickness, 0, 'f', 6));
        auto& resultData = thicknessData_->getResultData();
        resultData.clear();

        // The heatmap keeps its numeric statistics and legend, but rendering
        // Build the same shared-vertex mesh used by the deposition result. A
        // zero displacement keeps the surface in place while preserving the
        // exact triangle/thickness correspondence for rendering and picking.
        resultData.thicknessValues = result.thicknessValues;
        resultData.colors = result.colors;
        resultData.hasValidResult = true;
        resultData.minThickness = result.statistics.minThickness;
        resultData.maxThickness = result.statistics.maxThickness;
        resultData.averageThickness = result.statistics.averageThickness;
        resultData.stdDeviation = result.statistics.stdDeviation;
        resultData.variance = result.statistics.stdDeviation * result.statistics.stdDeviation;
        resultData.coefficientOfVariation = std::abs(result.statistics.averageThickness) > 1e-12
            ? result.statistics.stdDeviation / std::abs(result.statistics.averageThickness) * 100.0
            : 0.0;
        resultData.underToleranceCount = result.statistics.underToleranceCount;
        resultData.overToleranceCount = result.statistics.overToleranceCount;
        // Convert colors to vertex colors for rendering
        std::vector<float> vertexColors;
        vertexColors.reserve(resultData.colors.size() * 3);
        for (const Eigen::Vector3d& color : resultData.colors) {
            vertexColors.push_back(static_cast<float>(color.x()));
            vertexColors.push_back(static_cast<float>(color.y()));
            vertexColors.push_back(static_cast<float>(color.z()));
        }

        // Update render with heatmap colors
        auto& thicknessRender = publicDataRender_->m_PublicThicknessRenderData;
        QElapsedTimer heatmapTimer;
        heatmapTimer.start();
        if (!PublicData::PublicDataRender::drawPublicThicknessHeatmap(
            thicknessRender,
            *publicModelData_,
            vertexColors,
            result.thicknessValues,
            0.0f)) {
            restoreWaitCursor();
            QMessageBox::critical(this, "Error", "Failed to prepare thickness heatmap mesh");
            return;
        }
        qInfo() << "[THICKNESS] heatmap mesh preparationMs=" << heatmapTimer.elapsed();

        // Use one absolute scale for every simulation. Per-result min/max
        // scaling makes different heatmaps look deceptively comparable.
        thicknessRender.minValue = 0.0;
        thicknessRender.maxValue = 1.0;
        thicknessRender.targetValue = input.targetThickness;
        thicknessRender.legendValues.clear();
        thicknessRender.legendColors.clear();
        for (int i = 0; i < 8; ++i) {
            const double t = i / 7.0;
            thicknessRender.legendValues.push_back(
                thicknessRender.minValue + (thicknessRender.maxValue - thicknessRender.minValue) * t);
            const Eigen::Vector3d color = thicknessAnalyzer_->colorMapJet(t);
            thicknessRender.legendColors.insert(thicknessRender.legendColors.end(), {
                static_cast<float>(color.x()), static_cast<float>(color.y()), static_cast<float>(color.z()) });
        }
        if (auto* overlay = static_cast<ThicknessOverlay*>(thicknessOverlay_)) {
            // Keep the overlay's colour-bar scale identical to the heatmap.
            // The statistics below it still show the measured values.
            overlay->setResult(0.0, 1.0,
                result.statistics.averageThickness, result.statistics.stdDeviation);
            overlay->show();
            overlay->raise();
        }

        PropertyPanel* displayPanel = qobject_cast<PropertyPanel*>(propertyPanel_);
        if (displayPanel) {
            // Once a heatmap is generated, show the deposition surface by
            // default and leave the original model hidden to avoid z-fighting.
            displayPanel->setShowModel(false);
        }
        publicDataRender_->m_PublicSTLRenderData.isVisible = displayPanel
            ? displayPanel->showModel()
            : true;
        // Keep the generated trajectory visible as an overlay while showing
        // the heatmap, so the user can verify coverage and spray direction.
        publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;
        thicknessRender.isVisible = displayPanel
            ? displayPanel->showDepositionResult()
            : true;

        if (renderData_ && trajPlanModel_) {
            const bool showTrajectory = displayPanel
                ? (displayPanel->showSprayGunPose() || displayPanel->showTrajectoryPoints() || displayPanel->showTrajectoryLine())
                : true;
            const bool showConnection = displayPanel
                ? displayPanel->showTrajectoryLine()
                : false;
            trajPlanModel_->setConnectionVisibility(*renderData_, showConnection);
            trajPlanModel_->setTrajectoryVisibility(*renderData_, showTrajectory);
            trajPlanModel_->setCuttingPlanesVisibility(*renderData_, false);
            trajPlanModel_->setBoundaryVisibility(*renderData_, false);
            trajPlanModel_->setOriginalDataVisibility(*renderData_, displayPanel ? displayPanel->showModel() : true);
        }

        refreshDataDisplay();

        // Do not enter a modal QMessageBox here. The OpenGL upload is queued
        // by refreshDataDisplay(); a nested modal event loop can make its
        // empty shell visible before its contents are painted.
        const auto& stats = result.statistics;
        QString successMsg = QString("Thickness heatmap generated successfully!\n\n"
            "Min: %1mm, Max: %2mm, Avg: %3mm\n"
            "Std Dev: %4mm\n"
            "Under tolerance: %5, Over tolerance: %6")
            .arg(stats.minThickness, 0, 'f', 4)
            .arg(stats.maxThickness, 0, 'f', 4)
            .arg(stats.averageThickness, 0, 'f', 4)
            .arg(stats.stdDeviation, 0, 'f', 4)
            .arg(stats.underToleranceCount)
            .arg(stats.overToleranceCount);

        qInfo() << "[THICKNESS] completed totalMs=" << totalTimer.elapsed();
        restoreWaitCursor();
		QTimer::singleShot(0, this, [this]() {
			if (!thicknessData_) {
				return;
			}
			const QString reportPath = writeThicknessReport(thicknessData_->getResultData(), openGLView_);
			if (!reportPath.isEmpty()) {
				qInfo() << "[THICKNESS] report generated:" << reportPath;
			}
		});
        QTimer::singleShot(0, this, [this, successMsg]() {
            const QPoint anchor = mapToGlobal(QPoint(width() / 2, 36));
            QToolTip::showText(anchor, successMsg, this, rect(), 6000);
        });

    } catch (const std::bad_alloc&) {
        restoreWaitCursor();
        QMessageBox::critical(this, "Error", "Out of memory during thickness simulation");
    } catch (const std::exception& e) {
        restoreWaitCursor();
        QMessageBox::critical(this, "Error", QString("Thickness simulation failed: %1").arg(e.what()));
    }

    // Emit signal for any listeners
    emit startThicknessSimulationClicked();
}

void RibbonWidget::onClearThicknessResultClicked()
{
    if (!publicModelData_ || !publicDataRender_) {
        return;
    }
    if (thicknessData_) 
    {
    thicknessData_->clearResult();
    }
    // Restore original model rendering (without vertex colors)
    auto& thicknessRender = publicDataRender_->m_PublicThicknessRenderData;
    thicknessRender.isVisible = false;
    thicknessRender.needsUpdate = true;
    PublicData::PublicDataRender::clearMeshData(thicknessRender.heatmapMeshData);
    thicknessRender.vertexThickness.clear();
    thicknessRender.legendValues.clear();
    thicknessRender.legendColors.clear();
    if (thicknessOverlay_) {
        thicknessOverlay_->hide();
    }

    publicDataRender_->m_PublicSTLRenderData.isVisible = true;
    publicDataRender_->m_PublicSTLRenderData.needsUpdate = true;

    PublicData::PublicDataRender::drawPublicModel(
        publicDataRender_->m_PublicSTLRenderData,
        *publicModelData_);
    refreshDataDisplay();
    QMessageBox::information(this, "Success", "Thickness heatmap cleared");

    // Emit signal for any listeners
    emit clearThicknessResultClicked();
}

// ================== Import Model Implementation ==================

void RibbonWidget::onImportModelClicked()
{
    if (!m_hasActiveWorkspace) {
        QMessageBox::warning(this,
            "No Active Project",
            "Please create or load a project first.");
        return;
    }

    // Get default path from DATA_PATH
    std::string data_path = DATA_PATH;
    QString defaultPath = QString::fromStdString(data_path + "/model");

    // Open file dialog to select STL file
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Select STL File",
        defaultPath,
        "STL Files (*.stl);;All Files (*.*)");

    if (fileName.isEmpty()) {
        return;
    }

    // Check required data pointers
    if (!publicModelData_) {
        QMessageBox::warning(this, "Error", "PublicModelData is not initialized");
        return;
    }

    // 1. Load STL file using PublicModelData only.
    bool loadSuccess = publicModelData_->loadModel(fileName.toStdString());
    if (!loadSuccess) {
        QMessageBox::warning(this, "Error", "Failed to load STL file");
        return;
    }

    const PublicData::PublicModel& publicModel = publicModelData_->getModel();

    // 2. Public STL overlay: import on the main view's GL context
    if (publicDataRender_ && publicModelData_) {
        publicDataRender_->m_PublicSTLRenderData.needsUpdate = true;
        publicDataRender_->m_PublicSTLRenderData.isVisible = true;
        if (openGLView_) {
            openGLView_->makeCurrent();
            PublicData::PublicDataRender::drawPublicModel(
                publicDataRender_->m_PublicSTLRenderData, *publicModelData_);
            openGLView_->doneCurrent();
        }
    }

    // 3. Refresh OpenGL display
    refreshDataDisplay();

    // 4. Show success message
    QMessageBox::information(this, "Success",
        QString("Public model loaded successfully\nFile: %1\nTriangles: %2")
            .arg(QString::fromStdString(publicModel.m_file_name))
            .arg(publicModel.m_indices.size() / 3));
}

// ================== Import/Export Operations Implementation ==================

void RibbonWidget::onImportTrajectoryClicked()
{
    if (!publicTrajData_) {
        QMessageBox::warning(this, "Warning", "Public trajectory data is not initialized.");
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Trajectory"),
        "",
        tr("Trajectory Files (*.traj *.txt *.paq);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    if (!publicTrajData_->loadFromFile(fileName.toStdString(), publicTrajData_->getTrajectoryPoints())) {
        QMessageBox::warning(this, "Error",
            QString("Failed to import trajectory file:\n%1").arg(fileName));
        return;
    }

    publicTrajData_->getTrajectoryFilePath() = fileName.toStdString();

    if (generalData_) {
        generalData_->trajData_.trajectoryPoints_.clear();
        generalData_->trajData_.connectionData_.connectedTrajectory_.clear();
        generalData_->trajData_.connectionData_.connectionSegments_.clear();
        generalData_->trajData_.connectionData_.isConnected_ = false;

        for (const auto& kv : publicTrajData_->getTrajectoryPoints()) {
            ModelTrajPlan::TrajectoryPoint point;
            point.pose = kv.second.pose;
            point.spray_distance = kv.second.spray_distance;
            point.timestamp = kv.first;
            point.velocity = 50.0;
            point.acceleration = 0.0;
            point.point_type = 0;
            generalData_->trajData_.trajectoryPoints_.push_back(point);
        }

        generalData_->processingState_.isTrajectoryGenerated =
            !generalData_->trajData_.trajectoryPoints_.empty();
        generalData_->processingState_.isTrajectoryConnected = false;
        generalData_->currentState_ = ModelTrajPlan::DataState::TrajectoryGenerated;
    }

    if (publicDataRender_) {
        publicDataRender_->m_PublicTrajectoryRenderData.isVisible = true;
        publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;
        if (openGLView_) {
            openGLView_->makeCurrent();
            PublicData::PublicDataRender::drawPublicTrajectory(
                publicDataRender_->m_PublicTrajectoryRenderData, *publicTrajData_);
            openGLView_->doneCurrent();
        }
    }

    if (renderData_ && trajPlanModel_) {
        trajPlanModel_->initializeRenderer(*renderData_);
        trajPlanModel_->setTrajectoryVisibility(*renderData_, true);
        trajPlanModel_->setConnectionVisibility(*renderData_, false);
    }

    refreshDataDisplay();
    QMessageBox::information(this, "Success",
        QString("Trajectory loaded successfully.\nFile: %1\nPoints: %2")
            .arg(fileName)
            .arg(publicTrajData_->getPointCount()));
}

void RibbonWidget::onImportRobotClicked()
{
    if (!trajOptimizationData_) {
        QMessageBox::warning(this, tr("Import Robot"), tr("Collision optimization data is not initialized."));
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Robot"),
        "",
        tr("Robot Models (*.xml *.rbt *.RBT);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    const std::string path = fileName.toStdString();
    if (std::find(collisionRobotPaths_.begin(), collisionRobotPaths_.end(), path) == collisionRobotPaths_.end()) {
        collisionRobotPaths_.push_back(path);
    }
    trajOptimizationData_->robotModelPaths_ = collisionRobotPaths_;

    if (collisionRobotPaths_.size() < 2) {
        QMessageBox::information(this, tr("Import Robot"),
            tr("Robot model added: %1\nImport one more robot model to initialize the collision robot system.").arg(fileName));
        return;
    }

    if (!ModelTrajPlan::TrajOptimizationAlgorithm::initializeRobotSystem(
            *trajOptimizationData_, collisionRobotPaths_)) {
        QMessageBox::warning(this, tr("Import Robot"),
            tr("The robot model was recorded, but the collision robot system could not be initialized."));
        return;
    }

    if (forceRefreshHook_) forceRefreshHook_();
    QMessageBox::information(this, tr("Import Robot"),
        tr("Collision robot system initialized with %1 robot models.").arg(collisionRobotPaths_.size()));
}

void RibbonWidget::onClearModelClicked()
{
    if (!publicModelData_ || !publicDataRender_) {
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "Clear Model",
        "Are you sure you want to clear the model?",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        // Clear model data
        publicModelData_->deleteModel();

        // Clear rendering
        publicDataRender_->m_PublicSTLRenderData.isVisible = false;
        publicDataRender_->m_PublicSTLRenderData.needsUpdate = true;
        PublicData::PublicDataRender::clearMeshData(
            publicDataRender_->m_PublicSTLRenderData.meshData);

        refreshDataDisplay();
        QMessageBox::information(this, "Success", "Model cleared");
    }
}

void RibbonWidget::onClearTrajectoryClicked()
{
    if (!publicTrajData_ || !publicDataRender_) {
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "Clear Trajectory",
        "Are you sure you want to clear the trajectory?",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        // Clear trajectory data
        publicTrajData_->clear();

        // Clear rendering
        publicDataRender_->m_PublicTrajectoryRenderData.isVisible = false;
        publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;
        PublicData::PublicDataRender::clearMeshData(
            publicDataRender_->m_PublicTrajectoryRenderData.trajectoryData);

        refreshDataDisplay();
        QMessageBox::information(this, "Success", "Trajectory cleared");
    }
}

void RibbonWidget::onClearConnectionClicked()
{
    if (!publicDataRender_) {
        return;
    }

    // Clear connection lines from rendering
    publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;

    refreshDataDisplay();
    QMessageBox::information(this, "Success", "Connection cleared");
}
// ================== Collision Optimization Implementation ==================

void RibbonWidget::onImportWorkpieceClicked()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Import Collision Workpiece"), QString(),
        tr("Workpiece Models (*.stl *.STL *.xml *.rbt *.RBT);;All Files (*)"));
    if (fileName.isEmpty()) return;

    const QString suffix = QFileInfo(fileName).suffix().toLower();
    bool loaded = false;
    if (suffix == "stl") {
        if (!publicModelData_) {
            QMessageBox::warning(this, tr("Import Collision Workpiece"), tr("Public model data is not initialized."));
            return;
        }
        loaded = publicModelData_->loadModel(fileName.toStdString());
        if (loaded && publicDataRender_ && openGLView_) {
            auto& render = publicDataRender_->m_PublicSTLRenderData;
            render.isVisible = true;
            render.needsUpdate = true;
            openGLView_->makeCurrent();
            PublicData::PublicDataRender::drawPublicModel(render, *publicModelData_);
            openGLView_->doneCurrent();
        }
    } else if (trajOptimizationData_) {
        loaded = trajOptimizationData_->loadWorkpieceModel(fileName.toStdString());
    }

    if (!loaded) {
        QMessageBox::warning(this, tr("Import Collision Workpiece"), tr("Failed to import workpiece: %1").arg(fileName));
        return;
    }
    m_collisionWorkpiecePath = fileName;
    if (trajOptimizationData_) trajOptimizationData_->workpieceModelPath_ = fileName.toStdString();
    if (forceRefreshHook_) forceRefreshHook_();
    refreshDataDisplay();
    QMessageBox::information(this, tr("Import Collision Workpiece"), tr("Workpiece imported: %1").arg(fileName));
}

void RibbonWidget::onImportCollisionModelClicked()
{
    if (!trajOptimizationData_) {
        QMessageBox::warning(this, tr("Import Tool"), tr("Collision optimization data is not initialized."));
        return;
    }
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Import Tool / Collision Model"), QString(),
        tr("Tool Models (*.xml *.rbt *.RBT);;All Files (*)"));
    if (fileName.isEmpty()) return;

    if (!trajOptimizationData_->loadToolModel(fileName.toStdString())) {
        QMessageBox::warning(this, tr("Import Tool"), tr("Failed to import tool model: %1").arg(fileName));
        return;
    }
    m_collisionModelPath = fileName;
    if (trajOptimizationData_->workpieceRobot_ && trajOptimizationData_->toolRobot_) {
        trajOptimizationData_->systemInitialized_ = true;
    }
    if (forceRefreshHook_) forceRefreshHook_();
    QMessageBox::information(this, tr("Import Tool"), tr("Tool / collision model imported: %1").arg(fileName));
}

void RibbonWidget::onRunCollisionOptimizationClicked()
{
    if (!trajOptimizationData_ || trajOptimizationData_->originalTrajectory_.empty()) {
        QMessageBox::warning(this, tr("Collision Optimization"),
            tr("Import a collision trajectory before running optimization."));
        return;
    }
    if (!trajOptimizationData_->workpieceRobot_ || !trajOptimizationData_->toolRobot_ ||
        trajOptimizationData_->m_robots.size() < 2) {
        QMessageBox::warning(this, tr("Collision Optimization"),
            tr("Import a collision workpiece and tool model before running optimization."));
        return;
    }
    if (!ModelTrajPlan::TrajOptimizationAlgorithm::transformTrajectory(*trajOptimizationData_) ||
        !ModelTrajPlan::TrajOptimizationAlgorithm::initCollision(*trajOptimizationData_) ||
        !ModelTrajPlan::TrajOptimizationAlgorithm::startCollisionDetection(*trajOptimizationData_)) {
        QMessageBox::warning(this, tr("Collision Optimization"),
            tr("Could not initialize the existing collision optimization pipeline."));
        return;
    }

    QProgressDialog progress(tr("Optimizing collision-free trajectory..."), tr("Cancel"),
        0, static_cast<int>(trajOptimizationData_->transformedTrajectory_.size()), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.show();

    const ModelTrajPlan::ToleranceLimitConfig limits;
    const bool success = ModelTrajPlan::TrajOptimizationAlgorithm::computeTrajectoryToleranceWithProgress(
        *trajOptimizationData_, limits,
        [this, &progress](int current, int total, const std::string&) {
            progress.setMaximum(total);
            progress.setValue(current);
            QApplication::processEvents();
            if (progress.wasCanceled()) trajOptimizationData_->collisionDetectionRunning_ = false;
        });
    ModelTrajPlan::TrajOptimizationAlgorithm::stopCollisionDetection(*trajOptimizationData_);
    progress.setValue(progress.maximum());

    if (!success) {
        QMessageBox::warning(this, tr("Collision Optimization"),
            tr("Collision optimization was cancelled or could not find a complete result."));
        return;
    }

    PublicData::TrajectoryPoints optimizedPoints;
    for (const auto& item : trajOptimizationData_->tolerancedTrajectory_) {
        PublicData::TrajectoryPoint point;
        point.pose = item.second.pose;
        point.spray_distance = item.second.spray_distance;
        point.velocity = 0.5 * (item.second.vel_range(0) + item.second.vel_range(1));
        optimizedPoints[item.first] = point;
    }
    if (publicTrajData_ && !optimizedPoints.empty()) {
        publicTrajData_->loadFromData(optimizedPoints);
        redrawPublicTransformData(false, true);
    }
    collisionPlaybackController_.loadOptimizedTrajectory(*trajOptimizationData_);
    if (forceRefreshHook_) forceRefreshHook_();
    refreshDataDisplay();
    QMessageBox::information(this, tr("Collision Optimization"),
        tr("Optimization completed. %1 poses are ready for export and playback.")
            .arg(trajOptimizationData_->tolerancedTrajectory_.size()));
}

void RibbonWidget::onClearCollisionModelClicked()
{
    if (trajOptimizationData_) {
        trajOptimizationData_->toolRobot_.reset();
        trajOptimizationData_->actualModelPath_.clear();
        trajOptimizationData_->hasActualModel_ = false;
        trajOptimizationData_->systemInitialized_ = false;
    }
    m_collisionModelPath.clear();
    if (forceRefreshHook_) forceRefreshHook_();
    QMessageBox::information(this, tr("Clear Collision Model"), tr("Tool / collision model cleared."));
}

// ================== Transform Operations Implementation ==================

void RibbonWidget::onImportTransformJsonClicked()
{
    auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_);
    if (!panel) return;
    const QString fileName = QFileDialog::getOpenFileName(this, tr("Import Transform Matrix"), QString(), tr("JSON Files (*.json);;All Files (*)"));
    if (fileName.isEmpty()) return;
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) { QMessageBox::critical(this, tr("Import failed"), file.errorString()); return; }
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &error);
    const QJsonArray rows = document.isObject() ? document.object().value("transform_matrix").toArray() : QJsonArray();
    QVector<double> matrix;
    if (rows.size() == 4) {
        for (const QJsonValue& rowValue : rows) {
            const QJsonArray row = rowValue.toArray();
            if (row.size() != 4) { matrix.clear(); break; }
            for (const QJsonValue& value : row) { if (!value.isDouble()) { matrix.clear(); break; } matrix.append(value.toDouble()); }
            if (matrix.isEmpty()) break;
        }
    }
    if (error.error != QJsonParseError::NoError || matrix.size() != 16 || std::abs(matrix[12]) > 1e-9 || std::abs(matrix[13]) > 1e-9 || std::abs(matrix[14]) > 1e-9 || std::abs(matrix[15] - 1.0) > 1e-9) {
        QMessageBox::warning(this, tr("Import failed"), tr("Expected transform_matrix as a valid 4x4 homogeneous matrix."));
        return;
    }
    panel->setTransformMatrixFlattened(matrix);
    QMessageBox::information(this, tr("Transform Matrix Imported"), QDir::toNativeSeparators(fileName));
}

void RibbonWidget::onExportTransformJsonClicked()
{
    auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_);
    if (!panel) return;
    QString fileName = QFileDialog::getSaveFileName(this, tr("Export Transform Matrix"), tr("transform_export.json"), tr("JSON Files (*.json);;All Files (*)"));
    if (fileName.isEmpty()) return;
    if (QFileInfo(fileName).suffix().isEmpty()) fileName += ".json";
    const QVector<double> matrix = panel->getTransformMatrixFlattened();
    QJsonArray rows;
    for (int row = 0; row < 4; ++row) { QJsonArray values; for (int col = 0; col < 4; ++col) values.append(matrix[row * 4 + col]); rows.append(values); }
    QJsonObject root; root.insert("version", "1.0"); root.insert("transform_matrix", rows);
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) { QMessageBox::critical(this, tr("Export failed"), file.errorString()); return; }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void RibbonWidget::onApplyTransformClicked()
{
    auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_);
    if (!panel || !publicDataRender_ || !openGLView_) return;
    if (!panel->transformImportedModelChecked() && !panel->transformImportedTrajectoryChecked()) {
        QMessageBox::warning(this, tr("Confirm Transform"), tr("Select at least one transformation target."));
        return;
    }
    if (QMessageBox::question(this, tr("Confirm Transform"),
            tr("Replace the selected public model and trajectory with their transformed results?"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    const QVector<double> values = panel->getTransformMatrixFlattened();
    if (values.size() != 16) return;
    Eigen::Matrix4d transform;
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) transform(row, col) = values[row * 4 + col];

    bool transformedModel = false;
    bool transformedTrajectory = false;
    if (panel->transformImportedModelChecked() && publicModelData_ && publicModelData_->hasValidData()) {
        if (!hasOriginalTransformModel_) {
            originalTransformModel_ = publicModelData_->getModel();
            hasOriginalTransformModel_ = true;
        }
        auto& model = publicModelData_->getModel();
        model = originalTransformModel_;
        const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        for (size_t index = 0; index + 2 < model.m_vertices.size(); index += 3) {
            const Eigen::Vector4d point(model.m_vertices[index], model.m_vertices[index + 1], model.m_vertices[index + 2], 1.0);
            const Eigen::Vector3d transformed = (transform * point).head<3>();
            model.m_vertices[index] = static_cast<float>(transformed.x());
            model.m_vertices[index + 1] = static_cast<float>(transformed.y());
            model.m_vertices[index + 2] = static_cast<float>(transformed.z());
        }
        for (size_t index = 0; index + 2 < model.m_normals.size(); index += 3) {
            Eigen::Vector3d normal(model.m_normals[index], model.m_normals[index + 1], model.m_normals[index + 2]);
            if (normal.squaredNorm() > 1e-12) normal = (rotation * normal).normalized();
            model.m_normals[index] = static_cast<float>(normal.x());
            model.m_normals[index + 1] = static_cast<float>(normal.y());
            model.m_normals[index + 2] = static_cast<float>(normal.z());
        }
        transformedModel = true;
    }
    if (panel->transformImportedTrajectoryChecked()) {
        if (!publicTrajData_ || !publicTrajData_->hasValidData()) {
            QMessageBox::warning(this, tr("Confirm Transform"), tr("No imported trajectory is available."));
            return;
        }
        if (!hasOriginalTransformTrajectory_) {
            originalTransformTrajectory_ = publicTrajData_->getTrajectoryPoints();
            hasOriginalTransformTrajectory_ = true;
        }
        PublicData::TrajectoryPoints output;
        for (const auto& item : originalTransformTrajectory_) {
            PublicData::TrajectoryPoint point = item.second;
            point.pose = transform * point.pose;
            output[item.first] = point;
        }
        transformedTrajectoryData_.getTrajectoryPoints() = output;
        publicTrajData_->loadFromData(output);
        transformedTrajectory = true;
    }
    redrawPublicTransformData(transformedModel, transformedTrajectory);
    // A later confirmation may target only one item.  Keep every previously
    // transformed source hidden until the user explicitly resets or clears it.
    setPrivateTransformSourcesVisible(!hasOriginalTransformModel_, !hasOriginalTransformTrajectory_);
    refreshDataDisplay();
}

void RibbonWidget::onResetTransformClicked()
{
    if (auto* panel = qobject_cast<PropertyPanel*>(propertyPanel_)) panel->resetTransformInputs();
    onClearTransformResultClicked();
}

void RibbonWidget::redrawPublicTransformData(bool redrawModel, bool redrawTrajectory)
{
    if (!publicDataRender_ || !openGLView_) return;
    openGLView_->makeCurrent();
    if (redrawModel && publicModelData_ && publicModelData_->hasValidData()) {
        auto& render = publicDataRender_->m_PublicSTLRenderData;
        render.isVisible = true;
        render.needsUpdate = true;
        PublicData::PublicDataRender::drawPublicModel(render, *publicModelData_);
    }
    if (redrawTrajectory && publicTrajData_ && publicTrajData_->hasValidData()) {
        auto& render = publicDataRender_->m_PublicTrajectoryRenderData;
        render.isVisible = true;
        render.needsUpdate = true;
        PublicData::PublicDataRender::drawPublicTrajectory(render, *publicTrajData_);
    }
    openGLView_->doneCurrent();
}

void RibbonWidget::setPrivateTransformSourcesVisible(bool modelVisible, bool trajectoryVisible)
{
    if (!renderData_) return;
    if (trajPlanModel_) {
        trajPlanModel_->setOriginalDataVisibility(*renderData_, modelVisible);
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, modelVisible);
        trajPlanModel_->setBoundaryVisibility(*renderData_, modelVisible);
        trajPlanModel_->setTrajectoryVisibility(*renderData_, trajectoryVisible);
        trajPlanModel_->setConnectionVisibility(*renderData_, trajectoryVisible);
        return;
    }
    renderData_->stl().isVisible = modelVisible;
    renderData_->slicing().isVisible = modelVisible;
    renderData_->boundary().isVisible = modelVisible;
    renderData_->trajectory().isVisible = trajectoryVisible;
    renderData_->connection().isVisible = trajectoryVisible;
}

void RibbonWidget::restoreTransformSources()
{
    const bool restoreModel = hasOriginalTransformModel_ && publicModelData_;
    const bool restoreTrajectory = hasOriginalTransformTrajectory_ && publicTrajData_;
    if (restoreModel) publicModelData_->getModel() = originalTransformModel_;
    if (restoreTrajectory) publicTrajData_->loadFromData(originalTransformTrajectory_);
    redrawPublicTransformData(restoreModel, restoreTrajectory);
    setPrivateTransformSourcesVisible(true, true);
    originalTransformModel_.clear();
    originalTransformTrajectory_.clear();
    hasOriginalTransformModel_ = false;
    hasOriginalTransformTrajectory_ = false;
    transformedTrajectoryData_.clear();
}
// ================== New Method Implementations for RibbonWidget ==================
// Append these to the end of RibbonWidget.cpp

// ================== Trajectory Generation Export Operations ==================

void RibbonWidget::exportTrajectory()
{
    onExportTrajectoryClicked();
}

void RibbonWidget::exportABB()
{
    if (!trajPlanModel_ || !generalData_) {
        QMessageBox::warning(this, "Warning", "Trajectory planning data is unavailable.");
        return;
    }

    std::vector<TrajectoryPoint> trajectoryPoints;
    trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
    if (trajectoryPoints.empty()) {
        QMessageBox::warning(this, "Warning", "No generated trajectory data to export.");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export ABB RAPID Module"),
        tr("MainModule.mod"),
        tr("ABB RAPID Module (*.mod);;All Files (*)"));
    if (fileName.isEmpty()) {
        return;
    }
    if (QFileInfo(fileName).suffix().isEmpty()) {
        fileName += ".mod";
    }

    if (!trajPlanModel_->exportABBProgram(
            *generalData_, fileName.toStdString(), fileName.toStdString())) {
        QMessageBox::critical(this, "Export failed",
            QString("Could not write ABB RAPID module:\n%1").arg(fileName));
        return;
    }

    QMessageBox::information(this, "Export complete",
        QString("Exported %1 ABB trajectory points to:\n%2")
            .arg(trajectoryPoints.size())
            .arg(QDir::toNativeSeparators(fileName)));
    emit exportABBClicked();
}

void RibbonWidget::exportFANUC()
{
    QMessageBox::information(this, "Info", "Export FANUC format - To be implemented");
    emit exportFANUCClicked();
}

void RibbonWidget::exportExcel()
{
    QMessageBox::information(this, "Info", "Export to Excel - To be implemented");
    emit exportExcelClicked();
}

void RibbonWidget::onExportTrajectoryClicked()
{
    if (!trajPlanModel_ || !generalData_) {
        QMessageBox::warning(this, "Warning", "Trajectory planning data is unavailable.");
        return;
    }

    std::vector<TrajectoryPoint> trajectoryPoints;
    trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
    if (trajectoryPoints.empty()) {
        QMessageBox::warning(this, "Warning", "No generated trajectory data to export.");
        return;
    }

    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Trajectory"),
        tr("trajectory.traj"),
        tr("Trajectory Files (*.traj *.txt);;ABB RAPID Module (*.mod);;FANUC LS Program (*.LS *.ls);;All Files (*)"),
        &selectedFilter);

    if (fileName.isEmpty()) {
        return;
    }

    const QString suffix = QFileInfo(fileName).suffix().toLower();
    if (suffix.isEmpty()) {
        fileName += selectedFilter.contains("FANUC", Qt::CaseInsensitive) ? ".LS"
            : (selectedFilter.contains("ABB", Qt::CaseInsensitive) ? ".mod" : ".traj");
    }

    const bool isAbbModule = QFileInfo(fileName).suffix().compare("mod", Qt::CaseInsensitive) == 0;
    const bool isFanucProgram = QFileInfo(fileName).suffix().compare("ls", Qt::CaseInsensitive) == 0;
    bool exported = false;
    if (isAbbModule) {
        exported = trajPlanModel_->exportABBProgram(*generalData_, fileName.toStdString(), fileName.toStdString());
    } else if (isFanucProgram) {
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream stream(&file);
            stream.setLocale(QLocale::c());
            stream.setRealNumberNotation(QTextStream::FixedNotation);
            stream.setRealNumberPrecision(6);
            QString programName = QFileInfo(fileName).completeBaseName().toUpper();
            programName.replace(QRegularExpression("[^A-Z0-9_]"), "_");
            if (programName.isEmpty()) programName = "TRAJECT";
            stream << "/PROG  " << programName.left(8) << "\n/ATTR\n"
                   << "OWNER\t\t= MNEDITOR;\nCOMMENT\t\t= \"Exported trajectory\";\n"
                   << "PROG_SIZE\t= 0;\nVERSION\t\t= 0;\nLINE_COUNT\t= " << trajectoryPoints.size() << ";\n"
                   << "MEMORY_SIZE\t= 0;\nPROTECT\t\t= READ_WRITE;\nDEFAULT_GROUP\t= 1,*,*,*,*;\n/MN\n";
            stream << "   1:  UFRAME_NUM=2 ;\n"
                   << "   2:  UTOOL_NUM=1 ;\n";
            int index = 3;
            for (const auto& point : trajectoryPoints) {
                const int speed = std::max(1, static_cast<int>(std::lround(point.velocity > 0.0 ? point.velocity : 50.0)));
                stream << QString("%1").arg(index, 4) << ":  L P[" << index << "] " << speed << "mm/sec CNT100    ;\n";
                ++index;
            }
            stream << "/POS\n";
            index = 1;
            constexpr double radiansToDegrees = 57.2957795130823208768;
            for (const auto& point : trajectoryPoints) {
                const auto xyz = point.pose.block<3, 1>(0, 3);
                const Eigen::Vector3d wpr = point.pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0) * radiansToDegrees;
                stream << "P[" << index++ << "]{\n   GP1:\n"
                       << "\tUF : 2, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\n"
                       << "\tX = " << xyz.x() << "  mm,\tY = " << xyz.y() << "  mm,\tZ = " << xyz.z() << "  mm,\n"
                       << "\tW = " << wpr.x() << " deg,\tP = " << wpr.y() << " deg,\tR = " << wpr.z() << " deg\n};\n";
            }
            stream << "/END\n";
            exported = stream.status() == QTextStream::Ok;
        }
    } else {
        exported = trajPlanModel_->exportTrajectoryToFile(*generalData_, fileName.toStdString());
    }
    if (!exported) {
        QMessageBox::critical(this, "Export failed",
            QString("Could not write trajectory file:\n%1").arg(fileName));
        return;
    }

    QMessageBox::information(this, "Export complete",
        QString("Exported %1 trajectory points%2 to:\n%3")
            .arg(trajectoryPoints.size())
            .arg(isAbbModule ? tr(" as ABB RAPID module") : (isFanucProgram ? tr(" as FANUC LS program") : QString()))
            .arg(QDir::toNativeSeparators(fileName)));

    emit exportTrajectoryClicked();
}

// ================== Collision Optimization Operations ==================

void RibbonWidget::importCollisionTrajectory()
{
    onImportCollisionTrajectoryClicked();
}

void RibbonWidget::exportOptimizedTrajectory()
{
    onExportOptimizedTrajectoryClicked();
}

void RibbonWidget::exportCollisionReport()
{
    onExportCollisionReportClicked();
}

void RibbonWidget::onImportCollisionTrajectoryClicked()
{
    if (!trajOptimizationData_) {
        QMessageBox::warning(this, tr("Import Trajectory for Collision Optimization"),
            tr("Collision optimization data is not initialized."));
        return;
    }
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Trajectory for Collision Optimization"),
        "",
        tr("Trajectory Files (*.traj *.txt *.paq);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    if (!ModelTrajPlan::TrajOptimizationAlgorithm::loadTrajectoryFromFile(
            *trajOptimizationData_, fileName.toStdString())) {
        QMessageBox::warning(this, tr("Import Trajectory for Collision Optimization"),
            tr("Failed to parse trajectory file: %1").arg(fileName));
        return;
    }

    if (publicTrajData_) {
        PublicData::TrajectoryPoints points;
        if (publicTrajData_->loadFromFile(fileName.toStdString(), points)) {
            publicTrajData_->getTrajectoryFilePath() = fileName.toStdString();
            redrawPublicTransformData(false, true);
        }
    }
    if (forceRefreshHook_) forceRefreshHook_();
    QMessageBox::information(this, tr("Import Trajectory for Collision Optimization"),
        tr("Collision trajectory imported: %1\nPoints: %2")
            .arg(fileName)
            .arg(trajOptimizationData_->originalTrajectory_.size()));

    emit importCollisionTrajectoryClicked();
}

void RibbonWidget::onExportOptimizedTrajectoryClicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Optimized Trajectory"),
        "",
        tr("Trajectory Files (*.traj *.txt);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Export optimized trajectory to: %1\nImplementation pending").arg(fileName));

    emit exportOptimizedTrajectoryClicked();
}

void RibbonWidget::onExportCollisionReportClicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Collision Report"),
        "",
        tr("Report Files (*.txt *.pdf *.html);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Export collision report to: %1\nImplementation pending").arg(fileName));

    emit exportCollisionReportClicked();
}

// ================== Pose Transformation Operations ==================

void RibbonWidget::importTransformTrajectory()
{
    onImportTransformTrajectoryClicked();
}

void RibbonWidget::exportTransformedModel()
{
    onExportTransformedModelClicked();
}

void RibbonWidget::exportTransformedTrajectory()
{
    onExportTransformedTrajectoryClicked();
}

void RibbonWidget::clearExtraModel()
{
    onClearExtraModelClicked();
}

void RibbonWidget::clearTransformResult()
{
    onClearTransformResultClicked();
}

void RibbonWidget::clearMatrix()
{
    onClearMatrixClicked();
}

void RibbonWidget::onImportTransformTrajectoryClicked()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Trajectory for Transformation"),
        "",
        tr("Trajectory Files (*.traj *.txt);;ABB RAPID Module (*.mod);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    if (!publicTrajData_) return;
    PublicData::TrajectoryPoints points;
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { QMessageBox::critical(this, tr("Import failed"), file.errorString()); return; }
    QTextStream stream(&file);
    const bool abbModule = QFileInfo(fileName).suffix().compare("mod", Qt::CaseInsensitive) == 0;
    const QRegularExpression numbers("[-+]?\\d*\\.?\\d+(?:[Ee][-+]?\\d+)?");
    const QRegularExpression abbTarget(
        "\\brobtarget\\s+[^:=\\s]+\\s*:=\\s*\\[\\[([^\\]]+)\\],\\[([^\\]]+)\\]",
        QRegularExpression::CaseInsensitiveOption);
    double timestamp = 0.0;
    while (!stream.atEnd()) {
        const QString line = stream.readLine();
        QVector<double> values;
        int offset = 1;
        if (abbModule) {
            const QRegularExpressionMatch targetMatch = abbTarget.match(line);
            if (!targetMatch.hasMatch()) continue;
            for (const QString& group : {targetMatch.captured(1), targetMatch.captured(2)}) {
                auto matches = numbers.globalMatch(group);
                while (matches.hasNext()) values << matches.next().captured().toDouble();
            }
            offset = 0;
        } else {
            auto matches = numbers.globalMatch(line);
            while (matches.hasNext()) values << matches.next().captured().toDouble();
        }
        if (values.size() < offset + 7) continue;
        PublicData::TrajectoryPoint point; point.pose.setIdentity();
        point.pose(0,3)=values[offset]; point.pose(1,3)=values[offset+1]; point.pose(2,3)=values[offset+2];
        Eigen::Quaterniond q(values[offset+3], values[offset+4], values[offset+5], values[offset+6]); if (q.norm()>1e-12) point.pose.block<3,3>(0,0)=q.normalized().toRotationMatrix();
        points[abbModule ? timestamp++ : values[0]] = point;
    }
    if (points.empty()) { QMessageBox::warning(this, tr("Import failed"), tr("No trajectory points were found.")); return; }
    // A newly imported file becomes the next transform's source, not a
    // continuation of a previous transformed result.
    originalTransformTrajectory_.clear();
    transformedTrajectoryData_.clear();
    hasOriginalTransformTrajectory_ = false;
    publicTrajData_->loadFromData(points);
    redrawPublicTransformData(false, true);
    refreshDataDisplay();

    emit importTransformTrajectoryClicked();
}

void RibbonWidget::onExportTransformedModelClicked()
{
    if (!publicModelData_ || !publicModelData_->hasValidData()) {
        QMessageBox::warning(this, tr("Export Transformed Model"),
            tr("Import and apply a model transform before exporting."));
        return;
    }

    const PublicData::PublicModel& model = publicModelData_->getModel();
    const QString suggestedName = model.m_file_name.empty()
        ? tr("transformed_model.stl")
        : QString::fromStdString(model.m_file_name).section('.', 0, -2) + tr("_transformed.stl");
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Transformed Model"),
        suggestedName,
        tr("STL Files (*.stl);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    if (QFileInfo(fileName).suffix().isEmpty()) fileName += ".stl";

    const bool hasIndices = model.m_indices.size() >= 3;
    const size_t triangleCount = hasIndices
        ? model.m_indices.size() / 3
        : model.m_vertices.size() / 9;
    if (triangleCount == 0) {
        QMessageBox::warning(this, tr("Export Transformed Model"), tr("The model contains no triangles."));
        return;
    }

    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, tr("Export failed"), file.errorString());
        return;
    }

    QByteArray header(80, '\0');
    const QByteArray label("Industrial Robot Spray System - transformed model");
    std::copy(label.cbegin(), label.cend(), header.begin());
    file.write(header);
    const quint32 count = static_cast<quint32>(triangleCount);
    file.write(reinterpret_cast<const char*>(&count), sizeof(count));

    const auto vertexAt = [&model](size_t index, float output[3]) -> bool {
        const size_t offset = index * 3;
        if (offset + 2 >= model.m_vertices.size()) return false;
        output[0] = model.m_vertices[offset];
        output[1] = model.m_vertices[offset + 1];
        output[2] = model.m_vertices[offset + 2];
        return true;
    };

    for (size_t triangle = 0; triangle < triangleCount; ++triangle) {
        const size_t i0 = hasIndices ? model.m_indices[triangle * 3] : triangle * 3;
        const size_t i1 = hasIndices ? model.m_indices[triangle * 3 + 1] : triangle * 3 + 1;
        const size_t i2 = hasIndices ? model.m_indices[triangle * 3 + 2] : triangle * 3 + 2;
        float vertices[9];
        if (!vertexAt(i0, vertices) || !vertexAt(i1, vertices + 3) || !vertexAt(i2, vertices + 6)) {
            file.cancelWriting();
            QMessageBox::critical(this, tr("Export failed"), tr("The model contains an invalid triangle index."));
            return;
        }
        const Eigen::Vector3f a(vertices[0], vertices[1], vertices[2]);
        const Eigen::Vector3f b(vertices[3], vertices[4], vertices[5]);
        const Eigen::Vector3f c(vertices[6], vertices[7], vertices[8]);
        Eigen::Vector3f normal = (b - a).cross(c - a);
        if (normal.squaredNorm() > 1e-12f) normal.normalize();
        else normal = Eigen::Vector3f::UnitZ();
        const quint16 attributeByteCount = 0;
        file.write(reinterpret_cast<const char*>(normal.data()), sizeof(float) * 3);
        file.write(reinterpret_cast<const char*>(vertices), sizeof(vertices));
        file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));
    }

    if (!file.commit()) {
        QMessageBox::critical(this, tr("Export failed"), file.errorString());
        return;
    }
    QMessageBox::information(this, tr("Export complete"),
        tr("Exported %1 triangles to:\n%2").arg(triangleCount).arg(QDir::toNativeSeparators(fileName)));

    emit exportTransformedModelClicked();
}

void RibbonWidget::onExportTransformedTrajectoryClicked()
{
    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Transformed Trajectory"),
        "",
        tr("Trajectory Files (*.traj *.txt);;ABB RAPID Module (*.mod);;FANUC LS Program (*.LS *.ls);;All Files (*)"),
        &selectedFilter);

    if (fileName.isEmpty()) {
        return;
    }

    const PublicData::TrajectoryPoints* trajectoryToExport = nullptr;
    if (hasOriginalTransformTrajectory_ && publicTrajData_ && publicTrajData_->hasValidData()) {
        trajectoryToExport = &publicTrajData_->getTrajectoryPoints();
    } else if (transformedTrajectoryData_.hasValidData()) {
        trajectoryToExport = &transformedTrajectoryData_.getTrajectoryPoints();
    }
    if (!trajectoryToExport) { QMessageBox::warning(this, tr("Export Transformed Trajectory"), tr("Apply a transform before exporting.")); return; }
    if (QFileInfo(fileName).suffix().isEmpty()) {
        fileName += selectedFilter.contains("FANUC", Qt::CaseInsensitive) ? ".LS"
            : (selectedFilter.contains("ABB", Qt::CaseInsensitive) ? ".mod" : ".traj");
    }
    const bool abbModule = QFileInfo(fileName).suffix().compare("mod", Qt::CaseInsensitive) == 0;
    const bool fanucProgram = QFileInfo(fileName).suffix().compare("ls", Qt::CaseInsensitive) == 0;
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) { QMessageBox::critical(this, tr("Export failed"), file.errorString()); return; }
    QTextStream stream(&file);
    if (abbModule) {
        stream << "MODULE MainModule\n";
    } else if (fanucProgram) {
        QString programName = QFileInfo(fileName).completeBaseName().toUpper();
        programName.replace(QRegularExpression("[^A-Z0-9_]"), "_");
        if (programName.isEmpty()) programName = "TRAJECT";
        stream << "/PROG  " << programName.left(8) << "\n"
               << "/ATTR\n"
               << "OWNER\t\t= MNEDITOR;\n"
               << "COMMENT\t\t= \"Exported trajectory\";\n"
               << "PROG_SIZE\t= 0;\n"
               << "VERSION\t\t= 0;\n"
               << "LINE_COUNT\t= " << trajectoryToExport->size() << ";\n"
               << "MEMORY_SIZE\t= 0;\n"
               << "PROTECT\t\t= READ_WRITE;\n"
               << "DEFAULT_GROUP\t= 1,*,*,*,*;\n"
               << "/MN\n";
    }
    int index = 1;
    stream.setLocale(QLocale::c());
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(6);
    for (const auto& item : *trajectoryToExport) {
        const auto& p = item.second;
        const Eigen::Quaterniond q(p.pose.block<3, 3>(0, 0));
        const auto xyz = p.pose.block<3, 1>(0, 3);
        if (abbModule) {
            stream << "  CONST robtarget p" << index << " := [[" << xyz.x() << "," << xyz.y() << "," << xyz.z() << "],["
                   << q.w() << "," << q.x() << "," << q.y() << "," << q.z()
                   << "],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
        } else if (!fanucProgram) {
            stream << item.first << " " << xyz.x() << " " << xyz.y() << " " << xyz.z() << " "
                   << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
        }
        ++index;
    }
    if (abbModule) {
        stream << "  PROC main()\n";
        for (int i = 1; i < index; ++i) {
            const char* zone = i + 1 == index ? "fine" : "z10";
            stream << "    MoveL p" << i << ", v50, " << zone << ", tool0;\n";
        }
        stream << "  ENDPROC\nENDMODULE\n";
    } else if (fanucProgram) {
        stream << "   1:  UFRAME_NUM=2 ;\n"
               << "   2:  UTOOL_NUM=1 ;\n";
        index = 3;
        for (const auto& item : *trajectoryToExport) {
            const int speed = std::max(1, static_cast<int>(std::lround(item.second.velocity > 0.0 ? item.second.velocity : 50.0)));
            stream << QString("%1").arg(index, 4) << ":  L P["
                   << index << "] " << speed << "mm/sec CNT100    ;\n";
            ++index;
        }
        stream << "/POS\n";
        index = 1;
        constexpr double radiansToDegrees = 57.2957795130823208768;
        for (const auto& item : *trajectoryToExport) {
            const auto& p = item.second;
            const auto xyz = p.pose.block<3, 1>(0, 3);
            const Eigen::Vector3d wpr = p.pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0) * radiansToDegrees;
            stream << "P[" << index << "]{\n"
                   << "   GP1:\n"
                   << "\tUF : 2, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\n"
                   << "\tX = " << xyz.x() << "  mm,\tY = " << xyz.y() << "  mm,\tZ = " << xyz.z() << "  mm,\n"
                   << "\tW = " << wpr.x() << " deg,\tP = " << wpr.y() << " deg,\tR = " << wpr.z() << " deg\n"
                   << "};\n";
            ++index;
        }
        stream << "/END\n";
    }
    if (stream.status() != QTextStream::Ok) { QMessageBox::critical(this, tr("Export failed"), file.errorString()); return; }

    emit exportTransformedTrajectoryClicked();
}

void RibbonWidget::onClearExtraModelClicked()
{
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "Clear Extra Model",
        "Are you sure you want to clear the extra model?",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        // TODO: Clear extra model from rendering
        refreshDataDisplay();
        QMessageBox::information(this, "Success", "Extra model cleared");
        emit clearExtraModelClicked();
    }
}

void RibbonWidget::onClearTransformResultClicked()
{
    restoreTransformSources();
    refreshDataDisplay();
    emit clearTransformResultClicked();
}

void RibbonWidget::onClearMatrixClicked()
{
    QMessageBox::information(this, "Info", "Clear matrix - To be implemented");
    emit clearMatrixClicked();
}

// ================== Thickness Prediction Operations ==================

void RibbonWidget::importThicknessWorkpiece()
{
    onImportThicknessWorkpieceClicked();
}

void RibbonWidget::importPointCloud()
{
    onImportPointCloudClicked();
}

void RibbonWidget::importThicknessTrajectory()
{
    onImportThicknessTrajectoryClicked();
}

void RibbonWidget::exportThicknessResult()
{
    onExportThicknessResultClicked();
}

void RibbonWidget::exportHeatmap()
{
    onExportHeatmapClicked();
}

void RibbonWidget::exportThicknessReport()
{
    onExportThicknessReportClicked();
}

void RibbonWidget::onImportThicknessWorkpieceClicked()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Workpiece for Thickness Prediction"),
        "",
        tr("STL Files (*.stl);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Import thickness workpiece: %1\nImplementation pending").arg(fileName));

    emit importThicknessWorkpieceClicked();
}

void RibbonWidget::onImportPointCloudClicked()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Import Point Cloud"),
        "",
        tr("Point Cloud Files (*.pcd *.ply *.xyz);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Import point cloud: %1\nImplementation pending").arg(fileName));

    emit importPointCloudClicked();
}

void RibbonWidget::onImportThicknessTrajectoryClicked()
{
    onImportTrajectoryClicked();
    emit importThicknessTrajectoryClicked();
}

void RibbonWidget::onExportThicknessResultClicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Thickness Result"),
        "",
        tr("Result Files (*.txt *.csv);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Export thickness result to: %1\nImplementation pending").arg(fileName));

    emit exportThicknessResultClicked();
}

void RibbonWidget::onExportHeatmapClicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Heatmap"),
        "",
        tr("Image Files (*.png *.jpg *.bmp);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    QMessageBox::information(this, "Info",
        QString("Export heatmap to: %1\nImplementation pending").arg(fileName));

    emit exportHeatmapClicked();
}

void RibbonWidget::onExportThicknessReportClicked()
{
    if (!thicknessData_ || !thicknessData_->getResultData().hasValidResult) {
        QMessageBox::warning(this, tr("Export Thickness Report"),
            tr("Run a thickness simulation before exporting a report."));
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Export Thickness Report"),
        "",
        tr("HTML Report (*.html);;All Files (*)"));

    if (fileName.isEmpty()) {
        return;
    }

    const QString reportPath = writeThicknessReport(thicknessData_->getResultData(), openGLView_, fileName);
    if (reportPath.isEmpty()) {
        QMessageBox::critical(this, tr("Export Thickness Report"), tr("Failed to write the report."));
        return;
    }

    QMessageBox::information(this, tr("Export Thickness Report"),
        tr("Report generated:\n%1").arg(reportPath));

    emit exportThicknessReportClicked();
}


void RibbonWidget::setThicknessAnalyzer(ModelTrajPlan::ThicknessAnalyzer_spointer analyzer)
{
    thicknessAnalyzer_ = analyzer;
}

void RibbonWidget::setThicknessData(ModelTrajPlan::ThicknessData* thicknessData)
{
    thicknessData_ = thicknessData;
}
