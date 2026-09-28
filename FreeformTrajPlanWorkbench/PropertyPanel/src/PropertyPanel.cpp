#include "PropertyPanel/PropertyPanel.h"
#include "ui_PropertyPanel.h"
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QStackedWidget>
#include <QMessageBox>

#include <algorithm>
#include <cmath>
#include <exception>

PropertyPanel::PropertyPanel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PropertyPanel)
{
    ui->setupUi(this);

    // Connect stacked widget page change signal
    connect(ui->stackedWidget, &QStackedWidget::currentChanged,
            this, &PropertyPanel::pageChanged);

    // Setup all internal connections
    setupConnections();
}

PropertyPanel::~PropertyPanel()
{
    delete ui;
}

void PropertyPanel::switchToPage(int index)
{
    if (index >= 0 && index < ui->stackedWidget->count()) {
        ui->stackedWidget->setCurrentIndex(index);
    }
}

int PropertyPanel::getCurrentPage() const
{
    return ui->stackedWidget->currentIndex();
}

void PropertyPanel::setSlicingWidget(QWidget* widget)
{
    if (!widget) return;

    ensureGenerationMethodStack();
    m_slicingWidget = widget;

    // Find and hide the controlGroupBox (right side - slicing control)
    m_controlGroupBox = widget->findChild<QGroupBox*>("controlGroupBox");
    if (m_controlGroupBox) {
        m_controlGroupBox->setVisible(false);
    }

    // Find paramGroupBox and add "Open Slicing Control" button
    QGroupBox* paramBox = widget->findChild<QGroupBox*>("paramGroupBox");
    if (paramBox && paramBox->layout()) {
        // Add button at the end of paramGroupBox
        m_openSlicingDialogButton = new QPushButton(tr("Open Slicing Control Dialog"), paramBox);
        m_openSlicingDialogButton->setObjectName("openSlicingControlDialogButton");
        m_openSlicingDialogButton->setMinimumHeight(42);
        m_openSlicingDialogButton->setEnabled(m_hasActiveWorkspace);  // Disable if no workspace

        if (auto* boxLayout = qobject_cast<QBoxLayout*>(paramBox->layout())) {
            const int insertIndex = qMax(0, boxLayout->count() - 1);
            boxLayout->insertSpacing(insertIndex, 12);
            boxLayout->insertWidget(insertIndex + 1, m_openSlicingDialogButton);
        } else {
            paramBox->layout()->addWidget(m_openSlicingDialogButton);
        }

        connect(m_openSlicingDialogButton, &QPushButton::clicked, [this]() {
            if (!m_hasActiveWorkspace) {
                QMessageBox::information(this,
                    tr("No Active Workspace"),
                    tr("Please create or open a workspace first.\n\n"
                       "Go to File > New Project or File > Load Project to get started."),
                    QMessageBox::Ok);
                return;
            }
            emit openSlicingControlDialogRequested();
        });
    }

    // Add SlicingWidgets to the slicing generation page.
    QLayout* existingLayout = m_slicingGenerationPage->layout();
    if (existingLayout) {
        QLayoutItem* item;
        while ((item = existingLayout->takeAt(0)) != nullptr) {
            delete item->widget();
            delete item;
        }
        delete existingLayout;
    }

    QVBoxLayout* layout = new QVBoxLayout(m_slicingGenerationPage);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_slicingWidget);
    m_generationMethodStack->setCurrentWidget(m_slicingGenerationPage);
}

void PropertyPanel::showSlicingGenerationPanel()
{
    ensureGenerationMethodStack();
    m_generationMethodStack->setCurrentWidget(m_slicingGenerationPage);
}

void PropertyPanel::showUvMappingGenerationPanel()
{
    ensureGenerationMethodStack();
    createUvMappingParameterPanel();
    m_generationMethodStack->setCurrentWidget(m_uvMappingGenerationPage);
}

double PropertyPanel::uvTargetSpacing() const { return m_uvSpinBoxes.value(0)->value(); }
double PropertyPanel::uvTrajectorySpeed() const { return m_uvSpinBoxes.value(1)->value(); }
double PropertyPanel::uvSprayDistance() const { return m_uvSpinBoxes.value(2)->value(); }
double PropertyPanel::uvMaxSeamSprayDistance() const { return m_uvSpinBoxes.value(3)->value(); }
double PropertyPanel::uvMaximumAcceleration() const { return m_uvSpinBoxes.value(4)->value(); }
double PropertyPanel::uvMaximumJerk() const { return m_uvSpinBoxes.value(5)->value(); }
double PropertyPanel::uvArcLockGuardD() const { return m_uvSpinBoxes.value(6)->value(); }
double PropertyPanel::uvArcSpeedLossPct() const { return m_uvSpinBoxes.value(7)->value(); }

void PropertyPanel::setUvControlsEnabled(bool enabled)
{
    for (QDoubleSpinBox* spinBox : m_uvSpinBoxes) spinBox->setEnabled(enabled);
}

void PropertyPanel::setUvTrajectoryGenerationEnabled(bool enabled)
{
    if (m_uvGenerateTrajectoryButton) m_uvGenerateTrajectoryButton->setEnabled(enabled);
}

void PropertyPanel::showMeshDecompositionGenerationPanel()
{
    ensureGenerationMethodStack();
    m_generationMethodStack->setCurrentWidget(m_meshDecompositionGenerationPage);
}

void PropertyPanel::ensureGenerationMethodStack()
{
    if (m_generationMethodStack) {
        return;
    }

    QLayout* existingLayout = ui->trajectoryGenerationContainer->layout();
    if (existingLayout) {
        QLayoutItem* item;
        while ((item = existingLayout->takeAt(0)) != nullptr) {
            delete item->widget();
            delete item;
        }
        delete existingLayout;
    }

    m_generationMethodStack = new QStackedWidget(ui->trajectoryGenerationContainer);
    m_slicingGenerationPage = new QWidget(m_generationMethodStack);
    m_uvMappingGenerationPage = new QWidget(m_generationMethodStack);
    m_meshDecompositionGenerationPage = new QWidget(m_generationMethodStack);

    m_generationMethodStack->addWidget(m_slicingGenerationPage);
    m_generationMethodStack->addWidget(m_uvMappingGenerationPage);
    m_generationMethodStack->addWidget(m_meshDecompositionGenerationPage);

    QVBoxLayout* containerLayout = new QVBoxLayout(ui->trajectoryGenerationContainer);
    containerLayout->setContentsMargins(0, 0, 0, 0);
    containerLayout->setSpacing(0);
    containerLayout->addWidget(m_generationMethodStack);
}

void PropertyPanel::createUvMappingParameterPanel()
{
    if (m_uvParameterGroup) return;
    m_uvParameterGroup = new QGroupBox(tr("UV Mapping Parameters"), m_uvMappingGenerationPage);
    auto* form = new QFormLayout(m_uvParameterGroup);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    const auto addSpinBox = [this, form](const QString& label, const QString& suffix,
        double value, double minimum, double maximum, int decimals) {
        auto* spinBox = new QDoubleSpinBox(m_uvParameterGroup);
        spinBox->setRange(minimum, maximum);
        spinBox->setDecimals(decimals);
        spinBox->setValue(value);
        spinBox->setSuffix(suffix);
        spinBox->setSingleStep(value > 100.0 ? 10.0 : 1.0);
        form->addRow(label, spinBox);
        m_uvSpinBoxes.append(spinBox);
    };
    addSpinBox(tr("Target spacing d"), tr(" mm"), 30.0, 0.001, 1.0e6, 3);
    addSpinBox(tr("Trajectory speed"), tr(" mm/s"), 300.0, 0.001, 1.0e6, 3);
    addSpinBox(tr("Spray distance"), tr(" mm"), 0.0, -1.0e6, 1.0e6, 3);
    addSpinBox(tr("Maximum seam spray distance"), tr(" mm"), 78.0, 0.001, 1.0e6, 3);
    addSpinBox(tr("Maximum acceleration"), tr(" mm/s2"), 1000.0, 0.001, 1.0e8, 3);
    addSpinBox(tr("Maximum jerk"), tr(" mm/s3"), 5000.0, 0.001, 1.0e9, 3);
    addSpinBox(tr("OVERTRAVEL length (d)"), QString(), 4.0, 0.0, 1.0e6, 3);
    addSpinBox(tr("Arc speed loss"), tr(" %"), 0.0, 0.0, 99.999, 3);
    m_uvSplitViewCheckBox = new QCheckBox(tr("Show UV split view"), m_uvParameterGroup);
    m_uvSplitViewCheckBox->setToolTip(tr("Show the UV map and UV trajectory beside the 3D view."));
    form->addRow(m_uvSplitViewCheckBox);
    connect(m_uvSplitViewCheckBox, &QCheckBox::toggled, this, &PropertyPanel::uvSplitViewChanged);
    m_uvGenerateTrajectoryButton = new QPushButton(tr("Generate Trajectory"), m_uvParameterGroup);
    m_uvGenerateTrajectoryButton->setToolTip(tr("Generate and render the trajectory using the current UV parameters."));
    m_uvGenerateTrajectoryButton->setEnabled(false);
    form->addRow(m_uvGenerateTrajectoryButton);
    connect(m_uvGenerateTrajectoryButton, &QPushButton::clicked,
        this, &PropertyPanel::uvTrajectoryGenerationRequested);

    auto* layout = new QVBoxLayout(m_uvMappingGenerationPage);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->addWidget(m_uvParameterGroup);
    layout->addStretch();
}

bool PropertyPanel::uvSplitViewEnabled() const
{
    return m_uvSplitViewCheckBox && m_uvSplitViewCheckBox->isChecked();
}

void PropertyPanel::setupConnections()
{
    // Connect translation/rotation spinboxes to sync matrix
    connect(ui->spinTransX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);
    connect(ui->spinTransY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);
    connect(ui->spinTransZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);
    connect(ui->spinRotX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);
    connect(ui->spinRotY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);
    connect(ui->spinRotZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseMatrixFromTranslationRotation);

    // Connect matrix spinboxes to sync translation/rotation
    connect(ui->spinM00, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM01, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM02, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM03, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM10, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM11, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM12, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM13, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM20, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM21, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM22, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM23, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM30, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM31, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM32, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);
    connect(ui->spinM33, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::syncPoseTranslationRotationFromMatrix);

    // Simulation page connections
    connect(ui->btnBrowseTrajectory, &QPushButton::clicked, [this]() {
        // Check if workspace is active before allowing browse
        if (!m_hasActiveWorkspace) {
            // Show a message box to inform user
            QMessageBox::information(this,
                tr("No Active Workspace"),
                tr("Please create or open a workspace first.\n\n"
                   "Go to File > New Project or File > Load Project to get started."),
                QMessageBox::Ok);
            return;
        }
        // Emit signal if workspace is active
        emit browseTrajectoryRequested();
    });
    connect(ui->sliderFrame, &QSlider::valueChanged,
            this, &PropertyPanel::frameSliderChanged);
    connect(ui->spinPlaybackSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::playbackSpeedChanged);

    connect(ui->checkShowModel, &QCheckBox::toggled,
            this, &PropertyPanel::modelVisibilityChanged);
    connect(ui->checkShowDeposition, &QCheckBox::toggled,
            this, &PropertyPanel::depositionVisibilityChanged);
    connect(ui->checkShowSprayGun, &QCheckBox::toggled,
            this, &PropertyPanel::sprayGunPoseVisibilityChanged);
    connect(ui->checkShowTrajectoryPoints, &QCheckBox::toggled,
            this, &PropertyPanel::trajectoryPointsVisibilityChanged);
    connect(ui->checkShowTrajectoryLine, &QCheckBox::toggled,
            this, &PropertyPanel::trajectoryLineVisibilityChanged);
    connect(ui->checkPickThickness, &QCheckBox::toggled,
            this, &PropertyPanel::thicknessPickingChanged);

    // Enable/disable trajectory file controls based on radio button
    connect(ui->radioImportTrajectoryFile, &QRadioButton::toggled, [this](bool checked) {
        ui->lineTrajectoryFile->setEnabled(checked);
        ui->btnBrowseTrajectory->setEnabled(checked);
    });
}

void PropertyPanel::setWorkspaceActive(bool active)
{
    m_hasActiveWorkspace = active;

    // Enable/disable "Open Slicing Control Dialog" button based on workspace state
    if (m_openSlicingDialogButton) {
        m_openSlicingDialogButton->setEnabled(active);
    }
}

// ============================================================================
// Collision Optimization - Getters/Setters
// ============================================================================

void PropertyPanel::setOptimizationAlgorithms(const QStringList& algorithms, int currentIndex)
{
    ui->comboAlgorithm->clear();
    ui->comboAlgorithm->addItems(algorithms);
    if (currentIndex >= 0 && currentIndex < algorithms.size()) {
        ui->comboAlgorithm->setCurrentIndex(currentIndex);
    }

    // Connect algorithm change signal
    connect(ui->comboAlgorithm, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::optimizationAlgorithmChanged, Qt::UniqueConnection);
}

int PropertyPanel::optimizationAlgorithmIndex() const
{
    return ui->comboAlgorithm->currentIndex();
}

QString PropertyPanel::optimizationAlgorithmText() const
{
    return ui->comboAlgorithm->currentText();
}

double PropertyPanel::safetyDistanceMm() const
{
    return ui->spinSafetyDistance->value();
}

int PropertyPanel::maxIterations() const
{
    return ui->spinMaxIterations->value();
}

bool PropertyPanel::showCollisionPoints() const
{
    return ui->checkShowCollisionPoints->isChecked();
}

bool PropertyPanel::showOriginalTrajectory() const
{
    return ui->checkShowOriginalTraj->isChecked();
}

bool PropertyPanel::showOptimizedTrajectory() const
{
    return ui->checkShowOptimizedTraj->isChecked();
}

bool PropertyPanel::showOptimizationProcess() const
{
    return ui->checkShowOptimizationProcess->isChecked();
}

// ============================================================================
// Pose Transformation - Getters/Setters
// ============================================================================

QVector<double> PropertyPanel::transformTranslationMm() const
{
    QVector<double> trans(3);
    trans[0] = ui->spinTransX->value();
    trans[1] = ui->spinTransY->value();
    trans[2] = ui->spinTransZ->value();
    return trans;
}

QVector<double> PropertyPanel::transformRotationDeg() const
{
    QVector<double> rot(3);
    rot[0] = ui->spinRotX->value();
    rot[1] = ui->spinRotY->value();
    rot[2] = ui->spinRotZ->value();
    return rot;
}

QVector<double> PropertyPanel::transformMatrixValues() const
{
    QVector<double> values(16);
    values[0] = ui->spinM00->value(); values[1] = ui->spinM01->value();
    values[2] = ui->spinM02->value(); values[3] = ui->spinM03->value();
    values[4] = ui->spinM10->value(); values[5] = ui->spinM11->value();
    values[6] = ui->spinM12->value(); values[7] = ui->spinM13->value();
    values[8] = ui->spinM20->value(); values[9] = ui->spinM21->value();
    values[10] = ui->spinM22->value(); values[11] = ui->spinM23->value();
    values[12] = ui->spinM30->value(); values[13] = ui->spinM31->value();
    values[14] = ui->spinM32->value(); values[15] = ui->spinM33->value();
    return values;
}

void PropertyPanel::setTransformTranslationMm(const QVector<double>& values)
{
    if (values.size() != 3) return;

    m_syncingTransformInputs = true;
    ui->spinTransX->setValue(values[0]);
    ui->spinTransY->setValue(values[1]);
    ui->spinTransZ->setValue(values[2]);
    m_syncingTransformInputs = false;
}

void PropertyPanel::setTransformRotationDeg(const QVector<double>& values)
{
    if (values.size() != 3) return;

    m_syncingTransformInputs = true;
    ui->spinRotX->setValue(values[0]);
    ui->spinRotY->setValue(values[1]);
    ui->spinRotZ->setValue(values[2]);
    m_syncingTransformInputs = false;
}

void PropertyPanel::setTransformMatrixValues(const QVector<double>& values)
{
    if (values.size() != 16) return;

    m_syncingTransformInputs = true;
    ui->spinM00->setValue(values[0]); ui->spinM01->setValue(values[1]);
    ui->spinM02->setValue(values[2]); ui->spinM03->setValue(values[3]);
    ui->spinM10->setValue(values[4]); ui->spinM11->setValue(values[5]);
    ui->spinM12->setValue(values[6]); ui->spinM13->setValue(values[7]);
    ui->spinM20->setValue(values[8]); ui->spinM21->setValue(values[9]);
    ui->spinM22->setValue(values[10]); ui->spinM23->setValue(values[11]);
    ui->spinM30->setValue(values[12]); ui->spinM31->setValue(values[13]);
    ui->spinM32->setValue(values[14]); ui->spinM33->setValue(values[15]);
    m_syncingTransformInputs = false;
}

void PropertyPanel::resetTransformInputs()
{
    m_syncingTransformInputs = true;

    // Reset translation to 0
    ui->spinTransX->setValue(0.0);
    ui->spinTransY->setValue(0.0);
    ui->spinTransZ->setValue(0.0);

    // Reset rotation to 0
    ui->spinRotX->setValue(0.0);
    ui->spinRotY->setValue(0.0);
    ui->spinRotZ->setValue(0.0);

    // Reset matrix to identity
    ui->spinM00->setValue(1.0); ui->spinM01->setValue(0.0);
    ui->spinM02->setValue(0.0); ui->spinM03->setValue(0.0);
    ui->spinM10->setValue(0.0); ui->spinM11->setValue(1.0);
    ui->spinM12->setValue(0.0); ui->spinM13->setValue(0.0);
    ui->spinM20->setValue(0.0); ui->spinM21->setValue(0.0);
    ui->spinM22->setValue(1.0); ui->spinM23->setValue(0.0);
    ui->spinM30->setValue(0.0); ui->spinM31->setValue(0.0);
    ui->spinM32->setValue(0.0); ui->spinM33->setValue(1.0);

    m_syncingTransformInputs = false;
}

bool PropertyPanel::transformImportedModelChecked() const
{
    return ui->checkTransformModel->isChecked();
}

bool PropertyPanel::transformImportedTrajectoryChecked() const
{
    return ui->checkTransformTrajectory->isChecked();
}

bool PropertyPanel::transformExtraModelChecked() const
{
    return ui->checkTransformExtraModel->isChecked();
}

void PropertyPanel::syncPoseMatrixFromTranslationRotation()
{
    if (m_syncingTransformInputs) return;

    // Get translation and rotation from UI
    QVector<double> translation = transformTranslationMm();
    QVector<double> rotation = transformRotationDeg();

    // Convert to matrix
    QVector<double> matrix = matrixFromTranslationRotation(
        translation[0], translation[1], translation[2],
        rotation[0], rotation[1], rotation[2]);

    // Update matrix display
    m_syncingTransformInputs = true;
    setTransformMatrixValues(matrix);
    m_syncingTransformInputs = false;

    // Signal that translation/rotation changed
    emit transformTranslationRotationChanged();
}

void PropertyPanel::syncPoseTranslationRotationFromMatrix()
{
    if (m_syncingTransformInputs) return;

    // Get matrix from UI
    QVector<double> matrix = transformMatrixValues();

    // Extract translation and rotation
    double tx, ty, tz, rx, ry, rz;
    extractTranslationRotation(matrix, tx, ty, tz, rx, ry, rz);

    // Update translation and rotation display
    m_syncingTransformInputs = true;
    setTransformTranslationMm({tx, ty, tz});
    setTransformRotationDeg({rx, ry, rz});
    m_syncingTransformInputs = false;

    // Signal that matrix was edited
    emit transformMatrixEdited();
}

QVector<double> PropertyPanel::getTransformMatrixFlattened() const
{
    return transformMatrixValues();
}

void PropertyPanel::setTransformMatrixFlattened(const QVector<double>& matrix16)
{
    m_syncingTransformInputs = true;

    // Set matrix values
    setTransformMatrixValues(matrix16);

    // Extract and set translation/rotation
    double tx, ty, tz, rx, ry, rz;
    extractTranslationRotation(matrix16, tx, ty, tz, rx, ry, rz);
    setTransformTranslationMm({tx, ty, tz});
    setTransformRotationDeg({rx, ry, rz});

    m_syncingTransformInputs = false;
}

// Static utility methods for matrix conversion (using QVector instead of Eigen)
QVector<double> PropertyPanel::matrixFromTranslationRotation(
    double tx, double ty, double tz,
    double rxDeg, double ryDeg, double rzDeg)
{
    constexpr double pi = 3.14159265358979323846;
    const double rx = rxDeg * pi / 180.0;
    const double ry = ryDeg * pi / 180.0;
    const double rz = rzDeg * pi / 180.0;

    // Compute rotation matrix elements (ZYX Euler)
    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);

    // R = Rz * Ry * Rx.  The extraction routine below uses this convention,
    // so the two directions must remain exact inverses.
    QVector<double> matrix(16);
    matrix[0]  = cz * cy;
    matrix[1]  = cz * sy * sx - sz * cx;
    matrix[2]  = cz * sy * cx + sz * sx;
    matrix[3]  = tx;

    matrix[4]  = sz * cy;
    matrix[5]  = sz * sy * sx + cz * cx;
    matrix[6]  = sz * sy * cx - cz * sx;
    matrix[7]  = ty;

    matrix[8]  = -sy;
    matrix[9]  = cy * sx;
    matrix[10] = cy * cx;
    matrix[11] = tz;

    matrix[12] = 0.0;
    matrix[13] = 0.0;
    matrix[14] = 0.0;
    matrix[15] = 1.0;

    return matrix;
}

void PropertyPanel::extractTranslationRotation(
    const QVector<double>& matrix16,
    double& tx, double& ty, double& tz,
    double& rxDeg, double& ryDeg, double& rzDeg)
{
    constexpr double pi = 3.14159265358979323846;

    // Extract translation
    tx = matrix16[3];
    ty = matrix16[7];
    tz = matrix16[11];

    // Extract rotation (ZYX Euler angles)
    double r00 = matrix16[0], r01 = matrix16[1], r02 = matrix16[2];
    double r10 = matrix16[4], r11 = matrix16[5], r12 = matrix16[6];
    double r20 = matrix16[8], r21 = matrix16[9], r22 = matrix16[10];

    double sy = std::sqrt(r00 * r00 + r10 * r10);
    bool singular = sy < 1e-6;

    double rx, ry, rz;
    if (!singular) {
        rx = std::atan2(r21, r22);
        ry = std::atan2(-r20, sy);
        rz = std::atan2(r10, r00);
    } else {
        rx = std::atan2(-r12, r11);
        ry = std::atan2(-r20, sy);
        rz = 0.0;
    }

    rxDeg = rx * 180.0 / pi;
    ryDeg = ry * 180.0 / pi;
    rzDeg = rz * 180.0 / pi;
}

// ============================================================================
// Thickness Prediction - Getters
// ============================================================================

int PropertyPanel::getThicknessDepositionFunctionIndex() const
{
    return ui->comboDepositionFunction->currentIndex();
}

double PropertyPanel::getThicknessSprayWidth() const
{
    return ui->spinSprayWidth->value();
}

double PropertyPanel::getThicknessDepositionEfficiency() const
{
    return ui->spinDepositionEfficiency->value();
}

double PropertyPanel::getThicknessTargetThickness() const
{
    return ui->spinTargetThickness->value();
}

double PropertyPanel::getThicknessQualityFactor() const
{
    return ui->spinQualityFactor->value();
}

double PropertyPanel::getThicknessDefaultVelocity() const
{
    return ui->spinDefaultVelocity->value();
}

double PropertyPanel::getTrajectoryOptimalSpeed() const
{
    if (!m_slicingWidget) {
        return 0.0;
    }

    const auto* speedSpinBox = m_slicingWidget->findChild<QDoubleSpinBox*>(
        QStringLiteral("normalVelocity_SpinBox"));
    return speedSpinBox ? speedSpinBox->value() : 0.0;
}

bool PropertyPanel::showModel() const
{
    return ui->checkShowModel->isChecked();
}

bool PropertyPanel::showDepositionResult() const
{
    return ui->checkShowDeposition->isChecked();
}

bool PropertyPanel::showSprayGunPose() const
{
    return ui->checkShowSprayGun->isChecked();
}

bool PropertyPanel::showTrajectoryPoints() const
{
    return ui->checkShowTrajectoryPoints->isChecked();
}

bool PropertyPanel::showTrajectoryLine() const
{
    return ui->checkShowTrajectoryLine->isChecked();
}

bool PropertyPanel::pickThicknessOnHover() const
{
    return ui->checkPickThickness->isChecked();
}

void PropertyPanel::setShowModel(bool visible)
{
    ui->checkShowModel->setChecked(visible);
}

// ============================================================================
// Thickness Prediction - Data initialization
// ============================================================================

