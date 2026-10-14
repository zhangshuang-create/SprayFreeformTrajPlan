#include "FreeformTrajPlanWorkbenchPanel.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{
    QDoubleSpinBox* makeSpinBox(QWidget* parent, double value, double minimum,
        const QString& suffix = QStringLiteral(" mm"))
    {
        auto* spinBox = new QDoubleSpinBox(parent);
        spinBox->setRange(minimum, 100000.0);
        spinBox->setDecimals(3);
        spinBox->setValue(value);
        spinBox->setSuffix(suffix);
        return spinBox;
    }

    QPushButton* addActionButton(QVBoxLayout& layout, QWidget* parent, const QString& text,
        spray_freeform_traj_plan::WorkflowAction action,
        smrobot::workbench::freeform::FreeformTrajPlanWorkflowPanel& panel)
    {
        auto* button = new QPushButton(text, parent);
        QObject::connect(button, &QPushButton::clicked, &panel,
            [&panel, action]() { emit panel.workflowActionRequested(action); });
        layout.addWidget(button);
        return button;
    }
}

namespace smrobot::workbench::freeform
{
    FreeformTrajPlanWorkflowPanel::FreeformTrajPlanWorkflowPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSizeConstraint(QLayout::SetMinimumSize);
        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        layout->setSpacing(8);

        auto* title = new QLabel(QStringLiteral("曲面轨迹规划"), this);
        title->setProperty("panelTitle", true);
        layout->addWidget(title);

        auto* planningMethodGroup = new QGroupBox(QStringLiteral("规划方法"), this);
        auto* planningMethodLayout = new QVBoxLayout(planningMethodGroup);
        auto* planningMethod = new QComboBox(planningMethodGroup);
        planningMethod->addItem(QStringLiteral("切片规划"), false);
        planningMethod->addItem(QStringLiteral("UV 规划"), true);
        planningMethodLayout->addWidget(planningMethod);
        connect(planningMethod, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [planningMethod, this](int index) {
                const bool uv = planningMethod->itemData(index).toBool();
                m_workflowStack->setCurrentIndex(uv ? 1 : 0);
                emit planningMethodRequested(uv);
            });
        layout->addWidget(planningMethodGroup);

        auto* importGroup = new QGroupBox(QStringLiteral("模型数据"), this);
        auto* importLayout = new QVBoxLayout(importGroup);
        m_modelPath = new QLineEdit(importGroup);
        m_modelPath->setReadOnly(true);
        m_modelPath->setPlaceholderText(QStringLiteral("选择曲面模型文件..."));
        auto* importButton = new QPushButton(QStringLiteral("导入模型"), importGroup);
        importLayout->addWidget(m_modelPath);
        importLayout->addWidget(importButton);
        connect(importButton, &QPushButton::clicked, this, [this]() {
            const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入曲面模型"),
                QString(), QStringLiteral("STL surface models (*.stl *.STL);;All files (*.*)"));
            if(!path.isEmpty()) {
                emit importModelRequested(path);
            }
        });
        layout->addWidget(importGroup);

        m_workflowStack = new QStackedWidget(this);
        auto* slicingPage = new QWidget(m_workflowStack);
        auto* slicingLayout = new QVBoxLayout(slicingPage);
        slicingLayout->setContentsMargins(0, 0, 0, 0);
        auto* slicingMethodGroup = new QGroupBox(QStringLiteral("切片方法"), slicingPage);
        auto* slicingMethodLayout = new QVBoxLayout(slicingMethodGroup);
        auto* slicingMethod = new QComboBox(slicingMethodGroup);
        slicingMethod->addItem(QStringLiteral("自适应切片"), true);
        slicingMethod->addItem(QStringLiteral("等距切片"), false);
        slicingMethodLayout->addWidget(slicingMethod);
        connect(slicingMethod, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [slicingMethod, this](int index) {
                emit slicingMethodRequested(slicingMethod->itemData(index).toBool());
            });
        slicingLayout->addWidget(slicingMethodGroup);
        auto* slicingActions = new QGroupBox(QStringLiteral("切片流程"), slicingPage);
        auto* slicingActionsLayout = new QVBoxLayout(slicingActions);
        addActionButton(*slicingActionsLayout, slicingActions, QStringLiteral("开始切片"),
            spray_freeform_traj_plan::WorkflowAction::StartSlicing, *this);
        addActionButton(*slicingActionsLayout, slicingActions, QStringLiteral("轨迹姿态配置"),
            spray_freeform_traj_plan::WorkflowAction::ConfigureTrajectoryFrame, *this);
        addActionButton(*slicingActionsLayout, slicingActions, QStringLiteral("生成轨迹"),
            spray_freeform_traj_plan::WorkflowAction::GenerateTrajectory, *this);
        addActionButton(*slicingActionsLayout, slicingActions, QStringLiteral("连接轨迹"),
            spray_freeform_traj_plan::WorkflowAction::ConnectTrajectory, *this);
        addActionButton(*slicingActionsLayout, slicingActions, QStringLiteral("切片调整"),
            spray_freeform_traj_plan::WorkflowAction::AdjustSlicing, *this);
        slicingLayout->addWidget(slicingActions);
        m_workflowStack->addWidget(slicingPage);

        auto* uvPage = new QWidget(m_workflowStack);
        auto* uvLayout = new QVBoxLayout(uvPage);
        uvLayout->setContentsMargins(0, 0, 0, 0);
        auto* uvActions = new QGroupBox(QStringLiteral("UV 规划流程"), uvPage);
        auto* uvActionsLayout = new QVBoxLayout(uvActions);
        addActionButton(*uvActionsLayout, uvActions, QStringLiteral("开始 UV 图"),
            spray_freeform_traj_plan::WorkflowAction::StartUvMap, *this);
        addActionButton(*uvActionsLayout, uvActions, QStringLiteral("生成轨迹"),
            spray_freeform_traj_plan::WorkflowAction::GenerateUvTrajectory, *this);
        addActionButton(*uvActionsLayout, uvActions, QStringLiteral("开始优化"),
            spray_freeform_traj_plan::WorkflowAction::OptimizeUv, *this);
        uvLayout->addWidget(uvActions);
        uvLayout->addStretch(1);
        m_workflowStack->addWidget(uvPage);
        layout->addWidget(m_workflowStack);

        auto* outputGroup = new QGroupBox(QStringLiteral("轨迹数据"), this);
        auto* outputLayout = new QVBoxLayout(outputGroup);
        auto* exportButton = new QPushButton(QStringLiteral("导出轨迹"), outputGroup);
        connect(exportButton, &QPushButton::clicked, this, [this]() {
            const QString paqFilter = QStringLiteral("PAQ Trajectory (*.txt)");
            const QString matrixFilter = QStringLiteral("Pose Matrix + Timestamp (*.txt)");
            const QString abbFilter = QStringLiteral("ABB RAPID Module (*.mod)");
            const QString fanucFilter = QStringLiteral("FANUC LS Program (*.LS *.ls)");
            QFileDialog dialog(this, QStringLiteral("导出轨迹"));
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            dialog.selectFile(QStringLiteral("freeform_trajectory_paq.txt"));
            dialog.setNameFilters({ paqFilter, matrixFilter, abbFilter, fanucFilter, QStringLiteral("All Files (*)") });
            dialog.selectNameFilter(paqFilter);
            dialog.setDefaultSuffix(QStringLiteral("txt"));
            if(dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
                return;
            }

            QString path = dialog.selectedFiles().front();
            const QString selectedFilter = dialog.selectedNameFilter();
            FreeformTrajPlanExportFormat format = FreeformTrajPlanExportFormat::Paq;
            QString suffix = QStringLiteral("txt");
            if(selectedFilter == matrixFilter) {
                format = FreeformTrajPlanExportFormat::Matrix;
            } else if(selectedFilter == abbFilter) {
                format = FreeformTrajPlanExportFormat::Abb;
                suffix = QStringLiteral("mod");
            } else if(selectedFilter == fanucFilter) {
                format = FreeformTrajPlanExportFormat::Fanuc;
                suffix = QStringLiteral("ls");
            }
            const QString extension = QStringLiteral(".") + suffix;
            if(!path.endsWith(extension, Qt::CaseInsensitive)) {
                path += extension;
            }
            QDialog options(this);
            options.setWindowTitle(QStringLiteral("轨迹导出设置"));
            auto* form = new QFormLayout(&options);
            auto* interpolate = new QCheckBox(QStringLiteral("按时间插值"), &options);
            interpolate->setChecked(false);
            auto* interval = new QDoubleSpinBox(&options);
            interval->setDecimals(6);
            interval->setRange(0.000001, 3600.0);
            interval->setValue(0.005);
            interval->setSuffix(QStringLiteral(" s"));
            interval->setEnabled(false);
            connect(interpolate, &QCheckBox::toggled, interval, &QWidget::setEnabled);
            form->addRow(interpolate);
            form->addRow(QStringLiteral("插值时间步长"), interval);
            auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &options);
            connect(buttons, &QDialogButtonBox::accepted, &options, &QDialog::accept);
            connect(buttons, &QDialogButtonBox::rejected, &options, &QDialog::reject);
            form->addRow(buttons);
            if(options.exec() != QDialog::Accepted) return;
            emit exportTrajectoryRequested(path, format, interpolate->isChecked() ? interval->value() : 0.0);
        });
        outputLayout->addWidget(exportButton);
        addActionButton(*outputLayout, outputGroup, QStringLiteral("位姿转换"),
            spray_freeform_traj_plan::WorkflowAction::TransformTrajectory, *this);
        layout->addWidget(outputGroup);

        auto* clearGroup = new QGroupBox(QStringLiteral("清除数据"), this);
        auto* clearLayout = new QVBoxLayout(clearGroup);
        addActionButton(*clearLayout, clearGroup, QStringLiteral("模型数据"),
            spray_freeform_traj_plan::WorkflowAction::ClearModelData, *this);
        addActionButton(*clearLayout, clearGroup, QStringLiteral("轨迹数据"),
            spray_freeform_traj_plan::WorkflowAction::ClearTrajectoryData, *this);
        addActionButton(*clearLayout, clearGroup, QStringLiteral("连接数据"),
            spray_freeform_traj_plan::WorkflowAction::ClearConnectionData, *this);
        addActionButton(*clearLayout, clearGroup, QStringLiteral("其他数据"),
            spray_freeform_traj_plan::WorkflowAction::ClearOtherData, *this);
        addActionButton(*clearLayout, clearGroup, QStringLiteral("全部清除"),
            spray_freeform_traj_plan::WorkflowAction::ClearAllData, *this);
        layout->addWidget(clearGroup);

        m_status = new QLabel(this);
        m_status->setWordWrap(true);
        layout->addWidget(m_status);
        layout->addStretch(1);
    }

    void FreeformTrajPlanWorkflowPanel::setModelPath(const QString& path)
    {
        m_modelPath->setText(path);
    }

    void FreeformTrajPlanWorkflowPanel::setStatus(const QString& message, bool success)
    {
        m_status->setText(message);
        m_status->setProperty("success", success);
    }

    void FreeformTrajPlanWorkflowPanel::setPlanningMethod(
        spray_freeform_traj_plan::PlanningMethod method)
    {
        m_workflowStack->setCurrentIndex(
            method == spray_freeform_traj_plan::PlanningMethod::Uv ? 1 : 0);
    }

    FreeformTrajPlanParameterPanel::FreeformTrajPlanParameterPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(8, 8, 8, 8);
        layout->setSizeConstraint(QLayout::SetMinimumSize);
        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        auto* title = new QLabel(QStringLiteral("轨迹参数"), this);
        title->setProperty("panelTitle", true);
        layout->addWidget(title);

        m_parameterStack = new QStackedWidget(this);
        auto* slicingPage = new QWidget(m_parameterStack);
        auto* slicingLayout = new QVBoxLayout(slicingPage);
        slicingLayout->setContentsMargins(0, 0, 0, 0);
        auto* slicingGroup = new QGroupBox(QStringLiteral("切片规划参数"), slicingPage);
        auto* slicingForm = new QFormLayout(slicingGroup);
        m_trajectorySpacing = makeSpinBox(slicingGroup, 1.0, 0.001);
        m_trajectorySpacing->setToolTip(QStringLiteral("单层轮廓的弧长取点间距；轨迹数量为 0 时计算公共点数，各层点数相同。"));
        m_chordError = makeSpinBox(slicingGroup, 5.0, 0.001, QString());
        m_chordError->setDecimals(6);
        m_minimumSpacing = makeSpinBox(slicingGroup, 50.0, 0.001);
        m_minimumSpacing->setToolTip(QStringLiteral("非双边界模式：沿 Z 的切片间距；切片层数为正数时不使用此步距。"));
        m_largeEndOffset = makeSpinBox(slicingGroup, 0.0, 0.0);
        m_smallEndOffset = makeSpinBox(slicingGroup, 0.0, 0.0);
        m_sprayDistance = makeSpinBox(slicingGroup, 1.0, -100000.0);
        m_sprayDistance->setToolTip(QStringLiteral(
            "有符号的法向偏移距离：负值相对于正值向相反一侧偏移，绝对值为偏移距离（mm）。"));
        m_speed = makeSpinBox(slicingGroup, 0.0, 0.0, QStringLiteral(" mm/s"));
        m_trajectoryCount = new QSpinBox(slicingGroup);
        m_trajectoryCount->setRange(0, 100000);
        m_trajectoryCount->setToolTip(QStringLiteral("0 按轨迹间距计算各层公共点数；正数覆盖间距，指定每层点数（至少 2，支持奇偶数）。"));
        m_maxSliceCount = new QSpinBox(slicingGroup);
        m_maxSliceCount->setRange(0, 100000);
        m_maxSliceCount->setToolTip(QStringLiteral("非双边界模式：0 按切片层间距计算 Z 向切片层数；正数指定层数并覆盖步距，1 层位于高度中点。双边界模式保持原切片层数限制。"));
        slicingForm->addRow(QStringLiteral("轨迹间距"), m_trajectorySpacing);
        slicingForm->addRow(QStringLiteral("弦误差容差"), m_chordError);
        slicingForm->addRow(QStringLiteral("切片层间距"), m_minimumSpacing);
        slicingForm->addRow(QStringLiteral("大端偏置"), m_largeEndOffset);
        slicingForm->addRow(QStringLiteral("小端偏置"), m_smallEndOffset);
        slicingForm->addRow(QStringLiteral("最佳喷涂距离"), m_sprayDistance);
        slicingForm->addRow(QStringLiteral("最佳速度"), m_speed);
        slicingForm->addRow(QStringLiteral("轨迹数量"), m_trajectoryCount);
        slicingForm->addRow(QStringLiteral("切片层数"), m_maxSliceCount);
        slicingLayout->addWidget(slicingGroup);

        auto* visibilityGroup = new QGroupBox(QStringLiteral("可视化"), slicingPage);
        auto* visibilityLayout = new QVBoxLayout(visibilityGroup);
        auto* modelVisible = new QCheckBox(QStringLiteral("显示模型"), visibilityGroup);
        auto* slicesVisible = new QCheckBox(QStringLiteral("显示切片"), visibilityGroup);
        auto* trajectoriesVisible = new QCheckBox(QStringLiteral("显示轨迹"), visibilityGroup);
        auto* connectionsVisible = new QCheckBox(QStringLiteral("显示连线"), visibilityGroup);
        auto* referenceLinesVisible = new QCheckBox(QStringLiteral("显示参考线"), visibilityGroup);
        for(QCheckBox* checkBox : { modelVisible, slicesVisible, trajectoriesVisible, connectionsVisible,
                referenceLinesVisible }) {
            checkBox->setChecked(true);
            visibilityLayout->addWidget(checkBox);
        }
        slicingLayout->addWidget(visibilityGroup);

        auto* tcpGroup = new QGroupBox(QStringLiteral("导出 TCP"), slicingPage);
        auto* tcpLayout = new QVBoxLayout(tcpGroup);
        m_tcpEnabled = new QCheckBox(QStringLiteral("在导出轨迹中附加 TCP"), tcpGroup);
        tcpLayout->addWidget(m_tcpEnabled);
        auto* tcpFormatForm = new QFormLayout();
        m_tcpFormat = new QComboBox(tcpGroup);
        m_tcpFormat->addItem(QStringLiteral("ABB"));
        m_tcpFormat->addItem(QStringLiteral("FANUC"));
        tcpFormatForm->addRow(QStringLiteral("控制器"), m_tcpFormat);
        tcpLayout->addLayout(tcpFormatForm);

        const auto makeTcpSpinBox = [](QWidget* parent, double value, const QString& suffix) {
            auto* spinBox = makeSpinBox(parent, value, -100000.0, suffix);
            spinBox->setDecimals(6);
            return spinBox;
        };
        m_tcpInputStack = new QStackedWidget(tcpGroup);
        auto* abbTcpPage = new QWidget(m_tcpInputStack);
        auto* abbTcpGrid = new QGridLayout(abbTcpPage);
        m_tcpX = makeTcpSpinBox(abbTcpPage, 0.0, QStringLiteral(" mm"));
        m_tcpY = makeTcpSpinBox(abbTcpPage, 0.0, QStringLiteral(" mm"));
        m_tcpZ = makeTcpSpinBox(abbTcpPage, 0.0, QStringLiteral(" mm"));
        m_tcpQ1 = makeTcpSpinBox(abbTcpPage, 1.0, QString());
        m_tcpQ2 = makeTcpSpinBox(abbTcpPage, 0.0, QString());
        m_tcpQ3 = makeTcpSpinBox(abbTcpPage, 0.0, QString());
        m_tcpQ4 = makeTcpSpinBox(abbTcpPage, 0.0, QString());
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("X"), abbTcpPage), 0, 0);
        abbTcpGrid->addWidget(m_tcpX, 0, 1);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Y"), abbTcpPage), 1, 0);
        abbTcpGrid->addWidget(m_tcpY, 1, 1);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Z"), abbTcpPage), 2, 0);
        abbTcpGrid->addWidget(m_tcpZ, 2, 1);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Q1"), abbTcpPage), 0, 2);
        abbTcpGrid->addWidget(m_tcpQ1, 0, 3);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Q2"), abbTcpPage), 1, 2);
        abbTcpGrid->addWidget(m_tcpQ2, 1, 3);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Q3"), abbTcpPage), 2, 2);
        abbTcpGrid->addWidget(m_tcpQ3, 2, 3);
        abbTcpGrid->addWidget(new QLabel(QStringLiteral("Q4"), abbTcpPage), 3, 2);
        abbTcpGrid->addWidget(m_tcpQ4, 3, 3);
        m_tcpInputStack->addWidget(abbTcpPage);

        auto* fanucTcpPage = new QWidget(m_tcpInputStack);
        auto* fanucTcpGrid = new QGridLayout(fanucTcpPage);
        auto* fanucX = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" mm"));
        auto* fanucY = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" mm"));
        auto* fanucZ = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" mm"));
        m_tcpW = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" deg"));
        m_tcpP = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" deg"));
        m_tcpR = makeTcpSpinBox(fanucTcpPage, 0.0, QStringLiteral(" deg"));
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("X"), fanucTcpPage), 0, 0);
        fanucTcpGrid->addWidget(fanucX, 0, 1);
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("Y"), fanucTcpPage), 1, 0);
        fanucTcpGrid->addWidget(fanucY, 1, 1);
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("Z"), fanucTcpPage), 2, 0);
        fanucTcpGrid->addWidget(fanucZ, 2, 1);
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("W"), fanucTcpPage), 0, 2);
        fanucTcpGrid->addWidget(m_tcpW, 0, 3);
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("P"), fanucTcpPage), 1, 2);
        fanucTcpGrid->addWidget(m_tcpP, 1, 3);
        fanucTcpGrid->addWidget(new QLabel(QStringLiteral("R"), fanucTcpPage), 2, 2);
        fanucTcpGrid->addWidget(m_tcpR, 2, 3);
        m_tcpInputStack->addWidget(fanucTcpPage);
        tcpLayout->addWidget(m_tcpInputStack);
        slicingLayout->addWidget(tcpGroup);
        m_tcpFormat->setEnabled(false);
        m_tcpInputStack->setEnabled(false);
        slicingLayout->addStretch(1);
        m_parameterStack->addWidget(slicingPage);

        auto* uvPage = new QWidget(m_parameterStack);
        auto* uvLayout = new QVBoxLayout(uvPage);
        uvLayout->setContentsMargins(0, 0, 0, 0);
        auto* uvGroup = new QGroupBox(QStringLiteral("UV 规划参数"), uvPage);
        auto* uvForm = new QFormLayout(uvGroup);
        m_uvTargetSpacing = makeSpinBox(uvGroup, 30.0, 0.001);
        m_uvTrajectorySpeed = makeSpinBox(uvGroup, 300.0, 0.0, QStringLiteral(" mm/s"));
        m_uvSprayDistance = makeSpinBox(uvGroup, 0.0, 0.0);
        m_uvMaximumSeamDistance = makeSpinBox(uvGroup, 78.0, 0.0);
        m_uvMaximumAcceleration = makeSpinBox(uvGroup, 1000.0, 0.001, QStringLiteral(" mm/s2"));
        m_uvMaximumJerk = makeSpinBox(uvGroup, 5000.0, 0.001, QStringLiteral(" mm/s3"));
        m_uvOvertravelLength = makeSpinBox(uvGroup, 4.0, 0.0, QString());
        m_uvArcSpeedLoss = makeSpinBox(uvGroup, 0.0, 0.0, QStringLiteral(" %"));
        m_uvShowSplitView = new QCheckBox(QStringLiteral("显示 UV 分割视图"), uvGroup);
        uvForm->addRow(QStringLiteral("目标步距"), m_uvTargetSpacing);
        uvForm->addRow(QStringLiteral("轨迹速度"), m_uvTrajectorySpeed);
        uvForm->addRow(QStringLiteral("喷涂距离"), m_uvSprayDistance);
        uvForm->addRow(QStringLiteral("最大缝隙喷涂距离"), m_uvMaximumSeamDistance);
        uvForm->addRow(QStringLiteral("最大加速度"), m_uvMaximumAcceleration);
        uvForm->addRow(QStringLiteral("最大加加速度"), m_uvMaximumJerk);
        uvForm->addRow(QStringLiteral("超程长度"), m_uvOvertravelLength);
        uvForm->addRow(QStringLiteral("弧段速度损失"), m_uvArcSpeedLoss);
        uvForm->addRow(m_uvShowSplitView);
        auto* generateButton = new QPushButton(QStringLiteral("生成轨迹"), uvGroup);
        uvForm->addRow(generateButton);
        connect(generateButton, &QPushButton::clicked, this, [this]() {
            emit workflowActionRequested(spray_freeform_traj_plan::WorkflowAction::GenerateUvTrajectory);
        });
        uvLayout->addWidget(uvGroup);
        uvLayout->addStretch(1);
        m_parameterStack->addWidget(uvPage);
        layout->addWidget(m_parameterStack);

        const auto publishSlicing = [this]() {
            emit parametersChanged(m_trajectorySpacing->value(), m_chordError->value(),
                m_minimumSpacing->value(), m_largeEndOffset->value(), m_smallEndOffset->value(),
                m_sprayDistance->value(), m_speed->value(), m_trajectoryCount->value(),
                m_maxSliceCount->value());
        };
        for(QDoubleSpinBox* spinBox : { m_trajectorySpacing, m_chordError, m_minimumSpacing,
                m_largeEndOffset, m_smallEndOffset, m_sprayDistance, m_speed }) {
            connect(spinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [publishSlicing](double) { publishSlicing(); });
        }
        for(QSpinBox* spinBox : { m_trajectoryCount, m_maxSliceCount }) {
            connect(spinBox, qOverload<int>(&QSpinBox::valueChanged), this,
                [publishSlicing](int) { publishSlicing(); });
        }
        const auto publishVisibility = [this, modelVisible, slicesVisible, trajectoriesVisible,
                                            connectionsVisible, referenceLinesVisible]() {
            emit visualizationVisibilityChanged(modelVisible->isChecked(), slicesVisible->isChecked(),
                trajectoriesVisible->isChecked(), connectionsVisible->isChecked(),
                referenceLinesVisible->isChecked());
        };
        for(QCheckBox* checkBox : { modelVisible, slicesVisible, trajectoriesVisible, connectionsVisible,
                referenceLinesVisible }) {
            connect(checkBox, &QCheckBox::toggled, this, [publishVisibility](bool) { publishVisibility(); });
        }
        const auto publishUv = [this]() {
            emit uvParametersChanged(m_uvTargetSpacing->value(), m_uvTrajectorySpeed->value(),
                m_uvSprayDistance->value(), m_uvMaximumSeamDistance->value(),
                m_uvMaximumAcceleration->value(), m_uvMaximumJerk->value(),
                m_uvOvertravelLength->value(), m_uvArcSpeedLoss->value(),
                m_uvShowSplitView->isChecked());
        };
        for(QDoubleSpinBox* spinBox : { m_uvTargetSpacing, m_uvTrajectorySpeed, m_uvSprayDistance,
                m_uvMaximumSeamDistance, m_uvMaximumAcceleration, m_uvMaximumJerk,
                m_uvOvertravelLength, m_uvArcSpeedLoss }) {
            connect(spinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [publishUv](double) { publishUv(); });
        }
        connect(m_uvShowSplitView, &QCheckBox::toggled, this,
            [publishUv](bool) { publishUv(); });

        const auto publishTcp = [this, fanucX, fanucY, fanucZ]() {
            FreeformTrajPlanTcpSettings settings;
            settings.enabled = m_tcpEnabled->isChecked();
            settings.format = m_tcpFormat->currentIndex() == 0
                ? FreeformTrajPlanTcpFormat::Abb
                : FreeformTrajPlanTcpFormat::Fanuc;
            if(settings.format == FreeformTrajPlanTcpFormat::Abb) {
                settings.x = m_tcpX->value();
                settings.y = m_tcpY->value();
                settings.z = m_tcpZ->value();
                settings.q1 = m_tcpQ1->value();
                settings.q2 = m_tcpQ2->value();
                settings.q3 = m_tcpQ3->value();
                settings.q4 = m_tcpQ4->value();
            } else {
                settings.x = fanucX->value();
                settings.y = fanucY->value();
                settings.z = fanucZ->value();
                settings.w = m_tcpW->value();
                settings.p = m_tcpP->value();
                settings.r = m_tcpR->value();
            }
            emit tcpSettingsChanged(settings);
        };
        connect(m_tcpEnabled, &QCheckBox::toggled, this, [this, publishTcp](bool enabled) {
            m_tcpFormat->setEnabled(enabled);
            m_tcpInputStack->setEnabled(enabled);
            publishTcp();
        });
        connect(m_tcpFormat, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this, publishTcp](int index) {
                m_tcpInputStack->setCurrentIndex(index);
                publishTcp();
            });
        for(QDoubleSpinBox* spinBox : { m_tcpX, m_tcpY, m_tcpZ, m_tcpQ1, m_tcpQ2, m_tcpQ3, m_tcpQ4,
                fanucX, fanucY, fanucZ, m_tcpW, m_tcpP, m_tcpR }) {
            connect(spinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [publishTcp](double) { publishTcp(); });
        }
    }

    void FreeformTrajPlanParameterPanel::setPlanningMethod(
        spray_freeform_traj_plan::PlanningMethod method)
    {
        m_parameterStack->setCurrentIndex(
            method == spray_freeform_traj_plan::PlanningMethod::Uv ? 1 : 0);
    }
}
