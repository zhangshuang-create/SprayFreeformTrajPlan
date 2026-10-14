#pragma once

#include <QWidget>
#include "FreeformTrajPlanWorkbenchExport.h"
#include <SlicingTrajPlan/FreeformTrajPlanTypes.h>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QLineEdit;
class QStackedWidget;

namespace smrobot::workbench::freeform
{
    enum class FreeformTrajPlanExportFormat
    {
        Paq,
        Fanuc,
        Abb,
        Matrix
    };

    enum class FreeformTrajPlanTcpFormat
    {
        Abb,
        Fanuc
    };

    struct FreeformTrajPlanTcpSettings
    {
        bool enabled = false;
        FreeformTrajPlanTcpFormat format = FreeformTrajPlanTcpFormat::Abb;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
        double p = 0.0;
        double r = 0.0;
        double q1 = 1.0;
        double q2 = 0.0;
        double q3 = 0.0;
        double q4 = 0.0;
    };

    class FREEFORM_TRAJ_PLAN_WORKBENCH_API FreeformTrajPlanWorkflowPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit FreeformTrajPlanWorkflowPanel(QWidget* parent = nullptr);
        void setModelPath(const QString& path);
        void setStatus(const QString& message, bool success);
        void setPlanningMethod(spray_freeform_traj_plan::PlanningMethod method);

    signals:
        void importModelRequested(const QString& path);
        void planningMethodRequested(bool uv);
        void slicingMethodRequested(bool adaptive);
        void exportTrajectoryRequested(const QString& path, FreeformTrajPlanExportFormat format, double interpolationInterval);
        void workflowActionRequested(spray_freeform_traj_plan::WorkflowAction action);

    private:
        QLineEdit* m_modelPath = nullptr;
        QLabel* m_status = nullptr;
        QStackedWidget* m_workflowStack = nullptr;
    };

    class FREEFORM_TRAJ_PLAN_WORKBENCH_API FreeformTrajPlanParameterPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit FreeformTrajPlanParameterPanel(QWidget* parent = nullptr);
        void setPlanningMethod(spray_freeform_traj_plan::PlanningMethod method);

    signals:
        void parametersChanged(double trajectorySpacingMm, double chordErrorTolerance,
            double minimumSpacingMm, double largeEndOffsetMm, double smallEndOffsetMm,
            double optimalSprayDistanceMm, double optimalSpeedMmPerSecond,
            int trajectoryCount, int maxSliceCount);
        void uvParametersChanged(double targetSpacingMm, double trajectorySpeedMmPerSecond,
            double sprayDistanceMm, double maximumSeamSprayDistanceMm,
            double maximumAccelerationMmPerSecondSquared,
            double maximumJerkMmPerSecondCubed, double overtravelLength,
            double arcSpeedLossPercent, bool showUvSplitView);
        void visualizationVisibilityChanged(
            bool modelVisible, bool slicesVisible, bool trajectoriesVisible, bool connectionsVisible,
            bool referenceLinesVisible);
        void tcpSettingsChanged(const FreeformTrajPlanTcpSettings& settings);
        void workflowActionRequested(spray_freeform_traj_plan::WorkflowAction action);

    private:
        QDoubleSpinBox* m_trajectorySpacing = nullptr;
        QDoubleSpinBox* m_chordError = nullptr;
        QDoubleSpinBox* m_minimumSpacing = nullptr;
        QDoubleSpinBox* m_largeEndOffset = nullptr;
        QDoubleSpinBox* m_smallEndOffset = nullptr;
        QDoubleSpinBox* m_sprayDistance = nullptr;
        QDoubleSpinBox* m_speed = nullptr;
        QSpinBox* m_trajectoryCount = nullptr;
        QSpinBox* m_maxSliceCount = nullptr;
        QStackedWidget* m_parameterStack = nullptr;
        QDoubleSpinBox* m_uvTargetSpacing = nullptr;
        QDoubleSpinBox* m_uvTrajectorySpeed = nullptr;
        QDoubleSpinBox* m_uvSprayDistance = nullptr;
        QDoubleSpinBox* m_uvMaximumSeamDistance = nullptr;
        QDoubleSpinBox* m_uvMaximumAcceleration = nullptr;
        QDoubleSpinBox* m_uvMaximumJerk = nullptr;
        QDoubleSpinBox* m_uvOvertravelLength = nullptr;
        QDoubleSpinBox* m_uvArcSpeedLoss = nullptr;
        QCheckBox* m_uvShowSplitView = nullptr;
        QCheckBox* m_tcpEnabled = nullptr;
        QComboBox* m_tcpFormat = nullptr;
        QStackedWidget* m_tcpInputStack = nullptr;
        QDoubleSpinBox* m_tcpX = nullptr;
        QDoubleSpinBox* m_tcpY = nullptr;
        QDoubleSpinBox* m_tcpZ = nullptr;
        QDoubleSpinBox* m_tcpW = nullptr;
        QDoubleSpinBox* m_tcpP = nullptr;
        QDoubleSpinBox* m_tcpR = nullptr;
        QDoubleSpinBox* m_tcpQ1 = nullptr;
        QDoubleSpinBox* m_tcpQ2 = nullptr;
        QDoubleSpinBox* m_tcpQ3 = nullptr;
        QDoubleSpinBox* m_tcpQ4 = nullptr;
    };
}
