#pragma once

#include <QObject>
#include "FreeformTrajPlanWorkbenchExport.h"
#include "PlanningDataSource.h"
#include "FreeformTrajPlanWorkbenchPanel.h"
#include "FreeformTrajPlanRenderer.h"
#include <SlicingTrajPlan/BurTrajPlan.h>
#include <SlicingTrajPlan/GeneralData.h>
#include <UVTrajPlan/TrajPlanPipeline.h>

#include <functional>

namespace robot_qt_viewer { class RobotQtViewerDocumentContext; struct RobotQtViewerEvent; }
namespace smrobot::workbench::freeform
{
    using FreeformTrajPlanPreviewUpdatedCallback =
        std::function<void(const FreeformTrajPlanPreview& preview)>;
    using FreeformTrajPlanPreviewClearedCallback = std::function<void()>;

    class FREEFORM_TRAJ_PLAN_WORKBENCH_API FreeformTrajPlanWorkbenchController final : public QObject
    {
        Q_OBJECT
    public:
        FreeformTrajPlanWorkbenchController(FreeformTrajPlanWorkflowPanel& workflowPanel,
            FreeformTrajPlanParameterPanel& parameterPanel,
            robot_qt_viewer::RobotQtViewerDocumentContext& context,
            robot_qt_viewer::PlanningDataSource* dataSource,
            FreeformTrajPlanPreviewUpdatedCallback previewUpdated,
            FreeformTrajPlanPreviewClearedCallback previewCleared,
            QObject* parent = nullptr);
        void handleEvent(const robot_qt_viewer::RobotQtViewerEvent& event);
    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
    private:
        void importModel(const QString& path);
        void exportTrajectory(const QString& path, FreeformTrajPlanExportFormat format, double interpolationInterval);
        void setPlanningMethod(bool uv);
        void setSlicingMethod(bool adaptive);
        void setParameters(double trajectorySpacingMm, double chordErrorTolerance,
            double minimumSpacingMm, double largeEndOffsetMm, double smallEndOffsetMm,
            double optimalSprayDistanceMm, double optimalSpeedMmPerSecond,
            int trajectoryCount, int maxSliceCount);
        void setUvParameters(double targetSpacingMm, double trajectorySpeedMmPerSecond,
            double sprayDistanceMm, double maximumSeamSprayDistanceMm,
            double maximumAccelerationMmPerSecondSquared,
            double maximumJerkMmPerSecondCubed, double overtravelLength,
            double arcSpeedLossPercent, bool showUvSplitView);
        void setVisualizationVisibility(
            bool modelVisible, bool slicesVisible, bool trajectoriesVisible, bool connectionsVisible,
            bool referenceLinesVisible);
        void setTcpSettings(const FreeformTrajPlanTcpSettings& settings);
        void requestWorkflowAction(spray_freeform_traj_plan::WorkflowAction action);
        void showSlicingAdjustDialog();
        void showTrajectoryTransformDialog();
        void showTrajectoryFrameDialog();
        bool applyTrajectoryTransform(const Eigen::Matrix4d& transform,
            bool transformModel, bool transformTrajectory, QString& errorMessage);
        void refreshSlicingOverlays();
        void showResult(bool success, const QString& message);
        SprayTrajPlan::TrajectoryParams uvTrajectoryParameters() const;
        void refreshUvTrajectoryOverlay();
        void publishSharedData();
        FreeformTrajPlanWorkflowPanel& m_workflowPanel;
        FreeformTrajPlanParameterPanel& m_parameterPanel;
        robot_qt_viewer::RobotQtViewerDocumentContext& m_context;
        robot_qt_viewer::PlanningDataSource* m_dataSource = nullptr;
        FreeformTrajPlanRenderer m_renderer;
        FreeformTrajPlanPreviewUpdatedCallback m_previewUpdated;
        FreeformTrajPlanPreviewClearedCallback m_previewCleared;
        // Persistent planning document: the original algorithm data model and
        // its algorithm facade. The Workbench only converts it for display.
        ModelTrajPlan::GeneralData m_generalData;
        ModelTrajPlan::BurTrajPlan m_plan;
        bool m_uvPlanning = false;
        QString m_modelPath;
        SprayTrajPlan::TrajectoryParams m_uvParameters;
        SprayTrajPlan::TrajPlanPipeline m_uvPipeline;
        QString m_modelObjectId;
        // The original planning algorithms keep the STL's native coordinates
        // (normally millimetres).  The scene can display that same STL in metres.
        // This factor is applied only when converting algorithm output to a view mesh.
        double m_algorithmToSceneScale = 1.0;
        bool m_modelVisible = true;
        bool m_slicesVisible = true;
        bool m_trajectoriesVisible = true;
        bool m_connectionsVisible = true;
        bool m_referenceLinesVisible = true;
        FreeformTrajPlanTcpSettings m_tcpSettings;
    };
}
