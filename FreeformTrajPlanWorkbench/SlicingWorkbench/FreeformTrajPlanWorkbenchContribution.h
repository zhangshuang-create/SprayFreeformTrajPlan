#pragma once

#include "FreeformTrajPlanWorkbenchExport.h"
#include "RobotQtViewerWorkbenchPackageRegistry.h"
#include "RobotQtViewerWorkbenchContribution.h"

#include <functional>
#include <QWidget>
#include <QString>

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;

    inline constexpr const char* kFreeformTrajPlanWorkbenchId = "smrobot.mode.freeform-trajectory-planning";

    FREEFORM_TRAJ_PLAN_WORKBENCH_API
    bool registerFreeformTrajPlanWorkbenchContribution(RobotQtViewerWorkbenchPackageRegistry& catalog);

    struct FreeformTrajPlanWorkbenchComposition
    {
        //! 观察指针，生命周期由外部维护，本结构体不拥有对象
        RobotQtViewerDocumentContext* documentContext = nullptr;

        std::function<void(QWidget* workflowPanel)> installLeftPanel;
        std::function<void(const QString& message, int timeoutMs)> showStatus;
    };

    FREEFORM_TRAJ_PLAN_WORKBENCH_API
    robot_qt_viewer::RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
    makeOwnedFreeformTrajPlanWorkbenchRuntimeContributionFactory(FreeformTrajPlanWorkbenchComposition&& composition);
}
