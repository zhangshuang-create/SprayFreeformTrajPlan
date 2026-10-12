cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotSpray::SprayFreeformTrajPlan
    SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
    SMRobotWorkbenchCommon::RobotQtModulesShared
    Qt5::Widgets
)
set(${TARGET_NAME}_RequiredLibsPrivate)
