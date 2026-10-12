cmake_minimum_required(VERSION 3.20)

add_executable(FTModelPointLineSurfaceTest main.cpp)
target_link_libraries(FTModelPointLineSurfaceTest
    PRIVATE
        SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
        SMRobotApps::RobotViewerCore
        Qt5::Widgets)
target_compile_features(FTModelPointLineSurfaceTest PRIVATE cxx_std_17)
set_target_properties(FTModelPointLineSurfaceTest PROPERTIES
    FOLDER tests/SMRobotWorkbenchFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanModelPointLineSurfaceTest
    COMMAND $<TARGET_FILE:FTModelPointLineSurfaceTest> --verify)
set_tests_properties(FreeformTrajPlanModelPointLineSurfaceTest PROPERTIES
    LABELS "SMRobotWorkbenchFreeformTrajPlan;FreeformTrajPlanRender;Model;Point;Line;Surface")
