cmake_minimum_required(VERSION 3.20)
add_executable(FTTrajConnPreviewTest main.cpp)
target_link_libraries(FTTrajConnPreviewTest PRIVATE
    SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
    SMRobotApps::RobotViewerCore
    Qt5::Widgets)
target_compile_features(FTTrajConnPreviewTest PRIVATE cxx_std_17)
set_target_properties(FTTrajConnPreviewTest PROPERTIES FOLDER tests/SMRobotWorkbenchFreeformTrajPlan)
add_test(NAME FreeformTrajPlanTrajectoryConnectionPreviewTest COMMAND $<TARGET_FILE:FTTrajConnPreviewTest> --verify)
set_tests_properties(FreeformTrajPlanTrajectoryConnectionPreviewTest PROPERTIES LABELS "SMRobotWorkbenchFreeformTrajPlan;FreeformTrajPlanRender;Trajectory;Connection;Line")
