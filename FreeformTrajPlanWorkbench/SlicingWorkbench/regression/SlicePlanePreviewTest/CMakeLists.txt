cmake_minimum_required(VERSION 3.20)
add_executable(FTSlicePlanePreviewTest main.cpp)
target_link_libraries(FTSlicePlanePreviewTest PRIVATE
    SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
    SMRobotApps::RobotViewerCore
    Qt5::Widgets)
target_compile_features(FTSlicePlanePreviewTest PRIVATE cxx_std_17)
set_target_properties(FTSlicePlanePreviewTest PROPERTIES FOLDER tests/SMRobotWorkbenchFreeformTrajPlan)
add_test(NAME FreeformTrajPlanSlicePlanePreviewTest COMMAND $<TARGET_FILE:FTSlicePlanePreviewTest> --verify)
set_tests_properties(FreeformTrajPlanSlicePlanePreviewTest PROPERTIES LABELS "SMRobotWorkbenchFreeformTrajPlan;FreeformTrajPlanRender;SlicePlane")
