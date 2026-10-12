cmake_minimum_required(VERSION 3.20)
add_executable(FTModelSurfacePreviewTest main.cpp)
target_link_libraries(FTModelSurfacePreviewTest PRIVATE
    SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
    SMRobotApps::RobotViewerCore
    Qt5::Widgets)
target_compile_features(FTModelSurfacePreviewTest PRIVATE cxx_std_17)
set_target_properties(FTModelSurfacePreviewTest PROPERTIES FOLDER tests/SMRobotWorkbenchFreeformTrajPlan)
add_test(NAME FreeformTrajPlanModelSurfacePreviewTest COMMAND $<TARGET_FILE:FTModelSurfacePreviewTest> --verify)
set_tests_properties(FreeformTrajPlanModelSurfacePreviewTest PROPERTIES LABELS "SMRobotWorkbenchFreeformTrajPlan;FreeformTrajPlanRender;Model;Surface")
