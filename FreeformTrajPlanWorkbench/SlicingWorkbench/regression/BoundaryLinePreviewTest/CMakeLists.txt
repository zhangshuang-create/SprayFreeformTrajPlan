cmake_minimum_required(VERSION 3.20)
add_executable(FTBoundaryLinePreviewTest main.cpp)
target_link_libraries(FTBoundaryLinePreviewTest PRIVATE
    SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
    SMRobotApps::RobotViewerCore
    Qt5::Widgets)
target_compile_features(FTBoundaryLinePreviewTest PRIVATE cxx_std_17)
set_target_properties(FTBoundaryLinePreviewTest PROPERTIES FOLDER tests/SMRobotWorkbenchFreeformTrajPlan)
add_test(NAME FreeformTrajPlanBoundaryLinePreviewTest COMMAND $<TARGET_FILE:FTBoundaryLinePreviewTest> --verify)
set_tests_properties(FreeformTrajPlanBoundaryLinePreviewTest PROPERTIES LABELS "SMRobotWorkbenchFreeformTrajPlan;FreeformTrajPlanRender;Boundary;Line")
