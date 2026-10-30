cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME FreeformTrajPlanWorkbench)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB WORKBENCH_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.cxx"
)
file(GLOB WORKBENCH_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench/*.hxx"
)

add_library(${TARGET_NAME} SHARED ${WORKBENCH_SOURCES} ${WORKBENCH_HEADERS})
add_library(SMRobotWorkbenchFreeformTrajPlan::${TARGET_NAME} ALIAS ${TARGET_NAME})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench" FILES
    ${WORKBENCH_SOURCES}
    ${WORKBENCH_HEADERS}
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${TARGET_NAME} PRIVATE FREEFORM_TRAJ_PLAN_WORKBENCH_BUILD)
target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SlicingWorkbench>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
)
target_link_libraries(${TARGET_NAME}
    PUBLIC
        SMRobotSpray::SprayFreeformTrajPlan
        SMRobotSpray::SprayTrajectoryCore
        SMRobotWorkbenchFreeformTrajPlan::FreeformTrajPlanRender
        SMRobotWorkbenchCommon::RobotQtModulesShared
        Qt5::Widgets
)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchFreeformTrajPlan
    AUTOMOC ON
    AUTOUIC ON
    DEBUG_POSTFIX _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
    SMROBOT_WORKBENCH_REGISTRAR_HEADER "SlicingWorkbench/FreeformTrajPlanWorkbenchContribution.h"
    SMROBOT_WORKBENCH_REGISTRAR_FUNCTION "robot_qt_viewer::registerFreeformTrajPlanWorkbenchContribution"
    EXPORT_PROPERTIES "SMROBOT_WORKBENCH_REGISTRAR_HEADER;SMROBOT_WORKBENCH_REGISTRAR_FUNCTION"
)

if(BUILD_TESTING)
    add_subdirectory(SlicingWorkbench/regression)
endif()

if(BUILD_UV_TRAJ_PLAN)
    target_link_libraries(${TARGET_NAME} PUBLIC UVTrajPlan::TrajModule)
endif()
target_compile_definitions(${TARGET_NAME} PUBLIC BUILD_UV_TRAJ_PLAN=$<BOOL:${BUILD_UV_TRAJ_PLAN}>)
