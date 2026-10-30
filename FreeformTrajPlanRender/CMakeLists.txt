cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME FreeformTrajPlanRender)

file(GLOB RENDER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)
file(GLOB RENDER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hxx"
)

add_library(${TARGET_NAME} SHARED ${RENDER_SOURCES} ${RENDER_HEADERS})
add_library(SMRobotWorkbenchFreeformTrajPlan::${TARGET_NAME} ALIAS ${TARGET_NAME})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES
    ${RENDER_SOURCES}
    ${RENDER_HEADERS}
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
)
target_link_libraries(${TARGET_NAME}
    PUBLIC
        SMRobotSpray::SprayFreeformTrajPlan
)

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchFreeformTrajPlan
    WINDOWS_EXPORT_ALL_SYMBOLS TRUE
    DEBUG_POSTFIX _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
)

if(BUILD_UV_TRAJ_PLAN)
    target_link_libraries(${TARGET_NAME} PUBLIC UVTrajPlan::TrajModule)
endif()
target_compile_definitions(${TARGET_NAME} PUBLIC BUILD_UV_TRAJ_PLAN=$<BOOL:${BUILD_UV_TRAJ_PLAN}>)
