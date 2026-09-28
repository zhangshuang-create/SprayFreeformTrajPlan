##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

##########################################################################
#	${PACKAGE_NAME} Configuration
##########################################################################
#   Get directory name as package name
message( STATUS "!!!!!!Package ${PACKAGE_NAME} configuration is set here!!!!!!" )


##########################################################################
#   Package ${PACKAGE_NAME} components definition
##########################################################################
set( ExcludeTargets )
collect_subdirs(
    ${CMAKE_CURRENT_LIST_DIR}
    AllTargetNames
    ONLY_CMAKE_PROJECTS
    EXCLUDE ${ExcludeTargets}
)

set( ${PACKAGE_NAME}_Components ${AllTargetNames} )
set( ${PACKAGE_NAME}_ExportComponents ${${PACKAGE_NAME}_Components} )
message( STATUS "==================================================================")
message( STATUS "All Targets in Package ${PACKAGE_NAME}: ${${PACKAGE_NAME}_Components}" )
message( STATUS "==================================================================\n")


##########################################################################
#	Package ${PACKAGE_NAME} required targets
##########################################################################

#   2. Collect required libs for each component (optional).
foreach( TARGET_NAME ${${PACKAGE_NAME}_Components} )
    set( ${TARGET_NAME}_RequiredLibsPublic "" )
    set( ${TARGET_NAME}_RequiredLibsPrivate "" )

    set( _target_config_file "${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}/TargetConfigSetting.cmake" )
    if( EXISTS "${_target_config_file}" )
        include( "${_target_config_file}" )
    endif()

    set( ${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic ${${TARGET_NAME}_RequiredLibsPublic} )
    set( ${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate ${${TARGET_NAME}_RequiredLibsPrivate} )
endforeach()



#   3. Remove duplicated libs and catolog the targets
set( ${PACKAGE_NAME}_RequiredLibsPublic "" )
set( ${PACKAGE_NAME}_RequiredLibsPrivate "" )
set( ${PACKAGE_NAME}_RequiredLibs "" )




