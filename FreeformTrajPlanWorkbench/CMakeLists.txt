##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

##########################################################################
#	Package ${PACKAGE_NAME} 
##########################################################################
get_filename_component( PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
set( PKG_MSG_HEADER "[PKG-${PACKAGE_NAME}]:" )
set( GROUP_NAME ${PACKAGE_NAME} )
#get_filename_component( _PARENT_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY )
#get_filename_component( PARENT_PACKAGE_NAME ${_PARENT_PACKAGE_DIR} NAME )

# Component list (PCTrajWidgets, PlanDialog, ...) from collect_subdirs(src)
include( ${CMAKE_CURRENT_SOURCE_DIR}/PackageConfigSetting.cmake )

#   Display all the components of this package
message( STATUS "${PKG_MSG_HEADER} Package ${PACKAGE_NAME}'s Components include: ${${PACKAGE_NAME}_Components}" )
foreach( comp_ ${${PACKAGE_NAME}_Components} )
    message( STATUS "  ----  ${comp_}" )
endforeach()
message( STATUS " " )

#   Display all the required libs for this package
message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}_RequiredLibsPublic: ${${PACKAGE_NAME}_RequiredLibsPublic}" )
message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}_RequiredLibsPrivate: ${${PACKAGE_NAME}_RequiredLibsPrivate}" )

#   Display all the required libs for each components of this package
foreach( comp_ ${${PACKAGE_NAME}_Components} )
    message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}${comp_}_RequiredLibsPublic: ${${PACKAGE_NAME}${comp_}_RequiredLibsPublic}" )
    message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}${comp_}_RequiredLibsPrivate: ${${PACKAGE_NAME}${comp_}_RequiredLibsPrivate}" )
endforeach()


# ##########################################################################
# #	Set all export option for each components of package ${PACKAGE_NAME}
# ##########################################################################
# #   Set the export option for each target
# option( ExportPacakge${PACKAGE_NAME} "Export package -- ${PACKAGE_NAME}" ON )
# foreach( export_comp_ ${${PACKAGE_NAME}_ExportComponents} )
#     cmake_dependent_option( ExportPacakge${PACKAGE_NAME}_${export_comp_} "${PACKAGE_NAME} : export ${export_comp_}" ON "ExportPacakge${PACKAGE_NAME}" OFF )
# endforeach()


##########################################################################
#	Build ${PACKAGE_NAME}
##########################################################################
message( STATUS "Build${PARENT_PACKAGE_NAME}_${PACKAGE_NAME} is ${Build${PARENT_PACKAGE_NAME}_${PACKAGE_NAME}} in ${PACKAGE_NAME} directory." )

if( UsingPrebuilt_${PACKAGE_NAME} )
    # #   Using Prebuilt
    # message( STATUS "Finding Prebuilt Package ${PACKAGE_NAME} with components ${${PACKAGE_NAME}_PrebuiltComponents}" )
    # find_prebuilt_package( 
    #     PACKAGE_NAME ${PACKAGE_NAME}
    #     COMPONENTS ${${PACKAGE_NAME}_PrebuiltComponents}
    # )
else()
    #   Build from source
    message( STATUS "Build package ${PACKAGE_NAME} on site" )
    set( CMAKE_AUTOMOC ON )
    set( CMAKE_AUTORCC ON )
    set( CMAKE_AUTOUIC ON )

    foreach( each_comp_ ${${PACKAGE_NAME}_Components} )
        cmake_dependent_option( Build${PACKAGE_NAME}_${each_comp_} "Build ${PACKAGE_NAME}::${each_comp_}" ON "Build${PARENT_PACKAGE_NAME}_${PACKAGE_NAME}" OFF )
        if( Build${PACKAGE_NAME}_${each_comp_} )
            add_subdirectory( ${each_comp_} )
            if( TARGET ${each_comp_} )
                set_target_properties( ${each_comp_} PROPERTIES FOLDER "${PARENT_PACKAGE_NAME}/${PACKAGE_NAME}" )
            endif()
        else()
            message( STATUS "Build${PACKAGE_NAME}_${each_comp_} is ${Build${PACKAGE_NAME}_${each_comp_}} in ${PACKAGE_NAME} directory." )
        endif()
    endforeach()
endif()



# #############################################################
# #	Build ${PACKAGE_NAME} Example
# #############################################################
# if( BuildExample_${PACKAGE_NAME} )
#     foreach( each_comp_ ${${PACKAGE_NAME}_Components} )
#         set( TARGET_NAME ${each_comp_} )
#         add_subdirectory( ${each_comp_}/example )
#     endforeach()
# endif()



# ##########################################################################
# #   Generate package
# ##########################################################################
# if( ${ExportPacakge${PACKAGE_NAME}} AND NOT UsingPrebuilt_${PACKAGE_NAME} )
#     include(CMakePackageConfigHelpers)
#     set(SUPPORTED_COMPONENTS "${${PACKAGE_NAME}_ExportComponents}")

#     configure_package_config_file(
#         ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
#         ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
#         INSTALL_DESTINATION ${PACKAGE_NAME}/lib/cmake
#     )

#     write_basic_package_version_file(
#         ${PACKAGE_NAME}ConfigVersion.cmake
#         VERSION ${PROJECT_VERSION}
#         COMPATIBILITY AnyNewerVersion
#     )

#     install(FILES
#         ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
#         ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
#         DESTINATION ${PACKAGE_NAME}/lib/cmake
#     )
# endif()



