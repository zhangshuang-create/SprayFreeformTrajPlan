##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

#   Set the requirement for current target
set( ${TARGET_NAME}_RequiredLibsPublic
)

set( ${TARGET_NAME}_RequiredLibsPrivate


    ATrajPlan::BurTrajPlan
    ATrajPlan::GeneralData
    ATrajPlan::RenderData
    ATrajPlan::TrajOptimizationData
    Guimodule::SlicingWidgets
    PCL::common
    PCL::io
    Common::CustomLog
    Qt5::Widgets
    Common::GLRuntime
    QtComponents::QtOpenGLView
    QtComponents::QtGLGMMenu
    QtComponents::QtRobotControlMenu
    QtComponents::QtAssemblyTreeView
    QtComponents::QtRobotTreeView
    assimp::assimp
    SMRobotGen1::BasicSystemTangQing
    SMRobotGen1::CameraTangQing
    SMRobotGen1::ModelTangQing
    SMRobotGen1::SMRobotTangQing
    PublicData::OptSafeTrajData
    PublicData::ModelData
    PublicData::PublicDataRender
    PublicData::TrajData
    Boost::locale
    Boost::filesystem
    Boost::system
    Boost::log
    Boost::log_setup

)

