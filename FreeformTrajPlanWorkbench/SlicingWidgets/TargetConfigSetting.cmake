##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

#   Set the requirement for current target
set( ${TARGET_NAME}_RequiredLibsPublic
)

set( ${TARGET_NAME}_RequiredLibsPrivate
    
        Eigen3::Eigen
        Boost::headers 
        Boost::filesystem 
        Boost::log 
        Boost::log_setup 
        Boost::date_time 

        Qt5::Widgets 
        Qt5::Core 
        Qt5::Gui 
        Qt5::OpenGL 
        Common::CustomLog
        QtComponents::QtOpenGLView 
        QtComponents::QtGLGMMenu 
        QtComponents::QtRobotControlMenu 
        QtComponents::QtRobotTreeView 
        QtComponents::QtAssemblyTreeView 
        SMRobotGen1::BasicSystemTangQing
        SMRobotGen1::CameraTangQing
        SMRobotGen1::ModelTangQing
        SMRobotGen1::SMRobotTangQing
                # ATrajPlan module dependencies
        ATrajPlan::BurTrajPlan 
        ATrajPlan::GeneralData
        ATrajPlan::RenderData
        PublicData::OptSafeTrajData
        PublicData::ModelData
        PublicData::TrajData
        PublicData::PublicDataRender
        ATrajPlan::STLProcessor 
        ATrajPlan::BoundaryDetector 
        ATrajPlan::UniformSlicer 
        ATrajPlan::TrajOptimizationData
        Guimodule::TrajOptimizationDialog 
        ATrajPlan::TrajOptimization
    
)

