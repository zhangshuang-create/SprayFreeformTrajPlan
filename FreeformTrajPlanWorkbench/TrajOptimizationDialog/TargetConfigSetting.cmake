##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

#   Set the requirement for current target
set( ${TARGET_NAME}_RequiredLibsPublic


              Eigen3::Eigen
         Common::CustomLog

         Boost::headers 
         Boost::filesystem 
         Boost::log 
         Boost::log_setup 
         Boost::date_time 

         Qt5::Widgets 
         Qt5::Core 
         Qt5::Gui 
         Qt5::OpenGL 

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
         ATrajPlan::STLProcessor 
         ATrajPlan::BoundaryDetector 
         ATrajPlan::UniformSlicer 
        # Trajectory optimization module dependencies
         ATrajPlan::TrajOptimization
         ATrajPlan::TrajOptimizationData
         ATrajPlan::CollisionDetection
        PublicData::ModelData
         PublicData::OptSafeTrajData
)

set( ${TARGET_NAME}_RequiredLibsPrivate
    

    
)

