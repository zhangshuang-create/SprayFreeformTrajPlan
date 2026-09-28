##########################################################################
#	CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

#   Set the requirement for current target
set( ${TARGET_NAME}_RequiredLibsPublic
)

set( ${TARGET_NAME}_RequiredLibsPrivate
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Eigen3::Eigen
    Guimodule::PropertyPanel
    ATrajPlan::BurTrajPlan
    ATrajPlan::GeneralData
    ATrajPlan::RenderData
    ATrajPlan::TrajOptimization
    ATrajPlan::TrajOptimizationData
    PublicData::ModelData
    PublicData::TrajData
    PublicData::PublicDataRender
    ThicknessPrediction::ThicknessAnalyzer
    ThicknessPrediction::GaussianModel
    TrajectoryInterpolation::LinearInterpolator
    UVTrajPlan::TrajModule
)
