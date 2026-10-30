#pragma once

#include <SlicingTrajPlan/FreeformTrajPlanTypes.h>
#include <SlicingTrajPlan/GeneralData.h>
#if BUILD_UV_TRAJ_PLAN
#include <UVTrajPlan/CommonDataTypes.h>
#endif

#include "FreeformTrajPlanRenderData.h"

namespace smrobot::workbench::freeform
{
    class FreeformTrajPlanRenderer final
    {
    public:
        FreeformTrajPlanMeshPreview makeModelPreview(
            const spray_freeform_traj_plan::FreeformSurfaceMesh& mesh) const;
        FreeformTrajPlanLinePreview makeBoundaryPreview(
            const spray_freeform_traj_plan::FreeformBoundaryResult& boundaries) const;
        FreeformTrajPlanPreview makePreview(
            const spray_freeform_traj_plan::FreeformTrajectory& trajectory) const;
        FreeformTrajPlanLinePreview makeTrajectoryFramePreview(
            const spray_freeform_traj_plan::FreeformTrajectory& trajectory,
            double axisLengthMeters = 0.03) const;
        FreeformTrajPlanLinePreview makeSlicingPreview(
            const spray_freeform_traj_plan::FreeformSlicingResult& slicing) const;
        FreeformTrajPlanLinePreview makeSlicingPlanePreview(
            const spray_freeform_traj_plan::FreeformSlicingResult& slicing) const;
        FreeformTrajPlanLinePreview makeConnectionPreview(
            const std::vector<spray_freeform_traj_plan::FreeformConnectionSegment>& connections) const;

        // These adapters are deliberately in the Workbench renderer. GeneralData
        // remains the algorithm document and never owns a view representation.
        FreeformTrajPlanLinePreview makeSlicingPreview(
            const ModelTrajPlan::GeneralData& document) const;
        FreeformTrajPlanMeshPreview makeSlicingSurfacePreview(
            const ModelTrajPlan::GeneralData& document) const;
        FreeformTrajPlanPreview makePreview(
            const std::vector<ModelTrajPlan::TrajectoryPoint>& trajectory) const;
        FreeformTrajPlanLinePreview makeTrajectoryFramePreview(
            const std::vector<ModelTrajPlan::TrajectoryPoint>& trajectory,
            double axisLength = 30.0) const;
        FreeformTrajPlanLinePreview makeConnectionPreview(
            const std::vector<ModelTrajPlan::ConnectionSegment>& connections) const;
        FreeformTrajPlanLinePreview makeReferenceLinePreview(
            const ModelTrajPlan::GeneralData& document) const;
#if BUILD_UV_TRAJ_PLAN
        FreeformTrajPlanLinePreview makeUvTrajectoryPreview(
            const std::vector<SprayTrajPlan::TrajectorySegment>& trajectory) const;
#endif

    };
}
