#include "FreeformTrajPlanRenderer.h"

#include <iostream>

int main()
{
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    ModelTrajPlan::GeneralData document;
    document.slicingData_.currentSliceResult_.isBoundarylessFallback = true;
    ModelTrajPlan::CuttingPlane plane;
    plane.vertices = { Eigen::Vector3d(-1.0, -1.0, 2.0), Eigen::Vector3d(1.0, -1.0, 2.0),
        Eigen::Vector3d(1.0, 1.0, 2.0), Eigen::Vector3d(-1.0, 1.0, 2.0) };
    document.slicingData_.cuttingPlanes_.push_back(plane);
    const auto preview = renderer.makeSlicingSurfacePreview(document);
    if(preview.vertices.size() != 4 || preview.indices.size() != 6 ||
        preview.vertices.front().alpha < 0.95 || preview.vertices.front().red > 0.20 ||
        preview.vertices.front().green < 0.85) {
        std::cerr << "Boundaryless slice surface is not high contrast.\n";
        return 1;
    }
    return 0;
}
