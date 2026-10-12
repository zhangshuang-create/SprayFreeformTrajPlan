#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"
#include <cmath>
#include <iostream>
int main(int argc, char** argv)
{
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    ModelTrajPlan::GeneralData document;
    ModelTrajPlan::CuttingPlane plane;
    plane.isSelected = true;
    plane.normal = Eigen::Vector3d::UnitZ();
    plane.center = Eigen::Vector3d(20.0, 40.0, 30.0);
    plane.translation_z = 15.0;
    plane.rotation_x = 90.0;
    plane.vertices = { Eigen::Vector3d(10.0, 20.0, 30.0), Eigen::Vector3d(50.0, 20.0, 30.0),
        Eigen::Vector3d(50.0, 60.0, 30.0), Eigen::Vector3d(10.0, 60.0, 30.0) };
    document.slicingData_.cuttingPlanes_.push_back(plane);
    ModelTrajPlan::CuttingPlane otherPlane = plane;
    otherPlane.isSelected = false;
    otherPlane.vertices[0].z() = 10.0;
    otherPlane.vertices[1].z() = 10.0;
    otherPlane.vertices[2].z() = 10.0;
    otherPlane.vertices[3].z() = 10.0;
    document.slicingData_.cuttingPlanes_.push_back(otherPlane);

    const auto surface = renderer.makeSlicingSurfacePreview(document);
    if(surface.vertices.size() != 8 || surface.indices.size() != 12 ||
        // The first vertex is rotated around the plane center, then translated
        // along Z: (10, 20, 30) -> (10, 40, 25).
        std::abs(surface.vertices.front().positionX - 10.0) > 1.0e-12 ||
        std::abs(surface.vertices.front().positionY - 40.0) > 1.0e-12 ||
        std::abs(surface.vertices.front().positionZ - 25.0) > 1.0e-12 ||
        surface.indices[5] != 3 || std::abs(surface.vertices.front().alpha - 1.0) > 1.0e-12 ||
        std::abs(surface.vertices[4].alpha - 1.0) > 1.0e-12) {
        std::cerr << "Algorithm cutting-plane surface conversion failed.\n";
        return 1;
    }
    if(surface.vertices.front().red < 0.99 || surface.vertices.front().green < 0.85 ||
        surface.vertices[4].red > 0.70 || surface.vertices[4].green > 0.90) {
        std::cerr << "Selected-layer color is not unique.\n";
        return 2;
    }
    return 0;
}
