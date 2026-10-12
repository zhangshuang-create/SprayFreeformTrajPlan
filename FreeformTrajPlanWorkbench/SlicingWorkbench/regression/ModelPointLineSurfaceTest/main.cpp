#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"

#include <cmath>
#include <iostream>

namespace
{
    bool nearlyEqual(double left, double right, double tolerance = 1.0e-12)
    {
        return std::abs(left - right) <= tolerance;
    }
}

int main(int argc, char** argv)
{
    using namespace spray_freeform_traj_plan;
    using smrobot::workbench::freeform::FreeformTrajPlanRenderer;

    FreeformTrajPlanRenderer renderer;

    // Model + surface: one document triangle becomes one indexed render triangle.
    FreeformSurfaceMesh mesh;
    FreeformTriangle triangle;
    triangle.vertices[0] = { 0.0, 0.0, 0.0 };
    triangle.vertices[1] = { 2.0, 0.0, 0.0 };
    triangle.vertices[2] = { 0.0, 3.0, 0.0 };
    mesh.triangles.push_back(triangle);
    const auto modelPreview = renderer.makeModelPreview(mesh);
    if(modelPreview.vertices.size() != 3 || modelPreview.indices.size() != 3 ||
        modelPreview.indices[0] != 0 || modelPreview.indices[1] != 1 ||
        modelPreview.indices[2] != 2) {
        std::cerr << "Model surface topology conversion failed.\n";
        return 1;
    }

    // Points: all document vertices retain their exact positions and generated normal.
    if(!nearlyEqual(modelPreview.vertices[1].positionX, 2.0) ||
        !nearlyEqual(modelPreview.vertices[2].positionY, 3.0) ||
        !nearlyEqual(modelPreview.vertices[0].normalZ, 1.0) ||
        !nearlyEqual(modelPreview.vertices[0].red, 0.70)) {
        std::cerr << "Model point conversion failed.\n";
        return 2;
    }

    // Lines: a document connection becomes one stable orange line segment.
    const std::vector<FreeformConnectionSegment> connections = {
        { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } }
    };
    const auto linePreview = renderer.makeConnectionPreview(connections);
    if(linePreview.size() != 1 || !nearlyEqual(linePreview.front().startX, 1.0) ||
        !nearlyEqual(linePreview.front().endZ, 6.0) ||
        !nearlyEqual(linePreview.front().red, 1.0) ||
        !nearlyEqual(linePreview.front().green, 0.60)) {
        std::cerr << "Connection line conversion failed.\n";
        return 3;
    }

    // Trajectory frame: each document point maps to red X, green Y, and blue Z axes.
    FreeformTrajectory trajectory(1);
    trajectory.front().positionX = 1.0;
    trajectory.front().positionY = 2.0;
    trajectory.front().positionZ = 3.0;
    trajectory.front().frameXAxisX = 2.0;
    trajectory.front().frameXAxisY = 0.0;
    trajectory.front().frameXAxisZ = 0.0;
    trajectory.front().frameYAxisX = 0.0;
    trajectory.front().frameYAxisY = 3.0;
    trajectory.front().frameYAxisZ = 0.0;
    trajectory.front().frameZAxisX = 0.0;
    trajectory.front().frameZAxisY = 0.0;
    trajectory.front().frameZAxisZ = 4.0;
    const auto framePreview = renderer.makeTrajectoryFramePreview(trajectory, 0.5);
    if(framePreview.size() != 3 || !nearlyEqual(framePreview[0].endX, 1.5) ||
        !nearlyEqual(framePreview[0].red, 1.0) || !nearlyEqual(framePreview[1].endY, 2.5) ||
        !nearlyEqual(framePreview[1].green, 1.0) || !nearlyEqual(framePreview[2].endZ, 3.5) ||
        !nearlyEqual(framePreview[2].blue, 1.0)) {
        std::cerr << "Trajectory frame axis conversion failed.\n";
        return 5;
    }

    // Boundary lines: document-owned upper/lower/offset lines keep their colors and closure.
    FreeformBoundaryResult boundaries;
    boundaries.upper.points = { { 0.0, 0.0, 1.0 }, { 1.0, 0.0, 1.0 }, { 1.0, 1.0, 1.0 } };
    boundaries.upper.color = { 1.0, 0.0, 0.0, 1.0 };
    boundaries.lower.points = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } };
    boundaries.lower.color = { 0.0, 0.0, 1.0, 1.0 };
    boundaries.lower.closed = false;
    const auto boundaryPreview = renderer.makeBoundaryPreview(boundaries);
    if(boundaryPreview.size() != 4 || !nearlyEqual(boundaryPreview[0].red, 1.0) ||
        !nearlyEqual(boundaryPreview[0].blue, 0.0) ||
        !nearlyEqual(boundaryPreview[2].endX, 0.0) ||
        !nearlyEqual(boundaryPreview[3].blue, 1.0)) {
        std::cerr << "Boundary line conversion failed.\n";
        return 6;
    }

    // Slice planes: a document plane is rendered from its persisted center and normal,
    // rather than as a fixed XY grid.
    FreeformSlicingResult slicing;
    slicing.selectedLayerIndex = 0;
    slicing.planes.push_back({ { 0.5, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } });
    slicing.contours.push_back({ { { 0.5, -0.2, -0.2 }, { 0.5, 0.2, -0.2 },
        { 0.5, 0.2, 0.2 }, { 0.5, -0.2, 0.2 } }, 0, true });
    const auto planePreview = renderer.makeSlicingPlanePreview(slicing);
    if(planePreview.empty() || !nearlyEqual(planePreview.front().startX, 0.5) ||
        !nearlyEqual(planePreview.front().endX, 0.5) ||
        nearlyEqual(planePreview.front().startZ, planePreview.front().endZ)) {
        std::cerr << "Rotated slicing plane conversion failed.\n";
        return 7;
    }

    if(freeform_preview_test::shouldShowViewport(argc, argv)) {
        freeform_preview_test::showPreview(argc, argv, QStringLiteral("Freeform: model, point, line and surface"),
            &modelPreview, { { "connections", linePreview }, { "trajectory-frames", framePreview },
                { "boundaries", boundaryPreview }, { "slice-planes", planePreview } });
    }
    return 0;
}
