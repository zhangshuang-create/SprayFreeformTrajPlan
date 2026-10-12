#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"
#include <iostream>
int main(int argc, char** argv)
{
    using namespace spray_freeform_traj_plan;
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    FreeformSurfaceMesh mesh;
    FreeformTriangle triangle;
    triangle.vertices[0] = { 0.0, 0.0, 0.0 };
    triangle.vertices[1] = { 1.0, 0.0, 0.0 };
    triangle.vertices[2] = { 0.0, 1.0, 0.0 };
    mesh.triangles.push_back(triangle);
    const auto preview = renderer.makeModelPreview(mesh);
    if(preview.vertices.size() != 3 || preview.indices.size() != 3 || preview.vertices[0].normalZ != 1.0) {
        std::cerr << "Model surface preview conversion failed.\n";
        return 1;
    }
    if(freeform_preview_test::shouldShowViewport(argc, argv)) {
        freeform_preview_test::showPreview(argc, argv, QStringLiteral("Freeform: model surface"), &preview, {});
    }
    return 0;
}
