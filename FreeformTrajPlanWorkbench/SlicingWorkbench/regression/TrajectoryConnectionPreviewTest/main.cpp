#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"
#include <cmath>
#include <iostream>
int main(int argc, char** argv)
{
    using namespace spray_freeform_traj_plan;
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    const std::vector<FreeformConnectionSegment> connections = { { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } } };
    const auto preview = renderer.makeConnectionPreview(connections);
    if(preview.size() != 1 || std::abs(preview.front().startX - 1.0) > 1.0e-12 ||
        std::abs(preview.front().endZ - 6.0) > 1.0e-12 || preview.front().red != 1.0 ||
        std::abs(preview.front().green - 0.60) > 1.0e-12) {
        std::cerr << "Trajectory connection preview conversion failed.\n";
        return 1;
    }
    if(freeform_preview_test::shouldShowViewport(argc, argv)) {
        freeform_preview_test::showPreview(argc, argv, QStringLiteral("Freeform: trajectory connections"),
            nullptr, { { "connections", preview } });
    }
    return 0;
}
