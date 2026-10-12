#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"
#include <cmath>
#include <iostream>
int main(int argc, char** argv)
{
    using namespace spray_freeform_traj_plan;
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    FreeformTrajectory trajectory(1);
    trajectory[0].positionX = 1.0;
    trajectory[0].positionY = 2.0;
    trajectory[0].positionZ = 3.0;
    trajectory[0].frameXAxisX = 1.0;
    trajectory[0].frameYAxisY = 1.0;
    trajectory[0].frameZAxisZ = 1.0;
    const auto preview = renderer.makeTrajectoryFramePreview(trajectory, 0.5);
    if(preview.size() != 3 || std::abs(preview[0].endX - 1.5) > 1.0e-12 ||
        preview[0].red != 1.0 || preview[1].green != 1.0 || preview[2].blue != 1.0) {
        std::cerr << "Trajectory frame preview conversion failed.\n";
        return 1;
    }
    if(freeform_preview_test::shouldShowViewport(argc, argv)) {
        freeform_preview_test::showPreview(argc, argv, QStringLiteral("Freeform: trajectory frames"),
            nullptr, { { "trajectory-frames", preview } });
    }
    return 0;
}
