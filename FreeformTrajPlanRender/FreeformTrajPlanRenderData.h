#pragma once

#include <cstdint>
#include <vector>

namespace smrobot::workbench::freeform
{
    struct FreeformTrajPlanPreviewPoint
    {
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double directionX = 0.0;
        double directionY = 0.0;
        double directionZ = 1.0;
        double frameXAxisX = 1.0;
        double frameXAxisY = 0.0;
        double frameXAxisZ = 0.0;
        double frameYAxisX = 0.0;
        double frameYAxisY = 1.0;
        double frameYAxisZ = 0.0;
        double frameZAxisX = 0.0;
        double frameZAxisY = 0.0;
        double frameZAxisZ = 1.0;
        bool sprayEnabled = false;
        bool startsNewSegment = false;
    };

    using FreeformTrajPlanPreview = std::vector<FreeformTrajPlanPreviewPoint>;

    struct FreeformTrajPlanLineSegment
    {
        double startX = 0.0;
        double startY = 0.0;
        double startZ = 0.0;
        double endX = 0.0;
        double endY = 0.0;
        double endZ = 0.0;
        double red = 1.0;
        double green = 1.0;
        double blue = 1.0;
        double alpha = 1.0;
    };

    using FreeformTrajPlanLinePreview = std::vector<FreeformTrajPlanLineSegment>;

    struct FreeformTrajPlanMeshVertex
    {
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double normalX = 0.0;
        double normalY = 0.0;
        double normalZ = 1.0;
        double red = 0.70;
        double green = 0.70;
        double blue = 0.70;
        double alpha = 1.0;
    };

    struct FreeformTrajPlanMeshPreview
    {
        std::vector<FreeformTrajPlanMeshVertex> vertices;
        std::vector<std::uint32_t> indices;
    };
}
