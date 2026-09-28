#pragma once

#include <QOpenGLWidget>
#include <ThicknessAnalyzer/ThicknessAnalyzer.h>

namespace ModelTrajPlan
{
// GPU entry point. The call must be made on the GUI thread with the supplied
// QOpenGLWidget current; it returns false when compute shaders are unavailable.
bool computeThicknessOnGpu(
    QOpenGLWidget* view,
    const ThicknessAnalysisInput& input,
    ThicknessHeatmapResult& result);
}
