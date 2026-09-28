#pragma once

class QOpenGLWidget;

namespace ModelTrajPlan
{

// 诊断 GPU 计算能力，输出详细的硬件信息和能力检查
// 在程序启动时或第一次厚度计算前调用
void diagnoseGpuCapabilities(QOpenGLWidget* view);

} // namespace ModelTrajPlan
