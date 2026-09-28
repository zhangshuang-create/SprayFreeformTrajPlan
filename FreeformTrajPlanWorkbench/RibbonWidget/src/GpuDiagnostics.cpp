#include <QOpenGLContext>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>
#include <QDebug>

namespace ModelTrajPlan
{

void diagnoseGpuCapabilities(QOpenGLWidget* view)
{
    if (!view) {
        qWarning() << "=== GPU 诊断 ===";
        qWarning() << "❌ openGLView_ 为空指针";
        qWarning() << "原因：视图未初始化或未传入";
        return;
    }

    view->makeCurrent();
    QOpenGLContext* context = QOpenGLContext::currentContext();

    qInfo() << "\n=== GPU 厚度计算诊断 ===";

    // 检查 OpenGL 上下文
    if (!context || !context->isValid()) {
        qWarning() << "❌ OpenGL 上下文无效";
        qWarning() << "  - context 存在:" << (context != nullptr);
        if (context) {
            qWarning() << "  - context 有效:" << context->isValid();
        }
        view->doneCurrent();
        return;
    }

    qInfo() << "✅ OpenGL 上下文有效";

    // 检查 OpenGL 版本
    const auto format = context->format();
    const int major = format.majorVersion();
    const int minor = format.minorVersion();

    qInfo() << "OpenGL 版本:" << major << "." << minor;

    if (major > 4 || (major == 4 && minor >= 3)) {
        qInfo() << "✅ 支持 Compute Shader (需要 OpenGL 4.3+)";
    } else {
        qWarning() << "❌ 不支持 Compute Shader";
        qWarning() << "  - 当前版本:" << major << "." << minor;
        qWarning() << "  - 需要版本: 4.3 或更高";
        qWarning() << "  - 建议：更新显卡驱动";
        view->doneCurrent();
        return;
    }

    // 获取 OpenGL 函数
    auto* gl = context->versionFunctions<QOpenGLFunctions_4_3_Core>();
    if (!gl) {
        qWarning() << "❌ 无法获取 OpenGL 4.3 Core 函数";
        view->doneCurrent();
        return;
    }

    if (!gl->initializeOpenGLFunctions()) {
        qWarning() << "❌ OpenGL 函数初始化失败";
        view->doneCurrent();
        return;
    }

    qInfo() << "✅ OpenGL 4.3 Core 函数可用";

    // 查询硬件信息
    const char* renderer = reinterpret_cast<const char*>(gl->glGetString(GL_RENDERER));
    const char* vendor = reinterpret_cast<const char*>(gl->glGetString(GL_VENDOR));
    const char* version = reinterpret_cast<const char*>(gl->glGetString(GL_VERSION));
    const char* glslVersion = reinterpret_cast<const char*>(gl->glGetString(GL_SHADING_LANGUAGE_VERSION));

    qInfo() << "\n硬件信息:";
    qInfo() << "  - 显卡:" << (renderer ? renderer : "未知");
    qInfo() << "  - 厂商:" << (vendor ? vendor : "未知");
    qInfo() << "  - 驱动版本:" << (version ? version : "未知");
    qInfo() << "  - GLSL 版本:" << (glslVersion ? glslVersion : "未知");

    // 查询 Compute Shader 限制
    GLint maxSsboSize = 0;
    GLint maxComputeGroups[3] = {0, 0, 0};
    GLint maxComputeGroupSize[3] = {0, 0, 0};
    GLint maxComputeInvocations = 0;
    GLint maxSsbos = 0;

    gl->glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &maxSsboSize);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &maxComputeGroups[0]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &maxComputeGroups[1]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &maxComputeGroups[2]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &maxComputeGroupSize[0]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &maxComputeGroupSize[1]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &maxComputeGroupSize[2]);
    gl->glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &maxComputeInvocations);
    gl->glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &maxSsbos);

    qInfo() << "\nCompute Shader 能力:";
    qInfo() << "  - 最大 SSBO 大小:" << (maxSsboSize / 1024 / 1024) << "MB";
    qInfo() << "  - 最大工作组数量:" << maxComputeGroups[0] << "x"
            << maxComputeGroups[1] << "x" << maxComputeGroups[2];
    qInfo() << "  - 最大工作组大小:" << maxComputeGroupSize[0] << "x"
            << maxComputeGroupSize[1] << "x" << maxComputeGroupSize[2];
    qInfo() << "  - 最大调用数:" << maxComputeInvocations;
    qInfo() << "  - 最大 SSBO 绑定:" << maxSsbos;

    // 检查基本要求
    qInfo() << "\n能力检查:";

    bool allPassed = true;

    if (maxSsboSize < 16 * 1024 * 1024) {  // 至少 16MB
        qWarning() << "⚠️  SSBO 大小可能不足（需要 >=16MB）";
        allPassed = false;
    } else {
        qInfo() << "✅ SSBO 大小充足";
    }

    if (maxSsbos < 5) {  // 需要 5 个 SSBO
        qWarning() << "❌ SSBO 绑定点不足（需要 >=5）";
        allPassed = false;
    } else {
        qInfo() << "✅ SSBO 绑定点充足";
    }

    if (maxComputeGroupSize[0] < 256) {  // local_size_x = 256
        qWarning() << "⚠️  工作组 X 维度可能不足（需要 >=256）";
        allPassed = false;
    } else {
        qInfo() << "✅ 工作组大小支持";
    }

    if (allPassed) {
        qInfo() << "\n🎉 GPU 加速完全可用！";
        qInfo() << "如果仍然使用 CPU 后备，检查：";
        qInfo() << "  1. shader 文件是否正确加载（:/RibbonWidget/thickness_compute.comp）";
        qInfo() << "  2. 输入数据是否有效（surfaceSamples, trajectory）";
        qInfo() << "  3. 查看程序日志中的 [THICKNESS GPU] 消息";
    } else {
        qWarning() << "\n⚠️  GPU 能力不足，将使用 CPU 后备";
    }

    qInfo() << "==================\n";

    view->doneCurrent();
}

} // namespace ModelTrajPlan
