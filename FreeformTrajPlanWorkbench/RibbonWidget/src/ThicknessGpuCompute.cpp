#include "ThicknessGpuCompute.h"

#include <QFile>
#include <QOpenGLContext>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLShaderProgram>
#include <QDebug>
#include <QElapsedTimer>
#include <QVector4D>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <vector>
#include <LinearInterpolator/LinearInterpolator.h>

namespace ModelTrajPlan
{
namespace
{
struct GpuSurface { float px, py, pz, pw; float nx, ny, nz, nw; };
struct GpuSpray {
    float px, py, pz, timestamp;
    float axx, axy, axz, axw;
    float ayx, ayy, ayz, ayw;
    float azx, azy, azz, sprayDistance;
    int trajectoryIndex, pad0, pad1, pad2;
};

bool hasCompute(QOpenGLContext* context)
{
    if (!context || !context->isValid()) return false;
    const auto format = context->format();
    return format.majorVersion() > 4 || (format.majorVersion() == 4 && format.minorVersion() >= 3);
}

const char* glErrorName(GLenum error)
{
    switch (error) {
    case GL_NO_ERROR: return "GL_NO_ERROR";
    case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
    case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
    case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
    default: return "unknown OpenGL error";
    }
}

bool checkGlError(QOpenGLFunctions_4_3_Core* gl, const char* stage)
{
    const GLenum error = gl->glGetError();
    if (error == GL_NO_ERROR) return true;
    qWarning() << "[THICKNESS GPU]" << stage << "failed:" << glErrorName(error)
               << "(0x" + QByteArray::number(error, 16) + ')';
    return false;
}
}

bool computeThicknessOnGpu(QOpenGLWidget* view, const ThicknessAnalysisInput& input,
                           ThicknessHeatmapResult& result)
{
    result = ThicknessHeatmapResult{};
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (!view || !context || !hasCompute(context)) {
        qInfo() << "[THICKNESS GPU] unavailable: no current OpenGL 4.3+ context";
        return false;
    }
    if (input.surfaceSamples.empty() || input.trajectory.size() < 2) {
        qInfo() << "[THICKNESS GPU] unavailable: insufficient input data";
        return false;
    }
    std::vector<TrajectoryPoint> dense;
    if (!LinearInterpolator::interpolateForThickness(input.trajectory, input.interpolation, dense)
        || dense.size() < 2) {
        qWarning() << "[THICKNESS GPU] interpolation failed; CPU fallback required";
        return false;
    }

    QFile shaderFile(QStringLiteral(":/RibbonWidget/thickness_compute_v3.comp"));
    if (!shaderFile.open(QIODevice::ReadOnly)) return false;
    const QByteArray shaderSource = shaderFile.readAll();
    qInfo() << "[THICKNESS GPU] shader revision="
            << (shaderSource.contains("MODEL_SHADER_REVISION 4") ? 4 : 0);
    QOpenGLShaderProgram program;
    if (!program.addShaderFromSourceCode(QOpenGLShader::Compute, shaderSource)) {
        qWarning() << "Thickness compute shader compilation failed:" << program.log();
        return false;
    }
    if (!program.link()) {
        qWarning() << "Thickness compute shader link failed:" << program.log();
        return false;
    }
    auto* gl = context->versionFunctions<QOpenGLFunctions_4_3_Core>();
    if (!gl || !gl->initializeOpenGLFunctions()) return false;

    GLint maxSsboSize = 0;
    GLint maxComputeGroups[3] = {};
    gl->glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &maxSsboSize);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &maxComputeGroups[0]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &maxComputeGroups[1]);
    gl->glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &maxComputeGroups[2]);
    qInfo() << "[THICKNESS GPU] renderer="
            << reinterpret_cast<const char*>(gl->glGetString(GL_RENDERER))
            << "version=" << reinterpret_cast<const char*>(gl->glGetString(GL_VERSION))
            << "maxSsboBytes=" << maxSsboSize << "maxGroups="
            << maxComputeGroups[0] << maxComputeGroups[1] << maxComputeGroups[2];
    // Driver limit queries may report a compatibility-profile error even when
    // the values are returned correctly. Do not attribute that stale error to
    // the first SSBO upload below.
    for (GLenum error = gl->glGetError(); error != GL_NO_ERROR; error = gl->glGetError()) {
        qWarning() << "[THICKNESS GPU] clearing pre-upload OpenGL error:"
                   << glErrorName(error) << "(0x" + QByteArray::number(error, 16) + ')';
    }

    std::vector<GpuSurface> surfaces(input.surfaceSamples.size());
    for (size_t i = 0; i < surfaces.size(); ++i) {
        const auto& s = input.surfaceSamples[i];
        surfaces[i] = {float(s.position.x()), float(s.position.y()), float(s.position.z()), 0.0f,
                        float(s.normal.x()), float(s.normal.y()), float(s.normal.z()), 0.0f};
    }
    std::vector<GpuSpray> sprays(dense.size());
    for (size_t i = 0; i < dense.size(); ++i) {
        const auto& p = dense[i];
        const auto pos = p.pose.block<3,1>(0,3);
        const auto x = p.pose.block<3,1>(0,0);
        const auto y = p.pose.block<3,1>(0,1);
        const auto z = p.pose.block<3,1>(0,2);
        sprays[i] = {float(pos.x()), float(pos.y()), float(pos.z()), float(p.timestamp),
                     float(x.x()), float(x.y()), float(x.z()), 0.0f,
                     float(y.x()), float(y.y()), float(y.z()), 0.0f,
                     float(z.x()), float(z.y()), float(z.z()), float(std::abs(p.spray_distance)),
                     p.trajectory_index, 0, 0, 0};
    }
    qInfo() << "[THICKNESS GPU] shader ready; surfaces=" << surfaces.size()
            << "spray points=" << sprays.size();

    // Build a compact per-surface candidate list. Keep this bounded: a
    // stand-off-scaled cube duplicates spray indices for every surface and
    // exhausts host memory on dense rotating models.
    constexpr float kCellSize = 25.0f;
    constexpr int kCellRadius = 2;
    float minX = surfaces.front().px, minY = surfaces.front().py, minZ = surfaces.front().pz;
    float maxX = minX, maxY = minY, maxZ = minZ;
    for (const auto& s : surfaces) {
        minX = std::min(minX, s.px); minY = std::min(minY, s.py); minZ = std::min(minZ, s.pz);
        maxX = std::max(maxX, s.px); maxY = std::max(maxY, s.py); maxZ = std::max(maxZ, s.pz);
    }
    minX -= kCellSize * kCellRadius; minY -= kCellSize * kCellRadius; minZ -= kCellSize * kCellRadius;
    maxX += kCellSize * kCellRadius; maxY += kCellSize * kCellRadius; maxZ += kCellSize * kCellRadius;
    const int dimX = int(std::ceil((maxX - minX) / kCellSize)) + 1;
    const int dimY = int(std::ceil((maxY - minY) / kCellSize)) + 1;
    const int dimZ = int(std::ceil((maxZ - minZ) / kCellSize)) + 1;
    const size_t cellCount = size_t(dimX) * size_t(dimY) * size_t(dimZ);
    auto cellOf = [&](float x, float y, float z) {
        const int cx = std::clamp(int((x - minX) / kCellSize), 0, dimX - 1);
        const int cy = std::clamp(int((y - minY) / kCellSize), 0, dimY - 1);
        const int cz = std::clamp(int((z - minZ) / kCellSize), 0, dimZ - 1);
        return size_t(cx) + size_t(dimX) * (size_t(cy) + size_t(dimY) * size_t(cz));
    };
    std::vector<uint32_t> cellCounts(cellCount, 0);
    for (const auto& p : sprays) ++cellCounts[cellOf(p.px, p.py, p.pz)];
    std::vector<uint32_t> cellOffsets(cellCount + 1, 0);
    for (size_t i = 0; i < cellCount; ++i) cellOffsets[i + 1] = cellOffsets[i] + cellCounts[i];
    std::vector<uint32_t> cellSprays(sprays.size());
    std::vector<uint32_t> cellCursor = cellOffsets;
    for (uint32_t i = 0; i < sprays.size(); ++i) cellSprays[cellCursor[cellOf(sprays[i].px, sprays[i].py, sprays[i].pz)]++] = i;
    // Store every spray index once. The compute shader looks up neighboring
    // cells itself; duplicating those indices for every surface can exceed the
    // 2 GB per-SSBO driver limit on dense trajectories.
    qInfo() << "[THICKNESS GPU] spatial grid cells=" << cellCount
            << "spray indices=" << cellSprays.size()
            << "grid=" << dimX << dimY << dimZ << "cellRadius=" << kCellRadius;

    const GLsizeiptr surfaceBytes = GLsizeiptr(surfaces.size() * sizeof(GpuSurface));
    const GLsizeiptr outputBytes = GLsizeiptr(surfaces.size() * sizeof(float));
    const GLsizeiptr sprayBytes = GLsizeiptr(sprays.size() * sizeof(GpuSpray));
    const GLsizeiptr cellOffsetBytes = GLsizeiptr(cellOffsets.size() * sizeof(uint32_t));
    const GLsizeiptr cellSprayBytes = GLsizeiptr(cellSprays.size() * sizeof(uint32_t));
    if (surfaceBytes > maxSsboSize || outputBytes > maxSsboSize || sprayBytes > maxSsboSize ||
        cellOffsetBytes > maxSsboSize || cellSprayBytes > maxSsboSize) {
        qWarning() << "[THICKNESS GPU] SSBO exceeds GL_MAX_SHADER_STORAGE_BLOCK_SIZE:"
                   << "surface=" << surfaceBytes << "output=" << outputBytes << "spray=" << sprayBytes
                   << "cellOffsets=" << cellOffsetBytes << "cellSprays=" << cellSprayBytes
                   << "limit=" << maxSsboSize;
        return false;
    }

    GLuint buffers[5] = {};
    gl->glGenBuffers(5, buffers);
    if (!checkGlError(gl, "SSBO allocation")) { return false; }
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[0]);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER, surfaceBytes, surfaces.data(), GL_STATIC_DRAW);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, buffers[0]);
    if (!checkGlError(gl, "surface SSBO upload")) { gl->glDeleteBuffers(5, buffers); return false; }

    // An uninitialized buffer often appears as zeros. NaN makes a missing
    // shader write observable in the readback diagnostics.
    std::vector<float> unwritten(surfaces.size(), std::numeric_limits<float>::quiet_NaN());
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[1]);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER, outputBytes, unwritten.data(), GL_DYNAMIC_READ);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffers[1]);
    if (!checkGlError(gl, "output SSBO upload")) { gl->glDeleteBuffers(5, buffers); return false; }

    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[2]);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER, sprayBytes, sprays.data(), GL_STATIC_DRAW);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, buffers[2]);
    if (!checkGlError(gl, "spray SSBO upload")) { gl->glDeleteBuffers(5, buffers); return false; }
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[3]);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER, cellOffsetBytes, cellOffsets.data(), GL_STATIC_DRAW);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, buffers[3]);
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[4]);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER, cellSprayBytes, cellSprays.data(), GL_STATIC_DRAW);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, buffers[4]);
    if (!checkGlError(gl, "spatial-grid SSBO upload")) { gl->glDeleteBuffers(5, buffers); return false; }

    if (!program.bind()) { gl->glDeleteBuffers(5, buffers); return false; }
    // Use the raw program binding as well. Some Qt/OpenGL driver combinations
    // leave a stale pipeline state after QOpenGLShaderProgram::bind(), which
    // makes glDispatchCompute report GL_INVALID_OPERATION.
    gl->glUseProgram(program.programId());
    const GLint surfaceCountLocation = gl->glGetUniformLocation(program.programId(), "surfaceCount");
    const GLint sprayCountLocation = gl->glGetUniformLocation(program.programId(), "sprayCount");
    const GLint modelTypeLocation = gl->glGetUniformLocation(program.programId(), "modelType");
    if (surfaceCountLocation < 0 || sprayCountLocation < 0 || modelTypeLocation < 0) {
        qWarning() << "[THICKNESS GPU] required uniform missing:" << surfaceCountLocation
                   << sprayCountLocation << modelTypeLocation;
        program.release(); gl->glDeleteBuffers(5, buffers); return false;
    }
    gl->glUniform1ui(surfaceCountLocation, GLuint(surfaces.size()));
    gl->glUniform1ui(sprayCountLocation, GLuint(sprays.size()));
    gl->glUniform1i(modelTypeLocation, int(input.modelType));
    const GLint gaussianSigmaLocation = gl->glGetUniformLocation(program.programId(), "gaussianSigma");
    const GLint gaussianPeakLocation = gl->glGetUniformLocation(program.programId(), "gaussianPeak");
    const GLint gaussianEfficiencyLocation = gl->glGetUniformLocation(program.programId(), "gaussianEfficiency");
    if (gaussianSigmaLocation >= 0) gl->glUniform1f(gaussianSigmaLocation, float(input.gaussian.sigma));
    if (gaussianPeakLocation >= 0) gl->glUniform1f(gaussianPeakLocation, float(input.gaussian.peakHeight));
    if (gaussianEfficiencyLocation >= 0) gl->glUniform1f(gaussianEfficiencyLocation, float(input.gaussian.efficiency));
    const auto setDepositionUniform = [&](const char* name, double value) {
        const GLint location = gl->glGetUniformLocation(program.programId(), name);
        if (location >= 0) gl->glUniform1f(location, float(value));
    };
    setDepositionUniform("depositionGamma", input.deposition.gamma);
    setDepositionUniform("depositionAlphaX", input.deposition.alpha_x);
    setDepositionUniform("depositionAlphaY", input.deposition.alpha_y);
    setDepositionUniform("depositionBeta1", input.deposition.beta1);
    setDepositionUniform("depositionBeta2", input.deposition.beta2);
    setDepositionUniform("depositionBeta3", input.deposition.beta3);
    setDepositionUniform("depositionKEpsilon", input.deposition.k_epsilon);
    setDepositionUniform("depositionTMax", input.deposition.T_max);
    setDepositionUniform("depositionDelta", input.deposition.delta);
    const GLint gridDimLocation = gl->glGetUniformLocation(program.programId(), "spatialGridDim");
    const GLint gridMinLocation = gl->glGetUniformLocation(program.programId(), "spatialGridMin");
    const GLint gridCellSizeLocation = gl->glGetUniformLocation(program.programId(), "spatialCellSize");
    const GLint gridRadiusLocation = gl->glGetUniformLocation(program.programId(), "spatialCellRadius");
    if (gridDimLocation < 0 || gridMinLocation < 0 || gridCellSizeLocation < 0 || gridRadiusLocation < 0) {
        qWarning() << "[THICKNESS GPU] spatial-grid uniform missing";
        program.release(); gl->glDeleteBuffers(5, buffers); return false;
    }
    gl->glUniform3i(gridDimLocation, dimX, dimY, dimZ);
    gl->glUniform3f(gridMinLocation, minX, minY, minZ);
    gl->glUniform1f(gridCellSizeLocation, kCellSize);
    gl->glUniform1i(gridRadiusLocation, kCellRadius);
    if (!checkGlError(gl, "uniform upload")) { program.release(); gl->glDeleteBuffers(5, buffers); return false; }
    qInfo() << "[THICKNESS GPU] uniforms modelType=" << int(input.modelType)
            << "surfaceCount=" << surfaces.size() << "sprayCount=" << sprays.size()
            << "firstDt=" << (sprays.size() > 1 ? sprays[1].timestamp - sprays[0].timestamp : 0.0f)
            << "firstTrajectoryIndex=" << (sprays.empty() ? -999 : sprays.front().trajectoryIndex);
    const GLuint groups = (GLuint(surfaces.size()) + 255u) / 256u;
    while (gl->glGetError() != GL_NO_ERROR) {}
    QElapsedTimer dispatchTimer;
    dispatchTimer.start();
    gl->glDispatchCompute(groups, 1, 1);
    gl->glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
    gl->glFinish();
    qInfo() << "[THICKNESS GPU] dispatch completed in" << dispatchTimer.elapsed() << "ms";
    if (!checkGlError(gl, "compute dispatch") || !context->isValid()) {
        qWarning() << "[THICKNESS GPU] context invalid after compute dispatch";
        program.release(); gl->glDeleteBuffers(5, buffers); return false;
    }
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffers[1]);
    void* mapped = gl->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0,
        outputBytes, GL_MAP_READ_BIT);
    if (!mapped) { program.release(); gl->glDeleteBuffers(5, buffers); return false; }
    result.thicknessValues.resize(input.surfaceSamples.size());
    const float* values = static_cast<const float*>(mapped);
    bool hasFiniteValue = false;
    bool hasPositiveValue = false;
    for (size_t i = 0; i < result.thicknessValues.size(); ++i) {
        const float value = values[i];
        if (std::isfinite(value)) {
            hasFiniteValue = true;
            hasPositiveValue = hasPositiveValue || value > 1e-12f;
            result.thicknessValues[i] = value;
        } else {
            result.thicknessValues[i] = 0.0;
        }
    }
    qInfo() << "[THICKNESS GPU] raw output first=" << result.thicknessValues.front()
            << "second=" << (result.thicknessValues.size() > 1 ? result.thicknessValues[1] : 0.0)
            << "finite=" << hasFiniteValue << "positive=" << hasPositiveValue;
    gl->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    program.release();
    gl->glDeleteBuffers(5, buffers);

    // A successful dispatch that produced no finite/non-zero sample is not a
    // usable GPU result. Let RibbonWidget invoke the established CPU fallback.
    if (!hasFiniteValue || !hasPositiveValue) {
        qWarning() << "[THICKNESS GPU] dispatch returned zero/invalid values; CPU fallback required";
        result = ThicknessHeatmapResult{};
        return false;
    }

    ThicknessAnalyzer analyzer;
    result.statistics = analyzer.computeStatistics(result.thicknessValues, input.targetThickness,
                                                   input.lowerTolerance, input.upperTolerance,
                                                   input.surfaceSamples, input.meshIndices);
    analyzer.normalizeThickness(result.thicknessValues, result.normalizedValues);
    analyzer.generateHeatmapColors(result.thicknessValues, result.colors);
    qInfo() << "[THICKNESS GPU] enabled; min=" << result.statistics.minThickness
            << "max=" << result.statistics.maxThickness;
    return true;
}
}
