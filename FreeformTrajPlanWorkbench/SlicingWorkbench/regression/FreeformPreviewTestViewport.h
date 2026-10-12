#pragma once

#include "FreeformTrajPlanRenderer.h"

#include <RobotViewport.h>
#include <SimulationProject/ProjectDocument.h>
#include <VisualizationSDK/CustomMesh.h>

#include <QApplication>
#include <QSurfaceFormat>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>

namespace freeform_preview_test
{
    inline bool shouldShowViewport(int argc, char** argv)
    {
        for(int index = 1; index < argc; ++index) {
            if(std::string(argv[index]) == "--verify") {
                return false;
            }
        }
        return true;
    }

    inline void appendLine(
        smrobot::visualization::MeshData& mesh,
        const smrobot::workbench::freeform::FreeformTrajPlanLineSegment& segment,
        float halfWidth = 0.0025f)
    {
        using smrobot::visualization::Color4f;
        using smrobot::visualization::Vec3f;

        const Vec3f start{ static_cast<float>(segment.startX), static_cast<float>(segment.startY),
            static_cast<float>(segment.startZ) };
        const Vec3f end{ static_cast<float>(segment.endX), static_cast<float>(segment.endY),
            static_cast<float>(segment.endZ) };
        const float dx = end.x - start.x;
        const float dy = end.y - start.y;
        const float dz = end.z - start.z;
        const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
        if(length <= 1.0e-6f) {
            return;
        }
        const Vec3f direction{ dx / length, dy / length, dz / length };
        const Vec3f reference = std::abs(direction.z) < 0.9f
            ? Vec3f{ 0.0f, 0.0f, 1.0f }
            : Vec3f{ 0.0f, 1.0f, 0.0f };
        const Vec3f offset{
            (direction.y * reference.z - direction.z * reference.y) * halfWidth,
            (direction.z * reference.x - direction.x * reference.z) * halfWidth,
            (direction.x * reference.y - direction.y * reference.x) * halfWidth };
        const std::uint32_t base = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.insert(mesh.positions.end(), {
            { start.x + offset.x, start.y + offset.y, start.z + offset.z },
            { start.x - offset.x, start.y - offset.y, start.z - offset.z },
            { end.x - offset.x, end.y - offset.y, end.z - offset.z },
            { end.x + offset.x, end.y + offset.y, end.z + offset.z }
        });
        mesh.colors.insert(mesh.colors.end(), 4, Color4f{ static_cast<float>(segment.red),
            static_cast<float>(segment.green), static_cast<float>(segment.blue), static_cast<float>(segment.alpha) });
        mesh.indices.insert(mesh.indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
    }

    inline void showPreview(
        int argc,
        char** argv,
        const QString& title,
        const smrobot::workbench::freeform::FreeformTrajPlanMeshPreview* surface,
        std::initializer_list<std::pair<std::string, smrobot::workbench::freeform::FreeformTrajPlanLinePreview>> lines)
    {
        QSurfaceFormat format;
        format.setDepthBufferSize(24);
        format.setStencilBufferSize(8);
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        QSurfaceFormat::setDefaultFormat(format);

        QApplication application(argc, argv);
        RobotViewport viewport;
        viewport.setWindowTitle(title);
        viewport.resize(1000, 720);
        viewport.show();

        simulation_project::ProjectDocument document;
        document.view.showGrid = true;
        document.view.showAxis = true;
        if(!viewport.loadProjectDocument(document, {})) {
            return;
        }

        using namespace smrobot::visualization;
        if(surface != nullptr && !surface->vertices.empty() && !surface->indices.empty()) {
            CustomMeshDesc descriptor;
            descriptor.ownerId = "freeform-preview-test";
            descriptor.meshId = "surface";
            descriptor.name = "Surface";
            descriptor.mesh.normalMode = MeshNormalMode::GenerateFlat;
            descriptor.appearance.cull = MeshCullMode::None;
            descriptor.appearance.blend = MeshBlendMode::AlphaBlend;
            descriptor.appearance.depthWrite = false;
            for(const auto& vertex : surface->vertices) {
                descriptor.mesh.positions.push_back({ static_cast<float>(vertex.positionX),
                    static_cast<float>(vertex.positionY), static_cast<float>(vertex.positionZ) });
                descriptor.mesh.colors.push_back({ static_cast<float>(vertex.red), static_cast<float>(vertex.green),
                    static_cast<float>(vertex.blue), static_cast<float>(vertex.alpha) });
            }
            descriptor.mesh.indices = surface->indices;
            CustomMeshHandle handle;
            viewport.upsertCustomMesh(descriptor, handle);
        }

        for(const auto& [meshId, preview] : lines) {
            if(preview.empty()) {
                continue;
            }
            CustomMeshDesc descriptor;
            descriptor.ownerId = "freeform-preview-test";
            descriptor.meshId = meshId;
            descriptor.name = meshId;
            descriptor.mesh.normalMode = MeshNormalMode::GenerateFlat;
            descriptor.appearance.shading = MeshShadingMode::Unlit;
            descriptor.appearance.cull = MeshCullMode::None;
            descriptor.appearance.blend = MeshBlendMode::AlphaBlend;
            descriptor.appearance.depthWrite = false;
            descriptor.appearance.renderOrder = 20;
            for(const auto& segment : preview) {
                appendLine(descriptor.mesh, segment);
            }
            CustomMeshHandle handle;
            viewport.upsertCustomMesh(descriptor, handle);
        }

        application.exec();
    }
}
