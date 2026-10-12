#include "FreeformTrajPlanWorkbenchController.h"
#include "FreeformTrajPlanWorkbenchPanel.h"
#include "RobotQtViewerEvents.h"
#include "RobotQtViewerDocumentContext.h"
#include "SceneEntityWorkflowController.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportPorts.h"
#include "RobotQtViewerViewportServices.h"
#include "ViewportReloadWorkflowController.h"

#include <SimulationProject/ProjectDocument.h>
#include <VisualizationSDK/CustomMesh.h>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/TrajectoryExporter.h>
#include <SlicingTrajPlan/TrajectoryFrame.h>
#include <QSettings>
#include <QComboBox>

#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <QFileInfo>
#include <QFileDialog>
#include <QDialog>
#include <QDialogButtonBox>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <string>
#include <utility>
#include <Eigen/Geometry>
#include "RobotQtViewerDocumentController.h"
#include <SimulationProject/ProjectDocumentService.h>
namespace
{
    constexpr const char* kFreeformOverlayOwner = "freeformTrajPlan";

    std::string overlayOwnerId(const QString& overlayId)
    {
        return std::string(kFreeformOverlayOwner) + "." + overlayId.toStdString();
    }

    void appendLine(
        smrobot::visualization::MeshData& mesh,
        const smrobot::visualization::Vec3f& start,
        const smrobot::visualization::Vec3f& end,
        const smrobot::visualization::Color4f& color,
        float width = 0.0015f)
    {
        const float dx = end.x - start.x;
        const float dy = end.y - start.y;
        const float dz = end.z - start.z;
        const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
        if(length < 1.0e-6f) {
            return;
        }

        const smrobot::visualization::Vec3f direction{ dx / length, dy / length, dz / length };
        const smrobot::visualization::Vec3f reference = std::abs(direction.z) < 0.9f
            ? smrobot::visualization::Vec3f{ 0.0f, 0.0f, 1.0f }
            : smrobot::visualization::Vec3f{ 0.0f, 1.0f, 0.0f };
        const smrobot::visualization::Vec3f offset{
            (direction.y * reference.z - direction.z * reference.y) * width,
            (direction.z * reference.x - direction.x * reference.z) * width,
            (direction.x * reference.y - direction.y * reference.x) * width };
        const std::uint32_t base = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.insert(mesh.positions.end(), {
            { start.x + offset.x, start.y + offset.y, start.z + offset.z },
            { start.x - offset.x, start.y - offset.y, start.z - offset.z },
            { end.x - offset.x, end.y - offset.y, end.z - offset.z },
            { end.x + offset.x, end.y + offset.y, end.z + offset.z }
        });
        mesh.colors.insert(mesh.colors.end(), 4, color);
        mesh.indices.insert(mesh.indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
    }

    smrobot::visualization::CustomMeshDesc makeOverlayDescriptor(const QString& overlayId)
    {
        smrobot::visualization::CustomMeshDesc descriptor;
        descriptor.ownerId = overlayOwnerId(overlayId);
        descriptor.meshId = overlayId.toStdString();
        descriptor.name = descriptor.meshId;
        descriptor.mesh.normalMode = smrobot::visualization::MeshNormalMode::GenerateFlat;
        descriptor.appearance.shading = smrobot::visualization::MeshShadingMode::Unlit;
        descriptor.appearance.cull = smrobot::visualization::MeshCullMode::None;
        descriptor.appearance.blend = smrobot::visualization::MeshBlendMode::AlphaBlend;
        descriptor.appearance.depthWrite = false;
        if(overlayId == QStringLiteral("referenceLines")) {
            descriptor.appearance.depthTest = false;
            descriptor.appearance.renderOrder = 30;
        }
        return descriptor;
    }

    void clearOverlay(robot_qt_viewer::IRobotQtViewerVisualizationViewportPort* viewport, const QString& overlayId)
    {
        if(viewport != nullptr) {
            viewport->clearCustomMeshes(overlayOwnerId(overlayId));
        }
    }

    void clearAllOverlays(robot_qt_viewer::IRobotQtViewerVisualizationViewportPort* viewport)
    {
        for(const QString& overlayId : { QStringLiteral("connections"), QStringLiteral("slices"),
                QStringLiteral("slicePlanes"), QStringLiteral("sliceSurfaces"),
                QStringLiteral("trajectoryFrames"), QStringLiteral("referenceLines"), QStringLiteral("frameConfiguration") }) {
            clearOverlay(viewport, overlayId);
        }
    }

    bool selectModelUnitScale(QWidget* parent, const QString& path, double* scaleToMeters)
    {
        if(scaleToMeters == nullptr) {
            return false;
        }
        QMessageBox dialog(parent);
        dialog.setWindowTitle(QStringLiteral("模型单位"));
        dialog.setIcon(QMessageBox::Question);
        dialog.setText(QStringLiteral("请选择 %1 的坐标单位。").arg(QFileInfo(path).fileName()));
        dialog.setInformativeText(QStringLiteral(
            "STL 文件通常不包含单位信息。选择毫米会转换为场景使用的米单位。"));
        QPushButton* millimeters = dialog.addButton(QStringLiteral("毫米 (mm)"), QMessageBox::AcceptRole);
        QPushButton* meters = dialog.addButton(QStringLiteral("米 (m)"), QMessageBox::AcceptRole);
        dialog.addButton(QMessageBox::Cancel);
        dialog.setDefaultButton(millimeters);
        dialog.exec();
        if(dialog.clickedButton() == millimeters) {
            *scaleToMeters = 0.001;
            return true;
        }
        if(dialog.clickedButton() == meters) {
            *scaleToMeters = 1.0;
            return true;
        }
        return false;
    }

    smrobot::visualization::CustomMeshResult submitLineOverlay(
        robot_qt_viewer::IRobotQtViewerVisualizationViewportPort* viewport,
        const QString& overlayId,
        const smrobot::workbench::freeform::FreeformTrajPlanLinePreview& preview,
        double coordinateScale)
    {
        if(viewport == nullptr) {
            return smrobot::visualization::CustomMeshResult::fail(
                smrobot::visualization::CustomMeshError::GraphicsContextUnavailable,
                "Visualization viewport is unavailable.");
        }
        smrobot::visualization::CustomMeshDesc overlay = makeOverlayDescriptor(overlayId);
        overlay.appearance.renderOrder = overlayId == QStringLiteral("referenceLines") ? 30 : 20;
        for(const auto& segment : preview) {
            appendLine(overlay.mesh,
                { static_cast<float>(segment.startX * coordinateScale), static_cast<float>(segment.startY * coordinateScale),
                    static_cast<float>(segment.startZ * coordinateScale) },
                { static_cast<float>(segment.endX * coordinateScale), static_cast<float>(segment.endY * coordinateScale),
                    static_cast<float>(segment.endZ * coordinateScale) },
                { static_cast<float>(segment.red), static_cast<float>(segment.green),
                    static_cast<float>(segment.blue), static_cast<float>(segment.alpha) },
                overlayId == QStringLiteral("referenceLines") ? 0.0025f : 0.0015f);
        }
        smrobot::visualization::CustomMeshHandle handle;
        return viewport->upsertCustomMesh(overlay, handle);
    }

    void submitSurfaceOverlay(
        robot_qt_viewer::IRobotQtViewerVisualizationViewportPort* viewport,
        const QString& overlayId,
        const smrobot::workbench::freeform::FreeformTrajPlanMeshPreview& preview,
        double coordinateScale)
    {
        if(viewport == nullptr || preview.vertices.empty() || preview.indices.empty()) {
            return;
        }
        smrobot::visualization::CustomMeshDesc overlay = makeOverlayDescriptor(overlayId);
        if(overlayId == QStringLiteral("sliceSurfaces")) {
            overlay.appearance.blend = smrobot::visualization::MeshBlendMode::Opaque;
        }
        overlay.appearance.renderOrder = 10;
        overlay.mesh.positions.reserve(preview.vertices.size());
        overlay.mesh.colors.reserve(preview.vertices.size());
        overlay.mesh.indices = preview.indices;
        for(const auto& vertex : preview.vertices) {
            overlay.mesh.positions.push_back({ static_cast<float>(vertex.positionX * coordinateScale),
                static_cast<float>(vertex.positionY * coordinateScale),
                static_cast<float>(vertex.positionZ * coordinateScale) });
            overlay.mesh.colors.push_back({ static_cast<float>(vertex.red), static_cast<float>(vertex.green),
                static_cast<float>(vertex.blue), static_cast<float>(vertex.alpha) });
        }
        smrobot::visualization::CustomMeshHandle handle;
        viewport->upsertCustomMesh(overlay, handle);
    }
    void submitSliceSurfaces(robot_qt_viewer::IRobotQtViewerVisualizationViewportPort* viewport,
        const spray_freeform_traj_plan::FreeformSlicingResult& slicing)
    {
        if(viewport == nullptr) return;
        static const smrobot::visualization::Color4f colorTable[] = {
            { 1.0f, 0.2f, 0.2f, 0.7f }, { 0.2f, 1.0f, 0.2f, 0.7f }, { 1.0f, 0.2f, 1.0f, 0.7f },
            { 0.2f, 1.0f, 1.0f, 0.7f }, { 1.0f, 0.6f, 0.2f, 0.7f }, { 0.6f, 0.2f, 1.0f, 0.7f },
            { 0.2f, 0.6f, 1.0f, 0.7f }, { 0.2f, 0.8f, 1.0f, 0.7f }, { 1.0f, 0.2f, 0.8f, 0.7f },
            { 1.0f, 0.2f, 0.6f, 0.7f }, { 0.6f, 1.0f, 0.8f, 0.7f }, { 0.8f, 0.6f, 1.0f, 0.7f }
        };
        smrobot::visualization::CustomMeshDesc overlay = makeOverlayDescriptor(QStringLiteral("sliceSurfaces"));
        overlay.appearance.renderOrder = 10;
        for(std::size_t layer = 0; layer < slicing.planes.size(); ++layer) {
            const auto& plane = slicing.planes[layer];
            const double normalLength = std::sqrt(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y +
                plane.normal.z * plane.normal.z);
            if(normalLength <= 1.0e-9) continue;
            const spray_freeform_traj_plan::FreeformVec3 normal{
                plane.normal.x / normalLength, plane.normal.y / normalLength, plane.normal.z / normalLength };
            const spray_freeform_traj_plan::FreeformVec3 reference = std::abs(normal.z) < 0.9
                ? spray_freeform_traj_plan::FreeformVec3{ 0.0, 0.0, 1.0 }
                : spray_freeform_traj_plan::FreeformVec3{ 1.0, 0.0, 0.0 };
            const auto cross = [](const auto& left, const auto& right) {
                return spray_freeform_traj_plan::FreeformVec3{
                    left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
                    left.x * right.y - left.y * right.x };
            };
            const auto dot = [](const auto& left, const auto& right) {
                return left.x * right.x + left.y * right.y + left.z * right.z;
            };
            spray_freeform_traj_plan::FreeformVec3 localX = cross(normal, reference);
            const double localXLength = std::sqrt(dot(localX, localX));
            if(localXLength <= 1.0e-9) continue;
            localX.x /= localXLength; localX.y /= localXLength; localX.z /= localXLength;
            const spray_freeform_traj_plan::FreeformVec3 localY = cross(normal, localX);
            double minLocalX = std::numeric_limits<double>::max();
            double maxLocalX = std::numeric_limits<double>::lowest();
            double minLocalY = std::numeric_limits<double>::max();
            double maxLocalY = std::numeric_limits<double>::lowest();
            bool hasContour = false;
            for(const auto& contour : slicing.contours) {
                if(contour.layerIndex != layer) continue;
                for(const auto& point : contour.points) {
                    const spray_freeform_traj_plan::FreeformVec3 relative{
                        point.x - plane.center.x, point.y - plane.center.y, point.z - plane.center.z };
                    const double x = dot(relative, localX);
                    const double y = dot(relative, localY);
                    minLocalX = std::min(minLocalX, x); maxLocalX = std::max(maxLocalX, x);
                    minLocalY = std::min(minLocalY, y); maxLocalY = std::max(maxLocalY, y);
                    hasContour = true;
                }
            }
            if(!hasContour) continue;
            const double xPadding = (maxLocalX - minLocalX) * 0.1;
            const double yPadding = (maxLocalY - minLocalY) * 0.1;
            minLocalX -= xPadding; maxLocalX += xPadding;
            minLocalY -= yPadding; maxLocalY += yPadding;
            const auto makePoint = [&plane, &localX, &localY](double x, double y) {
                return smrobot::visualization::Vec3f{
                    static_cast<float>(plane.center.x + localX.x * x + localY.x * y),
                    static_cast<float>(plane.center.y + localX.y * x + localY.y * y),
                    static_cast<float>(plane.center.z + localX.z * x + localY.z * y) };
            };
            const bool selected = slicing.selectedLayerIndex == static_cast<int>(layer);
            const smrobot::visualization::Color4f color = selected
                ? smrobot::visualization::Color4f{ 1.0f, 1.0f, 0.2f, 0.7f }
                : colorTable[layer % (sizeof(colorTable) / sizeof(colorTable[0]))];
            const auto first = makePoint(minLocalX, minLocalY);
            const auto second = makePoint(maxLocalX, minLocalY);
            const auto third = makePoint(maxLocalX, maxLocalY);
            const auto fourth = makePoint(minLocalX, maxLocalY);
            const std::uint32_t base = static_cast<std::uint32_t>(overlay.mesh.positions.size());
            overlay.mesh.positions.insert(overlay.mesh.positions.end(), { first, second, third, fourth });
            overlay.mesh.colors.insert(overlay.mesh.colors.end(), 4, color);
            overlay.mesh.indices.insert(overlay.mesh.indices.end(),
                { base, base + 1, base + 2, base, base + 2, base + 3 });
        }
        smrobot::visualization::CustomMeshHandle handle;
        viewport->upsertCustomMesh(overlay, handle);
    }

    Eigen::Vector3d transformPoint(const Eigen::Matrix4d& transform, const Eigen::Vector3d& point)
    {
        return (transform * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0)).head<3>();
    }

    Eigen::Vector3d transformDirection(const Eigen::Matrix4d& transform, const Eigen::Vector3d& direction)
    {
        const Eigen::Vector3d transformed = transform.topLeftCorner<3, 3>() * direction;
        return transformed.squaredNorm() > 1.0e-16 ? transformed.normalized() : transformed;
    }

    void transformBoundaryLine(ModelTrajPlan::BoundaryLine& line, const Eigen::Matrix4d& transform)
    {
        for(ModelTrajPlan::BasicPoint& point : line.points) point.position = transformPoint(transform, point.position);
        line.center = transformPoint(transform, line.center);
        line.normal = transformDirection(transform, line.normal);
    }

    void transformLinePoints(ModelTrajPlan::Epiphyseal& lines, const Eigen::Matrix4d& transform)
    {
        for(auto& line : lines) for(ModelTrajPlan::LinePoint& point : line) {
            point.position = transformPoint(transform, point.position);
            point.normal = transformDirection(transform, point.normal);
            point.tangent = transformDirection(transform, point.tangent);
        }
    }

    bool matrixFromJson(const QJsonObject& object, Eigen::Matrix4d& matrix, QString& error)
    {
        const QJsonValue value = object.value(QStringLiteral("transform_matrix"));
        if(!value.isArray() || value.toArray().size() != 4) {
            error = QStringLiteral("JSON 中的 transform_matrix 必须是 4x4 数组。");
            return false;
        }
        const QJsonArray rows = value.toArray();
        for(int row = 0; row < 4; ++row) {
            if(!rows[row].isArray() || rows[row].toArray().size() != 4) {
                error = QStringLiteral("JSON 中的 transform_matrix 必须是 4x4 数组。");
                return false;
            }
            const QJsonArray columns = rows[row].toArray();
            for(int column = 0; column < 4; ++column) {
                if(!columns[column].isDouble()) {
                    error = QStringLiteral("transform_matrix 只能包含数值。");
                    return false;
                }
                matrix(row, column) = columns[column].toDouble();
            }
        }
        if(!matrix.allFinite() || !matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1.0e-9)) {
            error = QStringLiteral("变换矩阵必须是最后一行 [0, 0, 0, 1] 的仿射矩阵。");
            return false;
        }
        return true;
    }

    QJsonArray matrixToJson(const Eigen::Matrix4d& matrix)
    {
        QJsonArray rows;
        for(int row = 0; row < 4; ++row) {
            QJsonArray columns;
            for(int column = 0; column < 4; ++column) columns.append(matrix(row, column));
            rows.append(columns);
        }
        return rows;
    }
}

namespace smrobot::workbench::freeform
{
    FreeformTrajPlanWorkbenchController::FreeformTrajPlanWorkbenchController(
        FreeformTrajPlanWorkflowPanel& workflowPanel, FreeformTrajPlanParameterPanel& parameterPanel,
        robot_qt_viewer::RobotQtViewerDocumentContext& context,
        FreeformTrajPlanPreviewUpdatedCallback previewUpdated,
        FreeformTrajPlanPreviewClearedCallback previewCleared, QObject* parent)
        : QObject(parent), m_workflowPanel(workflowPanel), m_parameterPanel(parameterPanel), m_context(context),
          m_previewUpdated(std::move(previewUpdated)), m_previewCleared(std::move(previewCleared))
    {
        QSettings settings;
        const int normal = settings.value("freeform/normalAxis", 4).toInt();
        const int forward = settings.value("freeform/forwardAxis", 0).toInt();
        if(ModelTrajPlan::validTrajectoryAxes(normal, forward)) {
            m_generalData.trajData_.generationParams_.normalAxis = normal;
            m_generalData.trajData_.generationParams_.forwardAxis = forward;
        }
        connect(&workflowPanel, &FreeformTrajPlanWorkflowPanel::importModelRequested,
            this, &FreeformTrajPlanWorkbenchController::importModel);
        connect(&workflowPanel, &FreeformTrajPlanWorkflowPanel::planningMethodRequested,
            this, &FreeformTrajPlanWorkbenchController::setPlanningMethod);
        connect(&workflowPanel, &FreeformTrajPlanWorkflowPanel::slicingMethodRequested,
            this, &FreeformTrajPlanWorkbenchController::setSlicingMethod);
        connect(&workflowPanel, &FreeformTrajPlanWorkflowPanel::exportTrajectoryRequested,
            this, &FreeformTrajPlanWorkbenchController::exportTrajectory);
        connect(&workflowPanel, &FreeformTrajPlanWorkflowPanel::workflowActionRequested,
            this, &FreeformTrajPlanWorkbenchController::requestWorkflowAction);
        connect(&parameterPanel, &FreeformTrajPlanParameterPanel::parametersChanged,
            this, &FreeformTrajPlanWorkbenchController::setParameters);
        connect(&parameterPanel, &FreeformTrajPlanParameterPanel::uvParametersChanged,
            this, &FreeformTrajPlanWorkbenchController::setUvParameters);
        connect(&parameterPanel, &FreeformTrajPlanParameterPanel::visualizationVisibilityChanged,
            this, &FreeformTrajPlanWorkbenchController::setVisualizationVisibility);
        connect(&parameterPanel, &FreeformTrajPlanParameterPanel::tcpSettingsChanged,
            this, &FreeformTrajPlanWorkbenchController::setTcpSettings);
        connect(&parameterPanel, &FreeformTrajPlanParameterPanel::workflowActionRequested,
            this, &FreeformTrajPlanWorkbenchController::requestWorkflowAction);
    }
    void FreeformTrajPlanWorkbenchController::handleEvent(const robot_qt_viewer::RobotQtViewerEvent& event)
    {
        if(event.kind == robot_qt_viewer::RobotQtViewerEventKind::ProjectOpened) {
            if(auto* dialog = m_workflowPanel.findChild<QDialog*>(QStringLiteral("trajectoryFrameDialog"))) dialog->reject();
            if(!m_modelObjectId.isEmpty()) {
                if(auto* viewport = m_context.visualizationViewport()) {
                    viewport->clearSceneObjectVisibilityOverride(m_modelObjectId);
                }
            }
            m_plan.clear(m_generalData);
            m_modelObjectId.clear();
            clearAllOverlays(m_context.visualizationViewport());
            if(m_previewCleared) {
                m_previewCleared();
            }
            m_workflowPanel.setModelPath(QString());
            m_workflowPanel.setPlanningMethod(spray_freeform_traj_plan::PlanningMethod::Slicing);
            m_parameterPanel.setPlanningMethod(spray_freeform_traj_plan::PlanningMethod::Slicing);
            m_workflowPanel.setStatus(QStringLiteral("等待导入曲面模型。"), true);
        }
    }
    void FreeformTrajPlanWorkbenchController::setPlanningMethod(bool uv)
    {
        const auto method = uv ? spray_freeform_traj_plan::PlanningMethod::Uv
                               : spray_freeform_traj_plan::PlanningMethod::Slicing;
        m_uvPlanning = uv;
        m_workflowPanel.setPlanningMethod(method);
        m_parameterPanel.setPlanningMethod(method);
        showResult(true, uv ? QStringLiteral("UV 轨迹规划已启用。")
                            : QStringLiteral("切片轨迹规划已启用。"));
    }
    void FreeformTrajPlanWorkbenchController::importModel(const QString& path)
    {
        if(auto* dialog = m_workflowPanel.findChild<QDialog*>(QStringLiteral("trajectoryFrameDialog"))) dialog->reject();
        if(path.isEmpty()) {
            showResult(false, QStringLiteral("请先选择曲面模型。"));
            return;
        }

        double scaleToMeters = 1.0;
        if(!selectModelUnitScale(&m_workflowPanel, path, &scaleToMeters)) {
            return;
        }
        const QString previousObjectId = m_modelObjectId;
        ModelTrajPlan::GeneralData importedData;
        ModelTrajPlan::STLProcessor stlProcessor;
        if(!stlProcessor.preprocessSTL(path.toStdString(), importedData.modelDataManager_)) {
            showResult(false, QStringLiteral("无法读取曲面模型。"));
            return;
        }
        // BurTrajPlan and its UI parameters use the STL's native millimetre
        // coordinates. SceneObjectDesc owns the independent metres conversion
        // for the platform viewport below; never apply that conversion here.
        importedData.filePaths_ = { path.toStdString() };
        importedData.processingState_.isSTLProcessed = true;
        importedData.currentState_ = ModelTrajPlan::DataState::Loaded;

        robot_qt_viewer::SceneEntityWorkflowController sceneWorkflow(m_context);
        const auto importResult = sceneWorkflow.importSceneObjectFromPath(
            std::filesystem::path(path.toStdWString()), "workpiece");
        if(!importResult.success) {
            showResult(false, importResult.message);
            return;
        }

        // SceneObjectDesc defaults to scale 1.0. Make its displayed and collision
        // geometry use the exact same unit conversion as the Document mesh.
        const robot_qt_viewer::ProjectMutationResult scaleResult =
            m_context.documentController().mutateProject(
                QStringLiteral("freeformTrajPlan.setModelUnitScale"),
                robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                [&](simulation_project::ProjectDocumentService& service,
                    bool& changed,
                    std::string& error) {
                    simulation_project::SceneObjectDesc* object =
                        service.findSceneObject(importResult.entityId.toStdString());
                    if(object == nullptr) {
                        error = "Imported workpiece was not found in the project document.";
                        return false;
                    }

                    changed = object->visualScale != scaleToMeters ||
                        object->collisionScale != scaleToMeters;
                    object->visualScale = scaleToMeters;
                    object->collisionScale = scaleToMeters;
                    return true;
                });

        if(!scaleResult.success) {
            sceneWorkflow.restoreImportState(importResult);
            robot_qt_viewer::ViewportReloadWorkflowController(m_context).reload(
                QStringLiteral("freeformTrajPlan.restoreFailedModelImport"));
            showResult(false, scaleResult.message);
            return;
        }

        // Replace the previous workpiece only after the new model and its scale
        // have both been stored successfully.
        if(!previousObjectId.isEmpty()) {
            if(auto* viewport = m_context.visualizationViewport()) {
                viewport->clearSceneObjectVisibilityOverride(previousObjectId);
            }
            const auto deleteResult = sceneWorkflow.deleteEntity(
                robot_qt_viewer::SceneEntityKind::Object, previousObjectId);
            if(!deleteResult.success) {
                sceneWorkflow.restoreImportState(importResult);
                robot_qt_viewer::ViewportReloadWorkflowController(m_context).reload(
                    QStringLiteral("freeformTrajPlan.restoreFailedModelReplacement"));
                showResult(false, deleteResult.message);
                return;
            }
        }

        robot_qt_viewer::ViewportReloadWorkflowController reloadWorkflow(m_context);
        const auto reloadResult = reloadWorkflow.reload(
            QStringLiteral("freeformTrajPlan.importModel"));
        if(!reloadResult.success) {
            sceneWorkflow.restoreImportState(importResult);
            reloadWorkflow.reload(QStringLiteral("freeformTrajPlan.restoreFailedModelImport"));
            showResult(false, reloadResult.errorMessage);
            return;
        }

        m_modelObjectId = importResult.entityId;
        if(!m_modelVisible) {
            if(auto* viewport = m_context.visualizationViewport()) {
                viewport->setSceneObjectVisibilityOverride(m_modelObjectId, false);
            }
        }
        m_modelPath = path;
        m_algorithmToSceneScale = scaleToMeters;
        // Import creates a fresh algorithm document. Preserve values owned by
        // the persistent parameter panel; otherwise its displayed settings
        // (notably Slice Count Limit) disagree with the new document.
        const auto managedParameters = m_generalData.slicingData_.managedParams_;
        const auto generationParameters = m_generalData.trajData_.generationParams_;
        const double chordErrorTolerance = m_generalData.boundaryData_.epsilon_;
        const double minimumSpacing = m_generalData.boundaryData_.minStep_;
        m_plan.clear(m_generalData);
        m_generalData = std::move(importedData);
        m_generalData.slicingData_.managedParams_ = managedParameters;
        m_generalData.trajData_.generationParams_ = generationParameters;
        m_generalData.boundaryData_.epsilon_ = chordErrorTolerance;
        m_generalData.boundaryData_.minStep_ = minimumSpacing;
        m_workflowPanel.setModelPath(path);

        // New model invalidates all temporary slice, trajectory, and connection overlays.
        clearAllOverlays(m_context.visualizationViewport());
        if(m_previewCleared) {
            m_previewCleared();
        }

        m_context.selectionModel().selectSceneObject(
            m_modelObjectId, QStringLiteral("freeformTrajPlan.importModel"));
        if(auto* selectionViewport = m_context.selectionViewport()) {
            selectionViewport->selectSceneObject(m_modelObjectId);
        }

        showResult(true, QStringLiteral("曲面模型已导入并显示：%1").arg(m_modelObjectId));
    }
        
    void FreeformTrajPlanWorkbenchController::setSlicingMethod(bool adaptive)
    {
        m_generalData.slicingData_.managedParams_.mode = adaptive
            ? ModelTrajPlan::SlicingMode::AA : ModelTrajPlan::SlicingMode::Parallel;
        showResult(true, adaptive ? QStringLiteral("自适应切片参数已启用。")
                                  : QStringLiteral("平行切片参数已启用。"));
    }
    void FreeformTrajPlanWorkbenchController::exportTrajectory(
        const QString& path,
        FreeformTrajPlanExportFormat format)
    {
        switch(format) {
        case FreeformTrajPlanExportFormat::Fanuc:
            if(m_uvPlanning) {
                showResult(false, QStringLiteral("FANUC 程序导出当前仅支持切片轨迹。"));
                return;
            }
            {
                ModelTrajPlan::FanucTcpDefinition tcp;
                if(m_tcpSettings.enabled && m_tcpSettings.format == FreeformTrajPlanTcpFormat::Fanuc) {
                    tcp.enabled = true;
                    tcp.x = m_tcpSettings.x;
                    tcp.y = m_tcpSettings.y;
                    tcp.z = m_tcpSettings.z;
                    tcp.w = m_tcpSettings.w;
                    tcp.p = m_tcpSettings.p;
                    tcp.r = m_tcpSettings.r;
                }
                showResult(ModelTrajPlan::TrajectoryExporter::exportFANUCProgram(
                               m_generalData.trajData_.trajectoryPoints_, path.toStdString(), tcp),
                    QStringLiteral("FANUC 程序导出失败。"));
            }
            return;
        case FreeformTrajPlanExportFormat::Abb:
            if(m_uvPlanning) {
                showResult(false, QStringLiteral("ABB 程序导出当前仅支持切片轨迹。"));
                return;
            }
            {
                ModelTrajPlan::AbbTcpDefinition tcp;
                if(m_tcpSettings.enabled && m_tcpSettings.format == FreeformTrajPlanTcpFormat::Abb) {
                    tcp.enabled = true;
                    tcp.x = m_tcpSettings.x;
                    tcp.y = m_tcpSettings.y;
                    tcp.z = m_tcpSettings.z;
                    tcp.q1 = m_tcpSettings.q1;
                    tcp.q2 = m_tcpSettings.q2;
                    tcp.q3 = m_tcpSettings.q3;
                    tcp.q4 = m_tcpSettings.q4;
                }
                showResult(ModelTrajPlan::TrajectoryExporter::exportABBProgram(
                               m_generalData.trajData_.trajectoryPoints_, path.toStdString(),
                               path.toStdString(), tcp),
                    QStringLiteral("ABB 程序导出失败。"));
            }
            return;
        case FreeformTrajPlanExportFormat::Paq:
            showResult(m_uvPlanning
                    ? m_uvPipeline.exportPaq(path.toStdString(), uvTrajectoryParameters())
                    : m_plan.exportTrajectoryToFile(m_generalData, path.toStdString()),
                QStringLiteral("PAQ 轨迹导出失败。"));
            return;
        }
    }
    void FreeformTrajPlanWorkbenchController::setParameters(double trajectorySpacingMm,
        double chordErrorTolerance, double minimumSpacingMm, double largeEndOffsetMm,
        double smallEndOffsetMm, double optimalSprayDistanceMm, double optimalSpeedMmPerSecond,
        int trajectoryCount, int maxSliceCount)
    {
        m_generalData.slicingData_.managedParams_.trajectorySpacing = trajectorySpacingMm;
        m_generalData.slicingData_.managedParams_.numTrajectories = trajectoryCount;
        m_generalData.slicingData_.managedParams_.maxSliceCount = maxSliceCount;
        m_generalData.boundaryData_.epsilon_ = chordErrorTolerance;
        m_generalData.boundaryData_.minStep_ = minimumSpacingMm;
        m_generalData.trajData_.generationParams_.z1Offset = largeEndOffsetMm;
        m_generalData.trajData_.generationParams_.z2Offset = smallEndOffsetMm;
        m_generalData.trajData_.generationParams_.offsetDistance = optimalSprayDistanceMm;
        m_generalData.trajData_.generationParams_.normalVelocity = optimalSpeedMmPerSecond;
    }
    void FreeformTrajPlanWorkbenchController::setUvParameters(double targetSpacingMm,
        double trajectorySpeedMmPerSecond, double sprayDistanceMm,
        double maximumSeamSprayDistanceMm, double maximumAccelerationMmPerSecondSquared,
        double maximumJerkMmPerSecondCubed, double overtravelLength,
        double arcSpeedLossPercent, bool showUvSplitView)
    {
        m_uvParameters.target_spacing = targetSpacingMm;
        m_uvParameters.trajectory_speed = trajectorySpeedMmPerSecond;
        m_uvParameters.spray_distance = sprayDistanceMm;
        m_uvParameters.max_seam_spray_distance = maximumSeamSprayDistanceMm;
        m_uvParameters.a_max = maximumAccelerationMmPerSecondSquared;
        m_uvParameters.j_max = maximumJerkMmPerSecondCubed;
        m_uvParameters.arc_lock_guard_d = overtravelLength;
        m_uvParameters.arc_speed_loss_pct = arcSpeedLossPercent;
        Q_UNUSED(showUvSplitView);
    }

    void FreeformTrajPlanWorkbenchController::setTcpSettings(
        const FreeformTrajPlanTcpSettings& settings)
    {
        m_tcpSettings = settings;
    }

    void FreeformTrajPlanWorkbenchController::setVisualizationVisibility(
        bool modelVisible, bool slicesVisible, bool trajectoriesVisible, bool connectionsVisible,
        bool referenceLinesVisible)
    {
        const bool modelVisibilityChanged = m_modelVisible != modelVisible;
        m_modelVisible = modelVisible;
        m_slicesVisible = slicesVisible;
        m_trajectoriesVisible = trajectoriesVisible;
        m_connectionsVisible = connectionsVisible;
        m_referenceLinesVisible = referenceLinesVisible;

        if(modelVisibilityChanged && !m_modelObjectId.isEmpty()) {
            auto* viewport = m_context.visualizationViewport();
            if(viewport == nullptr ||
                !viewport->setSceneObjectVisibilityOverride(m_modelObjectId, m_modelVisible)) {
                showResult(false, QStringLiteral("模型可见性更新失败。"));
                return;
            }
        }

        refreshSlicingOverlays();
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("trajectoryFrames"));
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("uvTrajectory"));
        if(m_trajectoriesVisible) {
            if(m_uvPlanning) {
                refreshUvTrajectoryOverlay();
            } else if(!m_generalData.trajData_.trajectoryPoints_.empty()) {
                submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("trajectoryFrames"),
                    m_renderer.makeTrajectoryFramePreview(m_generalData.trajData_.trajectoryPoints_),
                    m_algorithmToSceneScale);
            }
        }
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("connections"));
        if(m_connectionsVisible && !m_generalData.trajData_.connectionData_.connectionSegments_.empty()) {
            submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("connections"),
                m_renderer.makeConnectionPreview(m_generalData.trajData_.connectionData_.connectionSegments_),
                m_algorithmToSceneScale);
        }
    }
    void FreeformTrajPlanWorkbenchController::requestWorkflowAction(
        spray_freeform_traj_plan::WorkflowAction action)
    {
        // A data-changing workflow invalidates the dialog's sampled preview.
        // Camera interaction does not enter this path and stays available.
        if(action != spray_freeform_traj_plan::WorkflowAction::ConfigureTrajectoryFrame) {
            if(auto* dialog = m_workflowPanel.findChild<QDialog*>(QStringLiteral("trajectoryFrameDialog"))) dialog->reject();
        }
        const bool clearsAllData =
            action == spray_freeform_traj_plan::WorkflowAction::ClearAllData;
        const bool clearsModelData =
            action == spray_freeform_traj_plan::WorkflowAction::ClearModelData || clearsAllData;
        if(clearsModelData && !m_modelObjectId.isEmpty()) {
            if(auto* viewport = m_context.visualizationViewport()) {
                viewport->clearSceneObjectVisibilityOverride(m_modelObjectId);
            }
            robot_qt_viewer::SceneEntityWorkflowController sceneWorkflow(m_context);
            const auto deleteResult = sceneWorkflow.deleteEntity(
                robot_qt_viewer::SceneEntityKind::Object, m_modelObjectId);
            if(!deleteResult.success) {
                showResult(false, deleteResult.message);
                return;
            }
            robot_qt_viewer::ViewportReloadWorkflowController reloadWorkflow(m_context);
            const auto reloadResult = reloadWorkflow.reload(QStringLiteral("freeformTrajPlan.clearModel"));
            if(!reloadResult.success) {
                showResult(false, reloadResult.errorMessage);
                return;
            }
            m_modelObjectId.clear();
        }
        bool success = true;
        QString message;
        switch(action) {
        case spray_freeform_traj_plan::WorkflowAction::StartSlicing:
            success = m_plan.generateCuttingPlanes(m_generalData);
            if(success) {
                const QString method = m_generalData.slicingData_.managedParams_.mode == ModelTrajPlan::SlicingMode::AA
                    ? QStringLiteral("自适应") : QStringLiteral("等距");
                message = QStringLiteral("%1切片已完成：生成 %2 个切片层。").arg(
                    method).arg(m_generalData.slicingData_.cuttingPlanes_.size());
            } else {
                message = QStringLiteral("切片失败：模型顶点=%1，边界=%2，最小步距=%3 mm。")
                    .arg(m_generalData.modelDataManager_.mainModelData_.m_vertices.size() / 3)
                    .arg(m_generalData.processingState_.isBoundaryDetected ? QStringLiteral("已识别") : QStringLiteral("未识别"))
                    .arg(m_generalData.boundaryData_.minStep_);
            }
            break;
        case spray_freeform_traj_plan::WorkflowAction::GenerateTrajectory:
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"));
            success = m_plan.generateMATLABTrajectory(m_generalData,
                m_generalData.slicingData_.managedParams_.numTrajectories,
                m_generalData.trajData_.generationParams_.z1Offset,
                m_generalData.trajData_.generationParams_.z2Offset,
                m_generalData.trajData_.generationParams_.offsetDistance);
            message = success
                ? QStringLiteral("轨迹已生成：%1 个轨迹点。").arg(m_generalData.trajData_.trajectoryPoints_.size())
                : QStringLiteral("轨迹生成失败：请先完成切片，并确认至少有一个有效切片轮廓。");
            break;
        case spray_freeform_traj_plan::WorkflowAction::ConnectTrajectory:
            success = m_plan.connectTrajectoryPoints(m_generalData, ModelTrajPlan::ConnectionStrategy::BowPath);
            message = success ? QStringLiteral("轨迹连线已生成。") : QStringLiteral("轨迹连线生成失败。");
            break;
        case spray_freeform_traj_plan::WorkflowAction::ExportTrajectory:
            success = false; message = QStringLiteral("请选择导出文件位置。"); break;
        case spray_freeform_traj_plan::WorkflowAction::StartUvMap:
            success = !m_modelPath.isEmpty() && m_uvPipeline.startUvMap(m_modelPath.toStdString(), uvTrajectoryParameters());
            message = success ? QStringLiteral("UV 展开已完成。") : QStringLiteral("UV 展开失败：请确认模型与 OptCuts 运行环境。 ");
            break;
        case spray_freeform_traj_plan::WorkflowAction::GenerateUvTrajectory:
            success = m_uvPipeline.startGetTraj(uvTrajectoryParameters());
            if(success) refreshUvTrajectoryOverlay();
            message = success ? QStringLiteral("UV 轨迹已生成。") : QStringLiteral("UV 轨迹生成失败：请先完成 UV 展开。 ");
            break;
        case spray_freeform_traj_plan::WorkflowAction::OptimizeUv:
            success = m_uvPipeline.startOptimize(uvTrajectoryParameters());
            if(success) refreshUvTrajectoryOverlay();
            message = success ? QStringLiteral("UV 轨迹优化已完成。") : QStringLiteral("UV 轨迹优化失败：请先生成 UV 轨迹。 ");
            break;
        case spray_freeform_traj_plan::WorkflowAction::AdjustSlicing:
            showSlicingAdjustDialog(); return;
        case spray_freeform_traj_plan::WorkflowAction::TransformTrajectory:
            showTrajectoryTransformDialog(); return;
        case spray_freeform_traj_plan::WorkflowAction::ConfigureTrajectoryFrame:
            showTrajectoryFrameDialog(); return;
        case spray_freeform_traj_plan::WorkflowAction::ClearModelData:
            m_plan.clear(m_generalData); m_uvPipeline.clearModel(); m_modelPath.clear();
            message = QStringLiteral("模型数据已清除。"); break;
        case spray_freeform_traj_plan::WorkflowAction::ClearTrajectoryData:
            m_generalData.trajData_.trajectoryPoints_.clear();
            m_generalData.trajData_.connectionData_ = {};
            m_generalData.processingState_.isTrajectoryGenerated = false;
            m_uvPipeline.clearTrajectory();
            message = QStringLiteral("轨迹数据已清除。"); break;
        case spray_freeform_traj_plan::WorkflowAction::ClearConnectionData:
            m_generalData.trajData_.connectionData_ = {}; m_uvPipeline.clearConnection();
            message = QStringLiteral("轨迹连线已清除。"); break;
        case spray_freeform_traj_plan::WorkflowAction::ClearOtherData:
            m_plan.clearCuttingPlanes(m_generalData); message = QStringLiteral("切片数据已清除。"); break;
        case spray_freeform_traj_plan::WorkflowAction::ClearAllData:
            m_plan.clear(m_generalData);
            m_uvPipeline.clearModel();
            m_modelPath.clear();
            message = QStringLiteral("全部规划数据已清除。");
            break;
        default:
            success = false; message = QStringLiteral("该操作尚未接入原有算法。"); break;
        }
        if(clearsModelData && success) {
            m_workflowPanel.setModelPath(QString());
        }
        if(action == spray_freeform_traj_plan::WorkflowAction::ClearTrajectoryData && success &&
            m_previewCleared) {
            m_previewCleared();
        }
        if(success && action == spray_freeform_traj_plan::WorkflowAction::StartSlicing) {
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"));
            if(m_previewCleared) {
                m_previewCleared();
            }
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("connections"));
            refreshSlicingOverlays();
        }
        if(success && action == spray_freeform_traj_plan::WorkflowAction::ConnectTrajectory) {
            if(m_connectionsVisible) {
                submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("connections"),
                    m_renderer.makeConnectionPreview(m_generalData.trajData_.connectionData_.connectionSegments_),
                    m_algorithmToSceneScale);
            }
        }
        if(success && (clearsModelData ||
                              action == spray_freeform_traj_plan::WorkflowAction::ClearOtherData)) {
            clearAllOverlays(m_context.visualizationViewport());
        }
        if(success && action == spray_freeform_traj_plan::WorkflowAction::ClearConnectionData) {
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("connections"));
        }
        if(success && action == spray_freeform_traj_plan::WorkflowAction::ClearTrajectoryData) {
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"));
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("trajectoryFrames"));
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("uvTrajectory"));
        }
        if(success && action == spray_freeform_traj_plan::WorkflowAction::GenerateTrajectory) {
            if(m_previewUpdated) {
                m_previewUpdated(m_renderer.makePreview(m_generalData.trajData_.trajectoryPoints_));
            }
            if(m_trajectoriesVisible) {
                submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("trajectoryFrames"),
                    m_renderer.makeTrajectoryFramePreview(m_generalData.trajData_.trajectoryPoints_),
                    m_algorithmToSceneScale);
            }
        }
        showResult(success, message);
    }

    void FreeformTrajPlanWorkbenchController::showTrajectoryFrameDialog()
    {
        if(auto* existing = m_workflowPanel.findChild<QDialog*>(QStringLiteral("trajectoryFrameDialog"))) {
            existing->show();
            existing->raise();
            existing->activateWindow();
            return;
        }
        auto* dialog = new QDialog(&m_workflowPanel);
        dialog->setObjectName(QStringLiteral("trajectoryFrameDialog"));
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowModality(Qt::NonModal);
        dialog->setWindowTitle(QStringLiteral("轨迹姿态配置"));
        dialog->setMinimumWidth(440);
        auto* layout = new QVBoxLayout(dialog);
        auto* form = new QFormLayout();
        auto* normal = new QComboBox(dialog);
        auto* forward = new QComboBox(dialog);
        const QStringList axes = { "X+", "X-", "Y+", "Y-", "Z+", "Z-" };
        normal->addItems(axes); forward->addItems(axes);
        const auto previous = m_generalData.trajData_.generationParams_;
        normal->setCurrentIndex(previous.normalAxis);
        forward->setCurrentIndex(previous.forwardAxis);
        form->addRow(QStringLiteral("喷涂法向轴"), normal);
        form->addRow(QStringLiteral("前进参考轴（回程不翻转）"), forward);
        layout->addLayout(form);
        auto* explanation = new QLabel(QStringLiteral("正向沿当前法向/前进参考方向，负向相反。\nX 红、Y 绿、Z 蓝；第三轴自动按右手系确定。"), dialog);
        explanation->setWordWrap(true); layout->addWidget(explanation);
        auto* status = new QLabel(dialog); status->setWordWrap(true); layout->addWidget(status);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, dialog);
        layout->addWidget(buttons);

        std::vector<ModelTrajPlan::TrajectoryPoint> first;
        if(!m_generalData.trajData_.trajectoryPoints_.empty()) {
            first.push_back(m_generalData.trajData_.trajectoryPoints_.front());
        } else if(!m_generalData.slicingData_.currentSliceResult_.internalBoundaries.empty()) {
            ModelTrajPlan::GeneralData draft = m_generalData;
            draft.trajData_.generationParams_.previewFirstPoint = true;
            ModelTrajPlan::BurTrajPlan previewPlan;
            if(previewPlan.generateMATLABTrajectory(draft, draft.slicingData_.managedParams_.numTrajectories,
                previous.z1Offset, previous.z2Offset, previous.offsetDistance) &&
                !draft.trajData_.trajectoryPoints_.empty()) {
                first.push_back(draft.trajData_.trajectoryPoints_.front());
            }
        }
        const auto oldMapping = ModelTrajPlan::trajectoryFrameMapping(previous.normalAxis, previous.forwardAxis);
        const auto updatePreview = [this, normal, forward, buttons, status, first, oldMapping]() {
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"));
            const bool valid = ModelTrajPlan::validTrajectoryAxes(normal->currentIndex(), forward->currentIndex());
            buttons->button(QDialogButtonBox::Ok)->setEnabled(valid);
            if(!valid) { status->setText(QStringLiteral("法向轴与前进轴不能使用同一个轴。")); return; }
            const int third = 3 - normal->currentIndex() / 2 - forward->currentIndex() / 2;
            status->setText(QStringLiteral("第三轴：%1，按右手系计算。%2")
                .arg(QStringList{"X", "Y", "Z"}[third])
                .arg(first.empty() ? QStringLiteral("请先完成切片，以预览首个轨迹点。") : QStringLiteral("当前显示首个轨迹点姿态。")));
            if(first.empty()) return;
            auto preview = first;
            const Eigen::Matrix3d delta = oldMapping.transpose() * ModelTrajPlan::trajectoryFrameMapping(normal->currentIndex(), forward->currentIndex());
            preview.front().pose.block<3, 3>(0, 0) = (preview.front().pose.block<3, 3>(0, 0) * delta).eval();
            submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"),
                m_renderer.makeTrajectoryFramePreview(preview), m_algorithmToSceneScale);
        };
        connect(normal, qOverload<int>(&QComboBox::currentIndexChanged), dialog, updatePreview);
        connect(forward, qOverload<int>(&QComboBox::currentIndexChanged), dialog, updatePreview);
        connect(buttons, &QDialogButtonBox::accepted, dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
        connect(dialog, &QDialog::finished, this, [this, dialog, normal, forward, updatePreview](int result) {
        dialog->setObjectName(QString());
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("frameConfiguration"));
        if(result != QDialog::Accepted) return;
        auto& trajectory = m_generalData.trajData_;
        const Eigen::Matrix3d currentMapping = ModelTrajPlan::trajectoryFrameMapping(
            trajectory.generationParams_.normalAxis, trajectory.generationParams_.forwardAxis);
        const Eigen::Matrix3d delta = currentMapping.transpose() * ModelTrajPlan::trajectoryFrameMapping(normal->currentIndex(), forward->currentIndex());
        const auto remap = [&delta](auto& point) {
            point.pose.template block<3, 3>(0, 0) = (point.pose.template block<3, 3>(0, 0) * delta).eval();
        };
        for(auto& point : trajectory.trajectoryPoints_) remap(point);
        for(auto& point : trajectory.connectionData_.connectedTrajectory_) remap(point);
        for(auto& point : trajectory.trajectoryPointsbf_) remap(point.second);
        for(auto& point : trajectory.trajectoryPointsaf_) remap(point.second);
        trajectory.generationParams_.normalAxis = normal->currentIndex();
        trajectory.generationParams_.forwardAxis = forward->currentIndex();
        QSettings settings;
        settings.setValue("freeform/normalAxis", normal->currentIndex());
        settings.setValue("freeform/forwardAxis", forward->currentIndex());
        setVisualizationVisibility(m_modelVisible, m_slicesVisible, m_trajectoriesVisible, m_connectionsVisible, m_referenceLinesVisible);
        if(!trajectory.trajectoryPoints_.empty() && m_previewUpdated) m_previewUpdated(m_renderer.makePreview(trajectory.trajectoryPoints_));
        if(trajectory.trajectoryPoints_.empty()) updatePreview();
        showResult(true, QStringLiteral("姿态配置已保存，已有轨迹姿态已更新。"));
        });
        updatePreview();
        dialog->show();
    }

    bool FreeformTrajPlanWorkbenchController::applyTrajectoryTransform(
        const Eigen::Matrix4d& transform, bool transformModel, bool transformTrajectory,
        QString& errorMessage)
    {
        if(!transformModel && !transformTrajectory) {
            errorMessage = QStringLiteral("请至少选择模型或轨迹。");
            return false;
        }
        if(!transform.allFinite() || !transform.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1.0e-9)) {
            errorMessage = QStringLiteral("变换矩阵无效。");
            return false;
        }

        if(transformModel && !m_modelObjectId.isEmpty()) {
            Eigen::Matrix4d sceneTransform = transform;
            sceneTransform.block<3, 1>(0, 3) *= m_algorithmToSceneScale;
            const auto sceneUpdate = m_context.documentController().mutateProject(
                QStringLiteral("freeformTrajPlan.transformModel"),
                robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
                [this, sceneTransform](simulation_project::ProjectDocumentService& service,
                    bool& changed, std::string& error) {
                    simulation_project::SceneObjectDesc* object =
                        service.findSceneObject(m_modelObjectId.toStdString());
                    if(object == nullptr) {
                        error = "The workpiece was not found in the project document.";
                        return false;
                    }
                    Eigen::Matrix4d current = Eigen::Matrix4d::Identity();
                    current.block<3, 1>(0, 3) = Eigen::Vector3d(
                        object->transform.x, object->transform.y, object->transform.z);
                    current.topLeftCorner<3, 3>() =
                        Eigen::AngleAxisd(object->transform.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                        Eigen::AngleAxisd(object->transform.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                        Eigen::AngleAxisd(object->transform.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
                    const Eigen::Matrix4d combined = sceneTransform * current;
                    const Eigen::Vector3d zyx = combined.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
                    simulation_project::TransformDesc updated;
                    updated.x = combined(0, 3); updated.y = combined(1, 3); updated.z = combined(2, 3);
                    updated.yaw = zyx.x(); updated.pitch = zyx.y(); updated.roll = zyx.z();
                    changed = true;
                    return service.setSceneObjectTransform(object->id, updated, &error);
                });
            if(!sceneUpdate.success) {
                errorMessage = sceneUpdate.message;
                return false;
            }
            robot_qt_viewer::ViewportReloadWorkflowController reloadWorkflow(m_context);
            const auto reload = reloadWorkflow.reload(QStringLiteral("freeformTrajPlan.transformModel"));
            if(!reload.success) {
                errorMessage = reload.errorMessage;
                return false;
            }
        }

        const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
        if(transformModel) {
            const auto transformMesh = [&transform, &rotation](ModelTrajPlan::PrivateModel& mesh) {
                for(std::size_t index = 0; index + 2 < mesh.m_vertices.size(); index += 3) {
                    const Eigen::Vector3d point = transformPoint(transform, Eigen::Vector3d(
                        mesh.m_vertices[index], mesh.m_vertices[index + 1], mesh.m_vertices[index + 2]));
                    mesh.m_vertices[index] = static_cast<float>(point.x());
                    mesh.m_vertices[index + 1] = static_cast<float>(point.y());
                    mesh.m_vertices[index + 2] = static_cast<float>(point.z());
                }
                for(std::size_t index = 0; index + 2 < mesh.m_normals.size(); index += 3) {
                    Eigen::Vector3d normal = rotation * Eigen::Vector3d(
                        mesh.m_normals[index], mesh.m_normals[index + 1], mesh.m_normals[index + 2]);
                    if(normal.squaredNorm() > 1.0e-16) normal.normalize();
                    mesh.m_normals[index] = static_cast<float>(normal.x());
                    mesh.m_normals[index + 1] = static_cast<float>(normal.y());
                    mesh.m_normals[index + 2] = static_cast<float>(normal.z());
                }
            };
            transformMesh(m_generalData.modelDataManager_.mainModelData_);
            for(auto& mesh : m_generalData.modelDataManager_.allModelsData_) transformMesh(mesh);
            for(auto& mesh : m_generalData.originalData_) transformMesh(mesh);
            for(auto& mesh : m_generalData.preprocessedData_) transformMesh(mesh);

            auto& boundary = m_generalData.boundaryData_;
            transformBoundaryLine(boundary.upperBoundary_, transform);
            transformBoundaryLine(boundary.lowerBoundary_, transform);
            transformBoundaryLine(boundary.upperBoundaryOffset_, transform);
            transformBoundaryLine(boundary.lowerBoundaryOffset_, transform);
            for(auto& line : boundary.boundaryLines_) transformBoundaryLine(line, transform);
            for(auto& line : boundary.referenceLines_) transformBoundaryLine(line, transform);
            transformLinePoints(boundary.intersectionPoints_, transform);
            transformLinePoints(boundary.projectedContourLines_, transform);
            transformLinePoints(boundary.LinePoints_, transform);
            transformLinePoints(boundary.Mainslicingnode, transform);
            for(auto& point : boundary.phaseReferenceLine_) {
                point.position = transformPoint(transform, point.position);
                point.normal = transformDirection(transform, point.normal);
                point.tangent = transformDirection(transform, point.tangent);
            }
            boundary.phaseReferencePlaneNormal_ = transformDirection(transform, boundary.phaseReferencePlaneNormal_);

            auto& slicing = m_generalData.slicingData_;
            for(auto& plane : slicing.cuttingPlanes_) {
                plane.center = transformPoint(transform, plane.center);
                plane.normal = transformDirection(transform, plane.normal);
                for(auto& vertex : plane.vertices) vertex = transformPoint(transform, vertex);
                plane.translation_x = plane.translation_y = plane.translation_z = 0.0;
                plane.rotation_x = plane.rotation_y = plane.rotation_z = 0.0;
            }
            for(auto& plane : slicing.slicingLocations_) {
                plane.point = transformPoint(transform, plane.point);
                plane.normal = transformDirection(transform, plane.normal);
            }
            for(auto& line : slicing.sliceLines_) transformBoundaryLine(line, transform);
            for(auto& line : slicing.currentSliceResult_.internalBoundaries) transformBoundaryLine(line, transform);
        }

        if(transformTrajectory) {
            auto& trajectory = m_generalData.trajData_;
            for(auto& point : trajectory.trajectoryPoints_) point.pose = transform * point.pose;
            for(auto& point : trajectory.connectionData_.connectedTrajectory_) point.pose = transform * point.pose;
            for(auto& point : trajectory.trajectoryPointsbf_) point.second.pose = transform * point.second.pose;
            for(auto& point : trajectory.trajectoryPointsaf_) point.second.pose = transform * point.second.pose;
            for(auto& segment : trajectory.connectionData_.connectionSegments_) {
                segment.startPoint = transformPoint(transform, segment.startPoint);
                segment.endPoint = transformPoint(transform, segment.endPoint);
            }
            for(auto& boundaryPoints : trajectory.sampledBoundaryPoints_) {
                for(auto& point : boundaryPoints) point = transformPoint(transform, point);
            }
        }
        m_generalData.transformMatrix_ = transform * m_generalData.transformMatrix_;
        refreshSlicingOverlays();
        if(m_previewUpdated && !m_generalData.trajData_.trajectoryPoints_.empty()) {
            m_previewUpdated(m_renderer.makePreview(m_generalData.trajData_.trajectoryPoints_));
        }
        setVisualizationVisibility(m_modelVisible, m_slicesVisible, m_trajectoriesVisible,
            m_connectionsVisible, m_referenceLinesVisible);
        return true;
    }

    void FreeformTrajPlanWorkbenchController::showTrajectoryTransformDialog()
    {
        auto* dialog = new QDialog(&m_workflowPanel);
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowTitle(QStringLiteral("轨迹转换"));
        dialog->setMinimumWidth(620);
        auto* layout = new QVBoxLayout(dialog);
        auto* targetGroup = new QGroupBox(QStringLiteral("转换对象"), dialog);
        auto* targetLayout = new QHBoxLayout(targetGroup);
        auto* modelCheck = new QCheckBox(QStringLiteral("转换导入模型"), targetGroup);
        auto* trajectoryCheck = new QCheckBox(QStringLiteral("转换导入轨迹"), targetGroup);
        modelCheck->setChecked(!m_generalData.modelDataManager_.mainModelData_.m_vertices.empty());
        trajectoryCheck->setChecked(!m_generalData.trajData_.trajectoryPoints_.empty());
        targetLayout->addWidget(modelCheck); targetLayout->addWidget(trajectoryCheck); targetLayout->addStretch(1);
        layout->addWidget(targetGroup);

        auto* poseGroup = new QGroupBox(QStringLiteral("平移与旋转"), dialog);
        auto* poseLayout = new QGridLayout(poseGroup);
        QDoubleSpinBox* poseValues[6];
        const QStringList labels = { QStringLiteral("平移 X (mm)"), QStringLiteral("平移 Y (mm)"), QStringLiteral("平移 Z (mm)"),
            QStringLiteral("旋转 X (deg)"), QStringLiteral("旋转 Y (deg)"), QStringLiteral("旋转 Z (deg)") };
        for(int index = 0; index < 6; ++index) {
            poseValues[index] = new QDoubleSpinBox(poseGroup);
            poseValues[index]->setRange(-1000000.0, 1000000.0); poseValues[index]->setDecimals(6);
            poseLayout->addWidget(new QLabel(labels[index], poseGroup), index / 3, (index % 3) * 2);
            poseLayout->addWidget(poseValues[index], index / 3, (index % 3) * 2 + 1);
        }
        layout->addWidget(poseGroup);

        auto* matrixGroup = new QGroupBox(QStringLiteral("4 x 4 齐次变换矩阵"), dialog);
        auto* matrixLayout = new QGridLayout(matrixGroup);
        QDoubleSpinBox* matrixValues[4][4];
        for(int row = 0; row < 4; ++row) for(int column = 0; column < 4; ++column) {
            matrixValues[row][column] = new QDoubleSpinBox(matrixGroup);
            matrixValues[row][column]->setRange(-1000000.0, 1000000.0); matrixValues[row][column]->setDecimals(9);
            matrixValues[row][column]->setValue(row == column ? 1.0 : 0.0);
            matrixLayout->addWidget(matrixValues[row][column], row, column);
        }
        layout->addWidget(matrixGroup);

        auto* fileButtons = new QHBoxLayout();
        auto* loadButton = new QPushButton(QStringLiteral("加载 JSON"), dialog);
        auto* saveButton = new QPushButton(QStringLiteral("保存 JSON"), dialog);
        fileButtons->addWidget(loadButton); fileButtons->addWidget(saveButton); fileButtons->addStretch(1);
        layout->addLayout(fileButtons);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, dialog);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定转换"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        layout->addWidget(buttons);

        const auto readMatrix = [matrixValues]() {
            Eigen::Matrix4d matrix;
            for(int row = 0; row < 4; ++row) for(int column = 0; column < 4; ++column) matrix(row, column) = matrixValues[row][column]->value();
            return matrix;
        };
        const auto writeMatrix = [matrixValues](const Eigen::Matrix4d& matrix) {
            for(int row = 0; row < 4; ++row) for(int column = 0; column < 4; ++column) {
                const QSignalBlocker blocker(matrixValues[row][column]); matrixValues[row][column]->setValue(matrix(row, column));
            }
        };
        const auto updatePoseFromMatrix = [poseValues, readMatrix]() {
            const Eigen::Matrix4d matrix = readMatrix();
            const Eigen::Vector3d zyx = matrix.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
            const double degrees = 180.0 / std::acos(-1.0);
            const double values[] = { matrix(0, 3), matrix(1, 3), matrix(2, 3), zyx.z() * degrees, zyx.y() * degrees, zyx.x() * degrees };
            for(int index = 0; index < 6; ++index) { const QSignalBlocker blocker(poseValues[index]); poseValues[index]->setValue(values[index]); }
        };
        const auto updateMatrixFromPose = [poseValues, writeMatrix]() {
            const double radians = std::acos(-1.0) / 180.0;
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            matrix.topLeftCorner<3, 3>() = Eigen::AngleAxisd(poseValues[5]->value() * radians, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                Eigen::AngleAxisd(poseValues[4]->value() * radians, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(poseValues[3]->value() * radians, Eigen::Vector3d::UnitX()).toRotationMatrix();
            matrix.block<3, 1>(0, 3) = Eigen::Vector3d(poseValues[0]->value(), poseValues[1]->value(), poseValues[2]->value());
            writeMatrix(matrix);
        };
        for(auto* value : poseValues) connect(value, qOverload<double>(&QDoubleSpinBox::valueChanged), dialog, updateMatrixFromPose);
        for(int row = 0; row < 4; ++row) for(int column = 0; column < 4; ++column) connect(matrixValues[row][column], qOverload<double>(&QDoubleSpinBox::valueChanged), dialog, updatePoseFromMatrix);
        connect(loadButton, &QPushButton::clicked, dialog, [dialog, writeMatrix, updatePoseFromMatrix]() {
            const QString path = QFileDialog::getOpenFileName(dialog, QStringLiteral("加载转换 JSON"), QString(), QStringLiteral("JSON files (*.json)"));
            if(path.isEmpty()) return;
            QFile file(path);
            if(!file.open(QIODevice::ReadOnly)) { QMessageBox::warning(dialog, QStringLiteral("加载失败"), QStringLiteral("无法读取 JSON 文件。")); return; }
            QJsonParseError parseError; const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
            Eigen::Matrix4d matrix; QString error;
            if(parseError.error != QJsonParseError::NoError || !document.isObject() || !matrixFromJson(document.object(), matrix, error)) {
                QMessageBox::warning(dialog, QStringLiteral("加载失败"), error.isEmpty() ? QStringLiteral("JSON 格式无效。") : error); return;
            }
            writeMatrix(matrix); updatePoseFromMatrix();
        });
        connect(saveButton, &QPushButton::clicked, dialog, [dialog, readMatrix]() {
            const QString path = QFileDialog::getSaveFileName(dialog, QStringLiteral("保存转换 JSON"), QStringLiteral("transform_export.json"), QStringLiteral("JSON files (*.json)"));
            if(path.isEmpty()) return;
            QSaveFile file(path); if(!file.open(QIODevice::WriteOnly)) { QMessageBox::warning(dialog, QStringLiteral("保存失败"), QStringLiteral("无法写入 JSON 文件。")); return; }
            QJsonObject object; object.insert(QStringLiteral("version"), QStringLiteral("1.0")); object.insert(QStringLiteral("export_type"), QStringLiteral("transformed_input_set")); object.insert(QStringLiteral("transform_matrix"), matrixToJson(readMatrix()));
            file.write(QJsonDocument(object).toJson(QJsonDocument::Indented));
            if(!file.commit()) QMessageBox::warning(dialog, QStringLiteral("保存失败"), QStringLiteral("JSON 文件保存失败。"));
        });
        connect(buttons, &QDialogButtonBox::accepted, dialog, [this, dialog, readMatrix, modelCheck, trajectoryCheck]() {
            QString error;
            if(!applyTrajectoryTransform(readMatrix(), modelCheck->isChecked(), trajectoryCheck->isChecked(), error)) { QMessageBox::warning(dialog, QStringLiteral("转换失败"), error); return; }
            showResult(true, QStringLiteral("已应用轨迹转换；后续导出将使用转换后的轨迹。")); dialog->accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
        dialog->show();
    }

    // Replaced by the GeneralData-based implementation below. Kept here only
    // temporarily as disabled history while the Workbench preview adapter is
    // completed against the original algorithm structures.
#if 0
    void FreeformTrajPlanWorkbenchController::refreshSlicingOverlays()
    {
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("slicePlanes"));
        submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("slices"),
            m_renderer.makeSlicingPreview(m_document.slicing()));
        submitSliceSurfaces(m_context.visualizationViewport(), m_document.slicing());
    }

    void FreeformTrajPlanWorkbenchController::showSlicingAdjustDialog()
    {
        const auto& slicing = m_document.slicing();
        if(slicing.layerHeights.empty()) {
            showResult({ false, "Slice the surface before adjusting a slicing layer." });
            return;
        }

        QDialog dialog(&m_workflowPanel);
        dialog.setWindowTitle(QStringLiteral("切片调整"));
        dialog.setMinimumSize(460, 430);
        auto* layout = new QVBoxLayout(&dialog);
        auto* description = new QLabel(
            QStringLiteral("选择切片层会读取已确认的平移和旋转。应用后会重新求交并刷新该层切平面。"), &dialog);
        description->setWordWrap(true);
        layout->addWidget(description);

        auto* table = new QTableWidget(&dialog);
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels({ QStringLiteral("层"), QStringLiteral("Z 坐标 (mm)"),
            QStringLiteral("轮廓数") });
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->horizontalHeader()->setStretchLastSection(true);
        layout->addWidget(table, 1);

        auto* form = new QFormLayout();
        const auto makeDraftSpinBox = [&dialog](const QString& suffix) {
            auto* spinBox = new QDoubleSpinBox(&dialog);
            spinBox->setRange(-1000000.0, 1000000.0);
            spinBox->setDecimals(3);
            spinBox->setSuffix(suffix);
            return spinBox;
        };
        auto* translationX = makeDraftSpinBox(QStringLiteral(" mm"));
        auto* translationY = makeDraftSpinBox(QStringLiteral(" mm"));
        auto* translationZ = makeDraftSpinBox(QStringLiteral(" mm"));
        auto* rotationX = makeDraftSpinBox(QStringLiteral(" deg"));
        auto* rotationY = makeDraftSpinBox(QStringLiteral(" deg"));
        auto* rotationZ = makeDraftSpinBox(QStringLiteral(" deg"));
        form->addRow(QStringLiteral("平移 X"), translationX);
        form->addRow(QStringLiteral("平移 Y"), translationY);
        form->addRow(QStringLiteral("平移 Z"), translationZ);
        form->addRow(QStringLiteral("旋转 X"), rotationX);
        form->addRow(QStringLiteral("旋转 Y"), rotationY);
        form->addRow(QStringLiteral("旋转 Z"), rotationZ);
        layout->addLayout(form);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Apply | QDialogButtonBox::Close, &dialog);
        layout->addWidget(buttons);

        const auto rebuildTable = [this, table]() {
            const auto& current = m_document.slicing();
            const QSignalBlocker blocker(table);
            table->setRowCount(static_cast<int>(current.layerHeights.size()));
            for(std::size_t layerIndex = 0; layerIndex < current.layerHeights.size(); ++layerIndex) {
                int contourCount = 0;
                for(const auto& contour : current.contours) {
                    if(contour.layerIndex == layerIndex) {
                        ++contourCount;
                    }
                }
                table->setItem(static_cast<int>(layerIndex), 0,
                    new QTableWidgetItem(QString::number(layerIndex + 1)));
                table->setItem(static_cast<int>(layerIndex), 1,
                    new QTableWidgetItem(QString::number(current.layerHeights[layerIndex] * 1000.0, 'f', 3)));
                table->setItem(static_cast<int>(layerIndex), 2,
                    new QTableWidgetItem(QString::number(contourCount)));
            }
            const int selected = current.selectedLayerIndex >= 0 ? current.selectedLayerIndex : 0;
            table->selectRow(selected);
            table->setCurrentCell(selected, 0);
        };
        const int initialLayer = slicing.selectedLayerIndex >= 0 ? slicing.selectedLayerIndex : 0;
        m_document.selectSliceLayer(static_cast<std::size_t>(initialLayer));
        rebuildTable();
        const auto loadDraftFromDocument = [this, translationX, translationY, translationZ,
                                               rotationX, rotationY, rotationZ](int row) {
            if(row < 0 || static_cast<std::size_t>(row) >= m_document.slicing().planes.size()) {
                return;
            }
            const auto& plane = m_document.slicing().planes[static_cast<std::size_t>(row)];
            const QSignalBlocker blockTranslationX(translationX);
            const QSignalBlocker blockTranslationY(translationY);
            const QSignalBlocker blockTranslationZ(translationZ);
            const QSignalBlocker blockRotationX(rotationX);
            const QSignalBlocker blockRotationY(rotationY);
            const QSignalBlocker blockRotationZ(rotationZ);
            translationX->setValue(plane.translation.x * 1000.0);
            translationY->setValue(plane.translation.y * 1000.0);
            translationZ->setValue(plane.translation.z * 1000.0);
            rotationX->setValue(plane.rotationDegrees.x);
            rotationY->setValue(plane.rotationDegrees.y);
            rotationZ->setValue(plane.rotationDegrees.z);
        };
        loadDraftFromDocument(initialLayer);

        connect(table, &QTableWidget::currentCellChanged, &dialog,
            [this, loadDraftFromDocument](int row, int, int, int) {
                if(row < 0) {
                    return;
                }
                const auto result = m_document.selectSliceLayer(static_cast<std::size_t>(row));
                if(!result.success) {
                    showResult(result);
                    return;
                }
                loadDraftFromDocument(row);
                refreshSlicingOverlays();
            });
        connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, &dialog,
            [this, table, translationX, translationY, translationZ,
                rotationX, rotationY, rotationZ, rebuildTable, loadDraftFromDocument]() {
                const int row = table->currentRow();
                if(row < 0) {
                    showResult({ false, "Select a slicing layer before applying an adjustment." });
                    return;
                }
                const auto result = m_document.adjustSliceLayerTransform(static_cast<std::size_t>(row),
                    { translationX->value() * 0.001, translationY->value() * 0.001,
                        translationZ->value() * 0.001 },
                    { rotationX->value(), rotationY->value(), rotationZ->value() });
                showResult(result);
                if(!result.success) {
                    return;
                }
                if(m_previewCleared) {
                    m_previewCleared();
                }
                clearOverlay(m_context.visualizationViewport(), QStringLiteral("connections"));
                refreshSlicingOverlays();
                rebuildTable();
                loadDraftFromDocument(row);
            });
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        dialog.exec();
    }
    void FreeformTrajPlanWorkbenchController::showResult(const spray_freeform_traj_plan::FreeformTrajPlanCommandResult& result)
    {
        const QString message = QString::fromStdString(result.message);
        m_workflowPanel.setStatus(message, result.success);
        emit statusMessageRequested(message, result.success ? 3000 : 6000);
    }
#endif

    void FreeformTrajPlanWorkbenchController::refreshSlicingOverlays()
    {
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("slices"));
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("sliceSurfaces"));
        if(m_slicesVisible) {
            submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("slices"),
                m_renderer.makeSlicingPreview(m_generalData), m_algorithmToSceneScale);
            submitSurfaceOverlay(m_context.visualizationViewport(), QStringLiteral("sliceSurfaces"),
                m_renderer.makeSlicingSurfacePreview(m_generalData), m_algorithmToSceneScale);
        }
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("referenceLines"));
        if(m_referenceLinesVisible) {
            const auto preview = m_renderer.makeReferenceLinePreview(m_generalData);
            if(!preview.empty()) {
                const auto result = submitLineOverlay(m_context.visualizationViewport(),
                    QStringLiteral("referenceLines"), preview, m_algorithmToSceneScale);
                if(!result.success) {
                    showResult(false, QStringLiteral("参考线显示失败：%1")
                        .arg(QString::fromStdString(result.message)));
                }
            }
        }
    }

    void FreeformTrajPlanWorkbenchController::showSlicingAdjustDialog()
    {
        if(m_generalData.slicingData_.cuttingPlanes_.empty()) {
            showResult(false, QStringLiteral("请先完成切片，再调整切片层。"));
            return;
        }

        auto* dialog = new QDialog(&m_workflowPanel);
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowModality(Qt::NonModal);
        dialog->setWindowTitle(QStringLiteral("切片控制"));
        dialog->setMinimumSize(840, 650);
        auto* layout = new QVBoxLayout(dialog);
        auto* controlGroup = new QGroupBox(QStringLiteral("切片控制"), dialog);
        auto* controlLayout = new QVBoxLayout(controlGroup);
        auto* table = new QTableWidget(controlGroup);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({ QStringLiteral("层"), QStringLiteral("Z 位置"),
            QStringLiteral("厚度"), QStringLiteral("轮廓点数") });
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->horizontalHeader()->setStretchLastSection(true);
        controlLayout->addWidget(table, 1);

        auto* currentLayer = new QLabel(controlGroup);
        auto* keepVisible = new QCheckBox(QStringLiteral("保持显示"), controlGroup);
        auto* currentRow = new QHBoxLayout();
        currentRow->addWidget(currentLayer);
        currentRow->addStretch(1);
        currentRow->addWidget(keepVisible);
        controlLayout->addLayout(currentRow);

        const auto makePlainValue = [dialog](double initialValue, const QString& suffix) {
            auto* spinBox = new QDoubleSpinBox(dialog);
            spinBox->setRange(-1000000.0, 1000000.0);
            spinBox->setDecimals(3);
            spinBox->setSuffix(suffix);
            spinBox->setValue(initialValue);
            return spinBox;
        };

        auto* layerStep = makePlainValue(10.0, QStringLiteral(" mm"));
        auto* targetZ = makePlainValue(0.0, QStringLiteral(" mm"));
        auto* decreaseLayer = new QPushButton(QStringLiteral("- 层"), controlGroup);
        auto* increaseLayer = new QPushButton(QStringLiteral("+ 层"), controlGroup);
        auto* setTarget = new QPushButton(QStringLiteral("设置"), controlGroup);
        auto* layerGrid = new QGridLayout();
        layerGrid->addWidget(new QLabel(QStringLiteral("步长"), controlGroup), 0, 0);
        layerGrid->addWidget(layerStep, 0, 1);
        layerGrid->addWidget(decreaseLayer, 0, 2);
        layerGrid->addWidget(increaseLayer, 0, 3);
        layerGrid->addWidget(new QLabel(QStringLiteral("目标 Z"), controlGroup), 1, 0);
        layerGrid->addWidget(targetZ, 1, 1, 1, 2);
        layerGrid->addWidget(setTarget, 1, 3);
        controlLayout->addLayout(layerGrid);
        auto* resetUniform = new QPushButton(QStringLiteral("重置均匀分布"), controlGroup);
        controlLayout->addWidget(resetUniform, 0, Qt::AlignLeft);
        auto* rangeLabel = new QLabel(controlGroup);
        controlLayout->addWidget(rangeLabel);
        layout->addWidget(controlGroup, 1);

        const auto makeAxisControl = [dialog](const QString& title, const QString& unit,
                                                  QDoubleSpinBox*& value, QDoubleSpinBox*& step,
                                                  QPushButton*& decrease, QPushButton*& increase) {
            auto* grid = new QGridLayout();
            grid->addWidget(new QLabel(title, dialog), 0, 0);
            decrease = new QPushButton(QStringLiteral("-"), dialog);
            increase = new QPushButton(QStringLiteral("+"), dialog);
            value = new QDoubleSpinBox(dialog);
            step = new QDoubleSpinBox(dialog);
            value->setRange(-1000000.0, 1000000.0);
            value->setDecimals(3);
            value->setSuffix(unit);
            step->setRange(0.001, 1000000.0);
            step->setValue(1.0);
            step->setDecimals(3);
            step->setSuffix(unit);
            grid->addWidget(decrease, 0, 1);
            grid->addWidget(value, 0, 2);
            grid->addWidget(increase, 0, 3);
            grid->addWidget(new QLabel(QStringLiteral("步长"), dialog), 0, 4);
            grid->addWidget(step, 0, 5);
            return grid;
        };
        auto* translationGroup = new QGroupBox(QStringLiteral("平移控制"), dialog);
        auto* translationLayout = new QVBoxLayout(translationGroup);
        QDoubleSpinBox *tx, *ty, *tz, *txStep, *tyStep, *tzStep;
        QPushButton *txMinus, *txPlus, *tyMinus, *tyPlus, *tzMinus, *tzPlus;
        translationLayout->addLayout(makeAxisControl(QStringLiteral("X 轴"), QStringLiteral(" mm"), tx, txStep, txMinus, txPlus));
        translationLayout->addLayout(makeAxisControl(QStringLiteral("Y 轴"), QStringLiteral(" mm"), ty, tyStep, tyMinus, tyPlus));
        translationLayout->addLayout(makeAxisControl(QStringLiteral("Z 轴"), QStringLiteral(" mm"), tz, tzStep, tzMinus, tzPlus));
        layout->addWidget(translationGroup);

        auto* rotationGroup = new QGroupBox(QStringLiteral("旋转控制"), dialog);
        auto* rotationLayout = new QVBoxLayout(rotationGroup);
        QDoubleSpinBox *rx, *ry, *rz, *rxStep, *ryStep, *rzStep;
        QPushButton *rxMinus, *rxPlus, *ryMinus, *ryPlus, *rzMinus, *rzPlus;
        rotationLayout->addLayout(makeAxisControl(QStringLiteral("X 轴"), QStringLiteral(" deg"), rx, rxStep, rxMinus, rxPlus));
        rotationLayout->addLayout(makeAxisControl(QStringLiteral("Y 轴"), QStringLiteral(" deg"), ry, ryStep, ryMinus, ryPlus));
        rotationLayout->addLayout(makeAxisControl(QStringLiteral("Z 轴"), QStringLiteral(" deg"), rz, rzStep, rzMinus, rzPlus));
        layout->addWidget(rotationGroup);

        auto* resetTransform = new QPushButton(QStringLiteral("重置变换"), dialog);
        auto* applyTransform = new QPushButton(QStringLiteral("应用变换"), dialog);
        auto* transformButtons = new QHBoxLayout();
        transformButtons->addWidget(resetTransform);
        transformButtons->addWidget(applyTransform);
        layout->addLayout(transformButtons);

        struct SlicingAdjustDialogState final {
            int selectedIndex = 0;
            bool rebuilding = false;
            bool loadingValues = false;
        };
        const auto state = std::make_shared<SlicingAdjustDialogState>();
        state->selectedIndex = m_generalData.slicingData_.lastSelectedLayerIndex;
        if(state->selectedIndex < 0 || state->selectedIndex >= static_cast<int>(m_generalData.slicingData_.cuttingPlanes_.size())) {
            state->selectedIndex = 0;
        }
        const auto selectLayer = [this, state]() {
            auto& planes = m_generalData.slicingData_.cuttingPlanes_;
            for(std::size_t index = 0; index < planes.size(); ++index) {
                planes[index].isSelected = static_cast<int>(index) == state->selectedIndex;
            }
            m_generalData.slicingData_.lastSelectedLayerIndex = state->selectedIndex;
            refreshSlicingOverlays();
        };
        const auto loadSelectedPlane = [this, state, tx, ty, tz, rx, ry, rz, targetZ]() {
            const auto& plane = m_generalData.slicingData_.cuttingPlanes_[static_cast<std::size_t>(state->selectedIndex)];
            state->loadingValues = true;
            const QSignalBlocker blockTx(tx), blockTy(ty), blockTz(tz), blockRx(rx), blockRy(ry), blockRz(rz), blockTarget(targetZ);
            tx->setValue(plane.center.x() + plane.translation_x);
            ty->setValue(plane.center.y() + plane.translation_y);
            tz->setValue(plane.center.z() + plane.translation_z);
            rx->setValue(plane.rotation_x); ry->setValue(plane.rotation_y); rz->setValue(plane.rotation_z);
            targetZ->setValue(plane.center.z() + plane.translation_z);
            state->loadingValues = false;
        };
        const auto updateSelectedPlane = [this, state, tx, ty, tz, rx, ry, rz]() {
            if(state->loadingValues) {
                return;
            }
            auto& plane = m_generalData.slicingData_.cuttingPlanes_[static_cast<std::size_t>(state->selectedIndex)];
            plane.translation_x = tx->value() - plane.center.x();
            plane.translation_y = ty->value() - plane.center.y();
            plane.translation_z = tz->value() - plane.center.z();
            plane.rotation_x = rx->value(); plane.rotation_y = ry->value(); plane.rotation_z = rz->value();
            m_generalData.slicingData_.slicingisupdata = true;
            if(!m_plan.updateSingleCuttingPlaneSize(m_generalData, state->selectedIndex)) {
                showResult(false, QStringLiteral("切片层几何更新失败。"));
                return;
            }
            clearOverlay(m_context.visualizationViewport(), QStringLiteral("connections"));
            refreshSlicingOverlays();
        };
        const auto rebuildTable = [this, state, table, currentLayer, rangeLabel]() {
            const auto& planes = m_generalData.slicingData_.cuttingPlanes_;
            state->rebuilding = true;
            const QSignalBlocker block(table);
            table->clearContents();
            table->setRowCount(static_cast<int>(planes.size()));
            double minZ = std::numeric_limits<double>::max();
            double maxZ = std::numeric_limits<double>::lowest();
            for(std::size_t index = 0; index < planes.size(); ++index) {
                const auto& plane = planes[index];
                const double z = plane.center.z() + plane.translation_z;
                minZ = std::min(minZ, z); maxZ = std::max(maxZ, z);
                const double nextZ = index + 1 < planes.size()
                    ? planes[index + 1].center.z() + planes[index + 1].translation_z : z;
                const int contourCount = index < m_generalData.slicingData_.currentSliceResult_.internalBoundaries.size()
                    ? static_cast<int>(m_generalData.slicingData_.currentSliceResult_.internalBoundaries[index].points.size()) : 0;
                auto* layerItem = new QTableWidgetItem(QString::number(plane.layer_index));
                layerItem->setFlags(layerItem->flags() & ~Qt::ItemIsEditable);
                auto* zItem = new QTableWidgetItem(QString::number(z, 'f', 3));
                auto* thicknessItem = new QTableWidgetItem(index + 1 < planes.size()
                    ? QString::number(std::abs(nextZ - z), 'f', 3) : QStringLiteral("-"));
                thicknessItem->setFlags(thicknessItem->flags() & ~Qt::ItemIsEditable);
                auto* contourItem = new QTableWidgetItem(QString::number(contourCount));
                contourItem->setFlags(contourItem->flags() & ~Qt::ItemIsEditable);
                table->setItem(static_cast<int>(index), 0, layerItem);
                table->setItem(static_cast<int>(index), 1, zItem);
                table->setItem(static_cast<int>(index), 2, thicknessItem);
                table->setItem(static_cast<int>(index), 3, contourItem);
            }
            table->resizeColumnsToContents();
            table->selectRow(state->selectedIndex);
            currentLayer->setText(QStringLiteral("当前层: %1").arg(state->selectedIndex + 1));
            rangeLabel->setText(QStringLiteral("范围: [%1, %2] mm").arg(minZ, 0, 'f', 3).arg(maxZ, 0, 'f', 3));
            state->rebuilding = false;
        };
        const auto selectAndLoad = [state, selectLayer, loadSelectedPlane, rebuildTable](int index) {
            state->selectedIndex = index;
            selectLayer();
            loadSelectedPlane();
            rebuildTable();
        };

        connect(table, &QTableWidget::currentCellChanged, dialog,
            [state, selectAndLoad](int row, int, int, int) { if(!state->rebuilding && row >= 0) selectAndLoad(row); });
        connect(table, &QTableWidget::itemChanged, dialog,
            [state, tz, updateSelectedPlane, rebuildTable](QTableWidgetItem* item) {
                if(state->rebuilding || item == nullptr || item->column() != 1) return;
                bool valid = false; const double z = item->text().toDouble(&valid);
                if(!valid) { rebuildTable(); return; }
                state->selectedIndex = item->row();
                { const QSignalBlocker blocker(tz); tz->setValue(z); }
                updateSelectedPlane();
                rebuildTable();
            });
        for(QDoubleSpinBox* value : { tx, ty, tz, rx, ry, rz }) {
            connect(value, qOverload<double>(&QDoubleSpinBox::valueChanged), dialog,
                [updateSelectedPlane, rebuildTable](double) { updateSelectedPlane(); rebuildTable(); });
        }
        connect(txMinus, &QPushButton::clicked, dialog, [tx, txStep]() { tx->setValue(tx->value() - txStep->value()); });
        connect(txPlus, &QPushButton::clicked, dialog, [tx, txStep]() { tx->setValue(tx->value() + txStep->value()); });
        connect(tyMinus, &QPushButton::clicked, dialog, [ty, tyStep]() { ty->setValue(ty->value() - tyStep->value()); });
        connect(tyPlus, &QPushButton::clicked, dialog, [ty, tyStep]() { ty->setValue(ty->value() + tyStep->value()); });
        connect(tzMinus, &QPushButton::clicked, dialog, [tz, tzStep]() { tz->setValue(tz->value() - tzStep->value()); });
        connect(tzPlus, &QPushButton::clicked, dialog, [tz, tzStep]() { tz->setValue(tz->value() + tzStep->value()); });
        connect(rxMinus, &QPushButton::clicked, dialog, [rx, rxStep]() { rx->setValue(rx->value() - rxStep->value()); });
        connect(rxPlus, &QPushButton::clicked, dialog, [rx, rxStep]() { rx->setValue(rx->value() + rxStep->value()); });
        connect(ryMinus, &QPushButton::clicked, dialog, [ry, ryStep]() { ry->setValue(ry->value() - ryStep->value()); });
        connect(ryPlus, &QPushButton::clicked, dialog, [ry, ryStep]() { ry->setValue(ry->value() + ryStep->value()); });
        connect(rzMinus, &QPushButton::clicked, dialog, [rz, rzStep]() { rz->setValue(rz->value() - rzStep->value()); });
        connect(rzPlus, &QPushButton::clicked, dialog, [rz, rzStep]() { rz->setValue(rz->value() + rzStep->value()); });
        connect(decreaseLayer, &QPushButton::clicked, dialog, [tz, layerStep]() { tz->setValue(tz->value() - layerStep->value()); });
        connect(increaseLayer, &QPushButton::clicked, dialog, [tz, layerStep]() { tz->setValue(tz->value() + layerStep->value()); });
        connect(setTarget, &QPushButton::clicked, dialog, [tz, targetZ]() { tz->setValue(targetZ->value()); });
        connect(resetTransform, &QPushButton::clicked, dialog, [this, state, tx, ty, tz, rx, ry, rz, updateSelectedPlane, rebuildTable]() {
            const QSignalBlocker blockTx(tx), blockTy(ty), blockTz(tz), blockRx(rx), blockRy(ry), blockRz(rz);
            const auto& plane = m_generalData.slicingData_.cuttingPlanes_[static_cast<std::size_t>(state->selectedIndex)];
            tx->setValue(plane.center.x()); ty->setValue(plane.center.y()); tz->setValue(plane.center.z());
            rx->setValue(0.0); ry->setValue(0.0); rz->setValue(0.0);
            updateSelectedPlane();
            rebuildTable();
        });
        connect(applyTransform, &QPushButton::clicked, dialog, [updateSelectedPlane, rebuildTable]() { updateSelectedPlane(); rebuildTable(); });
        connect(resetUniform, &QPushButton::clicked, dialog,
            [this, state, tz, loadSelectedPlane, rebuildTable, selectLayer]() {
                auto& planes = m_generalData.slicingData_.cuttingPlanes_;
                if(planes.size() < 2) return;
                double minZ = std::numeric_limits<double>::max(), maxZ = std::numeric_limits<double>::lowest();
                for(const auto& plane : planes) { minZ = std::min(minZ, plane.center.z() + plane.translation_z); maxZ = std::max(maxZ, plane.center.z() + plane.translation_z); }
                for(std::size_t index = 0; index < planes.size(); ++index) {
                    const double z = minZ + (maxZ - minZ) * static_cast<double>(index) / static_cast<double>(planes.size() - 1);
                    planes[index].translation_z = z - planes[index].center.z();
                    m_generalData.slicingData_.slicingisupdata = true;
                    m_plan.updateSingleCuttingPlaneSize(m_generalData, static_cast<int>(index));
                }
                selectLayer(); loadSelectedPlane(); rebuildTable(); refreshSlicingOverlays();
            });
        connect(keepVisible, &QCheckBox::toggled, dialog, [this](bool) { refreshSlicingOverlays(); });

        selectLayer();
        loadSelectedPlane();
        rebuildTable();
        dialog->show();
    }

    void FreeformTrajPlanWorkbenchController::showResult(bool success, const QString& message)
    {
        m_workflowPanel.setStatus(message, success);
        emit statusMessageRequested(message, success ? 3000 : 6000);
    }

    SprayTrajPlan::TrajectoryParams FreeformTrajPlanWorkbenchController::uvTrajectoryParameters() const
    {
        return m_uvParameters;
    }

    void FreeformTrajPlanWorkbenchController::refreshUvTrajectoryOverlay()
    {
        clearOverlay(m_context.visualizationViewport(), QStringLiteral("uvTrajectory"));
        if(!m_trajectoriesVisible) {
            return;
        }
        submitLineOverlay(m_context.visualizationViewport(), QStringLiteral("uvTrajectory"),
            m_renderer.makeUvTrajectoryPreview(m_uvPipeline.getTrajectory()), m_algorithmToSceneScale);
    }
}
