#include "FreeformTrajPlanRenderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    struct SliceColor
    {
        double red;
        double green;
        double blue;
        double alpha;
    };

    SliceColor colorForLayer(std::size_t layerIndex, bool selected)
    {
        if(selected) {
            // Yellow is reserved exclusively for the currently selected layer.
            return { 1.0, 0.90, 0.05, 1.0 };
        }
        constexpr std::array<SliceColor, 6> kLayerColors = {{
            { 0.10, 0.68, 1.00, 0.92 }, { 0.16, 0.86, 0.63, 0.92 },
            { 0.55, 0.36, 1.00, 0.92 }, { 0.90, 0.30, 0.72, 0.92 },
            { 0.25, 0.52, 0.96, 0.92 }, { 0.30, 0.82, 0.94, 0.92 }
        }};
        return kLayerColors[layerIndex % kLayerColors.size()];
    }

    Eigen::Matrix3d cuttingPlaneRotation(const ModelTrajPlan::CuttingPlane& plane)
    {
        const double rx = plane.rotation_x * M_PI / 180.0;
        const double ry = plane.rotation_y * M_PI / 180.0;
        const double rz = plane.rotation_z * M_PI / 180.0;
        Eigen::Matrix3d rotationX;
        Eigen::Matrix3d rotationY;
        Eigen::Matrix3d rotationZ;
        rotationX << 1.0, 0.0, 0.0, 0.0, std::cos(rx), -std::sin(rx), 0.0, std::sin(rx), std::cos(rx);
        rotationY << std::cos(ry), 0.0, std::sin(ry), 0.0, 1.0, 0.0, -std::sin(ry), 0.0, std::cos(ry);
        rotationZ << std::cos(rz), -std::sin(rz), 0.0, std::sin(rz), std::cos(rz), 0.0, 0.0, 0.0, 1.0;
        return rotationZ * rotationY * rotationX;
    }

    Eigen::Vector3d transformCuttingPlanePoint(const ModelTrajPlan::CuttingPlane& plane,
        const Eigen::Matrix3d& rotation, const Eigen::Vector3d& point)
    {
        return rotation * (point - plane.center) + plane.center + Eigen::Vector3d(
            plane.translation_x, plane.translation_y, plane.translation_z);
    }
}

namespace smrobot::workbench::freeform
{
    namespace
    {
        FreeformTrajPlanPreviewPoint makePreviewPoint(const ModelTrajPlan::TrajectoryPoint& point)
        {
            FreeformTrajPlanPreviewPoint preview;
            preview.positionX = point.pose(0, 3);
            preview.positionY = point.pose(1, 3);
            preview.positionZ = point.pose(2, 3);
            preview.frameXAxisX = point.pose(0, 0); preview.frameXAxisY = point.pose(1, 0); preview.frameXAxisZ = point.pose(2, 0);
            preview.frameYAxisX = point.pose(0, 1); preview.frameYAxisY = point.pose(1, 1); preview.frameYAxisZ = point.pose(2, 1);
            preview.frameZAxisX = point.pose(0, 2); preview.frameZAxisY = point.pose(1, 2); preview.frameZAxisZ = point.pose(2, 2);
            preview.directionX = preview.frameZAxisX;
            preview.directionY = preview.frameZAxisY;
            preview.directionZ = preview.frameZAxisZ;
            preview.sprayEnabled = true;
            preview.startsNewSegment = point.trajectory_index == 0;
            return preview;
        }
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeSlicingPreview(
        const ModelTrajPlan::GeneralData& document) const
    {
        FreeformTrajPlanLinePreview preview;
        const auto& slicing = document.slicingData_.currentSliceResult_;
        const auto& contours = slicing.internalBoundaries;
        for(std::size_t contourIndex = 0; contourIndex < contours.size(); ++contourIndex) {
            const auto& contour = contours[contourIndex];
            const SliceColor color = slicing.isBoundarylessFallback
                ? std::array<SliceColor, 5>{{
                    { 0.05, 0.95, 0.95, 1.0 }, { 1.0, 0.25, 0.85, 1.0 },
                    { 0.30, 1.0, 0.25, 1.0 }, { 0.45, 0.50, 1.0, 1.0 },
                    { 1.0, 0.20, 0.35, 1.0 } }}[contourIndex % 5]
                : SliceColor{ 0.10, 0.78, 1.00, 0.92 };
            for(std::size_t index = 1; index < contour.points.size(); ++index) {
                const auto& start = contour.points[index - 1].position;
                const auto& end = contour.points[index].position;
                preview.push_back({ start.x(), start.y(), start.z(), end.x(), end.y(), end.z(),
                    color.red, color.green, color.blue, color.alpha });
            }
        }
        return preview;
    }

    FreeformTrajPlanMeshPreview FreeformTrajPlanRenderer::makeSlicingSurfacePreview(
        const ModelTrajPlan::GeneralData& document) const
    {
        FreeformTrajPlanMeshPreview preview;
        const auto& planes = document.slicingData_.cuttingPlanes_;
        preview.vertices.reserve(planes.size() * 4);
        preview.indices.reserve(planes.size() * 6);
        for(std::size_t planeIndex = 0; planeIndex < planes.size(); ++planeIndex) {
            const auto& plane = planes[planeIndex];
            if(plane.vertices.size() < 4) {
                continue;
            }
            const Eigen::Matrix3d rotation = cuttingPlaneRotation(plane);
            Eigen::Vector3d normal = rotation * plane.normal;
            if(normal.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
                normal = Eigen::Vector3d::UnitZ();
            } else {
                normal.normalize();
            }
            SliceColor color = colorForLayer(planeIndex, plane.isSelected);
            if(!plane.isSelected && document.slicingData_.currentSliceResult_.isBoundarylessFallback) {
                constexpr std::array<SliceColor, 5> kBoundarylessColors = {{
                    { 0.05, 0.95, 0.95, 0.96 }, { 1.0, 0.25, 0.85, 0.96 },
                    { 0.30, 1.0, 0.25, 0.96 }, { 0.45, 0.50, 1.0, 0.96 },
                    { 1.0, 0.20, 0.35, 0.96 }
                }};
                color = kBoundarylessColors[planeIndex % kBoundarylessColors.size()];
            }
            const std::uint32_t base = static_cast<std::uint32_t>(preview.vertices.size());
            for(std::size_t vertexIndex = 0; vertexIndex < 4; ++vertexIndex) {
                const Eigen::Vector3d vertex = transformCuttingPlanePoint(
                    plane, rotation, plane.vertices[vertexIndex]);
                preview.vertices.push_back({ vertex.x(), vertex.y(), vertex.z(),
                    normal.x(), normal.y(), normal.z(), color.red, color.green, color.blue,
                    1.0 });
            }
            preview.indices.insert(preview.indices.end(),
                { base, base + 1, base + 2, base, base + 2, base + 3 });
        }
        return preview;
    }

    FreeformTrajPlanPreview FreeformTrajPlanRenderer::makePreview(
        const std::vector<ModelTrajPlan::TrajectoryPoint>& trajectory) const
    {
        FreeformTrajPlanPreview preview;
        preview.reserve(trajectory.size());
        for(const auto& point : trajectory) preview.push_back(makePreviewPoint(point));
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeTrajectoryFramePreview(
        const std::vector<ModelTrajPlan::TrajectoryPoint>& trajectory, double axisLength) const
    {
        FreeformTrajPlanLinePreview preview;
        for(const auto& point : trajectory) {
            const auto appendAxis = [&preview, &point, axisLength](int column, double red, double green, double blue) {
                preview.push_back({ point.pose(0, 3), point.pose(1, 3), point.pose(2, 3),
                    point.pose(0, 3) + point.pose(0, column) * axisLength,
                    point.pose(1, 3) + point.pose(1, column) * axisLength,
                    point.pose(2, 3) + point.pose(2, column) * axisLength, red, green, blue, 1.0 });
            };
            appendAxis(0, 1.0, 0.0, 0.0);
            appendAxis(1, 0.0, 1.0, 0.0);
            appendAxis(2, 0.0, 0.0, 1.0);
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeConnectionPreview(
        const std::vector<ModelTrajPlan::ConnectionSegment>& connections) const
    {
        FreeformTrajPlanLinePreview preview;
        preview.reserve(connections.size());
        for(const auto& connection : connections) {
            preview.push_back({ connection.startPoint.x(), connection.startPoint.y(), connection.startPoint.z(),
                connection.endPoint.x(), connection.endPoint.y(), connection.endPoint.z(),
                connection.color.x(), connection.color.y(), connection.color.z(), 1.0 });
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeReferenceLinePreview(
        const ModelTrajPlan::GeneralData& document) const
    {
        FreeformTrajPlanLinePreview preview;
        for(const auto& line : document.boundaryData_.referenceLines_) {
            if(line.points.size() < 2) continue;
            Eigen::Vector3d minimum = line.points.front().position;
            Eigen::Vector3d maximum = minimum;
            for(const auto& point : line.points) {
                minimum = minimum.cwiseMin(point.position);
                maximum = maximum.cwiseMax(point.position);
            }
            // The algorithm's center curve may have millions of 0.001-unit
            // samples. Coalesce display samples before conversion to floats.
            const double minimumStep = std::max(1.0, (maximum - minimum).norm() / 2000.0);
            Eigen::Vector3d start = line.points.front().position;
            for(std::size_t index = 1; index < line.points.size(); ++index) {
                const auto& end = line.points[index].position;
                if((end - start).norm() < minimumStep) continue;
                preview.push_back({ start.x(), start.y(), start.z(), end.x(), end.y(), end.z(),
                    line.color.x(), line.color.y(), line.color.z(), 1.0 });
                start = end;
            }
        }
        return preview;
    }

#if BUILD_UV_TRAJ_PLAN
    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeUvTrajectoryPreview(
        const std::vector<SprayTrajPlan::TrajectorySegment>& trajectory) const
    {
        FreeformTrajPlanLinePreview preview;
        for(const auto& segment : trajectory) {
            const bool sprayOn = segment.spray_on;
            const double red = sprayOn ? 0.95 : 1.0;
            const double green = sprayOn ? 0.25 : 0.65;
            const double blue = sprayOn ? 0.20 : 0.05;
            for(std::size_t index = 1; index < segment.xyz.size(); ++index) {
                const auto& start = segment.xyz[index - 1];
                const auto& end = segment.xyz[index];
                preview.push_back({ start.x(), start.y(), start.z(), end.x(), end.y(), end.z(),
                    red, green, blue, 1.0 });
            }
        }
        return preview;
    }


#endif
    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeBoundaryPreview(
        const spray_freeform_traj_plan::FreeformBoundaryResult& boundaries) const
    {
        FreeformTrajPlanLinePreview preview;
        const auto appendLine = [&preview](const auto& line) {
            if(line.points.size() < 2) {
                return;
            }
            const std::size_t segmentCount = line.closed ? line.points.size() : line.points.size() - 1;
            for(std::size_t index = 0; index < segmentCount; ++index) {
                const auto& start = line.points[index];
                const auto& end = line.points[(index + 1) % line.points.size()];
                preview.push_back({ start.x, start.y, start.z, end.x, end.y, end.z,
                    line.color.red, line.color.green, line.color.blue, line.color.alpha });
            }
        };
        appendLine(boundaries.upper);
        appendLine(boundaries.lower);
        appendLine(boundaries.upperOffset);
        appendLine(boundaries.lowerOffset);
        return preview;
    }

    FreeformTrajPlanMeshPreview FreeformTrajPlanRenderer::makeModelPreview(
        const spray_freeform_traj_plan::FreeformSurfaceMesh& mesh) const
    {
        FreeformTrajPlanMeshPreview preview;
        preview.vertices.reserve(mesh.triangles.size() * 3);
        preview.indices.reserve(mesh.triangles.size() * 3);
        for(const auto& triangle : mesh.triangles) {
            const auto subtract = [](const auto& left, const auto& right) {
                return spray_freeform_traj_plan::FreeformVec3{
                    left.x - right.x, left.y - right.y, left.z - right.z };
            };
            const auto cross = [](const auto& left, const auto& right) {
                return spray_freeform_traj_plan::FreeformVec3{
                    left.y * right.z - left.z * right.y,
                    left.z * right.x - left.x * right.z,
                    left.x * right.y - left.y * right.x };
            };
            const auto edgeA = subtract(triangle.vertices[1], triangle.vertices[0]);
            const auto edgeB = subtract(triangle.vertices[2], triangle.vertices[0]);
            auto normal = cross(edgeA, edgeB);
            const double length = std::sqrt(
                normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if(length <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            normal.x /= length;
            normal.y /= length;
            normal.z /= length;
            for(const auto& point : triangle.vertices) {
                preview.indices.push_back(static_cast<std::uint32_t>(preview.vertices.size()));
                preview.vertices.push_back({ point.x, point.y, point.z,
                    normal.x, normal.y, normal.z });
            }
        }
        return preview;
    }

    FreeformTrajPlanPreview FreeformTrajPlanRenderer::makePreview(
        const spray_freeform_traj_plan::FreeformTrajectory& trajectory) const
    {
        FreeformTrajPlanPreview preview;
        preview.reserve(trajectory.size());
        for(const auto& point : trajectory) {
            FreeformTrajPlanPreviewPoint output;
            output.positionX = point.positionX;
            output.positionY = point.positionY;
            output.positionZ = point.positionZ;
            output.directionX = point.directionX;
            output.directionY = point.directionY;
            output.directionZ = point.directionZ;
            output.frameXAxisX = point.frameXAxisX;
            output.frameXAxisY = point.frameXAxisY;
            output.frameXAxisZ = point.frameXAxisZ;
            output.frameYAxisX = point.frameYAxisX;
            output.frameYAxisY = point.frameYAxisY;
            output.frameYAxisZ = point.frameYAxisZ;
            output.frameZAxisX = point.frameZAxisX;
            output.frameZAxisY = point.frameZAxisY;
            output.frameZAxisZ = point.frameZAxisZ;
            output.sprayEnabled = point.sprayEnabled;
            output.startsNewSegment = point.startsNewSegment;
            preview.push_back(output);
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeTrajectoryFramePreview(
        const spray_freeform_traj_plan::FreeformTrajectory& trajectory, double axisLengthMeters) const
    {
        FreeformTrajPlanLinePreview preview;
        if(!std::isfinite(axisLengthMeters) || axisLengthMeters <= 0.0) {
            return preview;
        }
        preview.reserve(trajectory.size() * 3);
        const auto appendAxis = [&preview, axisLengthMeters](const auto& point, double axisX,
            double axisY, double axisZ, double red, double green, double blue) {
            const double length = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
            if(length <= std::numeric_limits<double>::epsilon()) {
                return;
            }
            preview.push_back({ point.positionX, point.positionY, point.positionZ,
                point.positionX + axisX / length * axisLengthMeters,
                point.positionY + axisY / length * axisLengthMeters,
                point.positionZ + axisZ / length * axisLengthMeters,
                red, green, blue, 1.0 });
        };
        for(const auto& point : trajectory) {
            appendAxis(point, point.frameXAxisX, point.frameXAxisY, point.frameXAxisZ, 1.0, 0.0, 0.0);
            appendAxis(point, point.frameYAxisX, point.frameYAxisY, point.frameYAxisZ, 0.0, 1.0, 0.0);
            appendAxis(point, point.frameZAxisX, point.frameZAxisY, point.frameZAxisZ, 0.0, 0.0, 1.0);
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeSlicingPreview(
        const spray_freeform_traj_plan::FreeformSlicingResult& slicing) const
    {
        FreeformTrajPlanLinePreview preview;
        for(const auto& contour : slicing.contours) {
            const SliceColor color = colorForLayer(contour.layerIndex,
                slicing.selectedLayerIndex == static_cast<int>(contour.layerIndex));
            for(std::size_t index = 1; index < contour.points.size(); ++index) {
                const auto& start = contour.points[index - 1];
                const auto& end = contour.points[index];
                preview.push_back({ start.x, start.y, start.z, end.x, end.y, end.z,
                    color.red, color.green, color.blue, color.alpha });
            }
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeSlicingPlanePreview(
        const spray_freeform_traj_plan::FreeformSlicingResult& slicing) const
    {
        FreeformTrajPlanLinePreview preview;
        constexpr int kGridDivisions = 4;
        constexpr double kMinimumPlaneSpanMeters = 0.01;
        const auto subtract = [](const auto& left, const auto& right) {
            return spray_freeform_traj_plan::FreeformVec3{
                left.x - right.x, left.y - right.y, left.z - right.z };
        };
        const auto add = [](const auto& left, const auto& right) {
            return spray_freeform_traj_plan::FreeformVec3{
                left.x + right.x, left.y + right.y, left.z + right.z };
        };
        const auto scale = [](const auto& value, double factor) {
            return spray_freeform_traj_plan::FreeformVec3{
                value.x * factor, value.y * factor, value.z * factor };
        };
        const auto dot = [](const auto& left, const auto& right) {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        };
        const auto cross = [](const auto& left, const auto& right) {
            return spray_freeform_traj_plan::FreeformVec3{
                left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x };
        };
        const auto normalize = [&dot, &scale](const auto& value) {
            const double length = std::sqrt(dot(value, value));
            return length > std::numeric_limits<double>::epsilon()
                ? scale(value, 1.0 / length)
                : spray_freeform_traj_plan::FreeformVec3{ 0.0, 0.0, 1.0 };
        };
        for(std::size_t layerIndex = 0; layerIndex < slicing.planes.size(); ++layerIndex) {
            const auto& plane = slicing.planes[layerIndex];
            const bool selected = slicing.selectedLayerIndex == static_cast<int>(layerIndex);
            const SliceColor color = colorForLayer(layerIndex, selected);
            const auto normal = normalize(plane.normal);
            const auto reference = std::abs(normal.z) < 0.9
                ? spray_freeform_traj_plan::FreeformVec3{ 0.0, 0.0, 1.0 }
                : spray_freeform_traj_plan::FreeformVec3{ 1.0, 0.0, 0.0 };
            const auto axisU = normalize(cross(reference, normal));
            const auto axisV = normalize(cross(normal, axisU));
            double span = kMinimumPlaneSpanMeters;
            for(const auto& contour : slicing.contours) {
                if(contour.layerIndex != layerIndex) {
                    continue;
                }
                for(const auto& point : contour.points) {
                    const auto offset = subtract(point, plane.center);
                    span = std::max(span, 2.0 * std::max(std::abs(dot(offset, axisU)),
                        std::abs(dot(offset, axisV))));
                }
            }
            const double halfSpan = span * 0.58;
            const auto at = [&add, &scale, &plane, &axisU, &axisV](double u, double v) {
                return add(add(plane.center, scale(axisU, u)), scale(axisV, v));
            };
            const auto addLine = [&preview, &color, selected](const auto& start, const auto& end) {
                preview.push_back({ start.x, start.y, start.z, end.x, end.y, end.z,
                    color.red, color.green, color.blue, selected ? 0.82 : 0.34 });
            };
            for(int division = 0; division <= kGridDivisions; ++division) {
                const double fraction = static_cast<double>(division) / kGridDivisions;
                const double position = -halfSpan + 2.0 * halfSpan * fraction;
                addLine(at(position, -halfSpan), at(position, halfSpan));
                addLine(at(-halfSpan, position), at(halfSpan, position));
            }
        }
        return preview;
    }

    FreeformTrajPlanLinePreview FreeformTrajPlanRenderer::makeConnectionPreview(
        const std::vector<spray_freeform_traj_plan::FreeformConnectionSegment>& connections) const
    {
        FreeformTrajPlanLinePreview preview;
        preview.reserve(connections.size());
        for(const auto& connection : connections) {
            preview.push_back({
                connection.start.x, connection.start.y, connection.start.z,
                connection.end.x, connection.end.y, connection.end.z,
                1.0, 0.60, 0.08
            });
        }
        return preview;
    }
}
