#include "FreeformTrajPlanRenderer.h"
#include "../FreeformPreviewTestViewport.h"
#include <iostream>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/BurTrajPlan.h>
#include <SlicingTrajPlan/TrajectoryFrame.h>
#include <SlicingTrajPlan/BoundaryPlaneGeometry.h>
#include <SlicingTrajPlan/BoundaryDetector.h>
int main(int argc, char** argv)
{
    {
        ModelTrajPlan::BoundaryLine circle, ellipse, square, second;
        const Eigen::Matrix3d rotation = Eigen::AngleAxisd(1.1, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
        const Eigen::Vector3d translation(120, -75, 32);
        for(int i = 0; i < 80; ++i) {
            const double angle = 2 * std::acos(-1.0) * i / 80;
            ModelTrajPlan::BasicPoint p;
            p.position = rotation * Eigen::Vector3d(10*std::cos(angle), 10*std::sin(angle), 0) + translation;
            circle.points.push_back(p);
            p.position = rotation * Eigen::Vector3d(20*std::cos(angle), 10*std::sin(angle), 0) + translation;
            ellipse.points.push_back(p);
            const double x = std::cos(angle), y = std::sin(angle);
            p.position = rotation * Eigen::Vector3d(10*x/std::max(std::abs(x),std::abs(y)), 10*y/std::max(std::abs(x),std::abs(y)), 0) + translation;
            square.points.push_back(p);
        }
        Eigen::Vector3d center;
        if(!ModelTrajPlan::fitBoundaryCircle(circle, center) || !center.isApprox(translation, 1e-8) ||
            ModelTrajPlan::fitBoundaryCircle(ellipse, center) || ModelTrajPlan::fitBoundaryCircle(square, center)) return 40;
        circle.center = translation;
        ellipse.center = translation;
        second = circle;
        const Eigen::Vector3d shift = rotation * Eigen::Vector3d(3, 0, 20);
        for(auto& p : second.points) p.position += shift;
        second.center = translation + shift;
        Eigen::Vector3d normal;
        if(!ModelTrajPlan::circularBoundaryPlane(circle, second, circle.center, second.center, normal) ||
            std::abs(normal.dot(shift)) > 1e-8) return 41;
        ModelTrajPlan::BoundaryDetector detector;
        ModelTrajPlan::ModelData emptyModel;
        ModelTrajPlan::Epiphyseal intersections;
        Eigen::Vector3d n1 = Eigen::Vector3d::Zero(), n2 = Eigen::Vector3d::Zero();
        detector.calculatePlaneIntersection(emptyModel, ellipse, second, intersections, &n1);
        detector.calculatePlaneIntersection(emptyModel, second, ellipse, intersections, &n2);
        if(n1.norm() < 0.9 || n2.norm() < 0.9 || std::abs(n1.dot(n2)) < 1-1e-8 ||
            std::abs(n1.dot(shift)) > 1e-8) return 42;
        std::cout << "Tilted circles, noncircle classification and mixed-rim order independence valid.\n";
    }
    for(int n = 0; n < 6; ++n) for(int f = 0; f < 6; ++f) {
        if(!ModelTrajPlan::validTrajectoryAxes(n, f)) continue;
        const Eigen::Matrix3d frame = ModelTrajPlan::trajectoryFrameMapping(n, f);
        if(!(frame.transpose() * frame).isApprox(Eigen::Matrix3d::Identity()) ||
            std::abs(frame.determinant() - 1.0) > 1e-12 ||
            !((n % 2 ? -1.0 : 1.0) * frame.col(n / 2)).isApprox(Eigen::Vector3d::UnitZ()) ||
            !((f % 2 ? -1.0 : 1.0) * frame.col(f / 2)).isApprox(Eigen::Vector3d::UnitX())) return 30;
    }
    using namespace spray_freeform_traj_plan;
    smrobot::workbench::freeform::FreeformTrajPlanRenderer renderer;
    if(argc > 2 && std::string(argv[1]) == "--model") {
        ModelTrajPlan::GeneralData document;
        ModelTrajPlan::STLProcessor loader;
        ModelTrajPlan::BurTrajPlan plan;
        if(!loader.preprocessSTL(argv[2], document.modelDataManager_)) return 2;
        document.processingState_.isSTLProcessed = true;
        document.slicingData_.managedParams_.maxSliceCount = 8;
        if(!plan.generateCuttingPlanes(document)) return 3;
        const auto& references = document.boundaryData_.referenceLines_;
        if (references.size() != 3) return 9; // two open sides and one center, no caps
        const auto& left = references[0].points;
        const auto& right = references[1].points; // displayed upper to lower
        const auto& middle = references[2].points;
        if (left.size() < 2 || right.size() < 2 || middle.size() < 2) return 10;
        for (const auto* side : { &left, &right }) {
            if (side->size() < 5) return 13;
            const size_t n = side->size();
            const Eigen::Vector3d firstEdge = (*side)[1].position - (*side)[0].position;
            const Eigen::Vector3d firstRun = (*side)[4].position - (*side)[1].position;
            const Eigen::Vector3d lastEdge = (*side)[n - 1].position - (*side)[n - 2].position;
            const Eigen::Vector3d lastRun = (*side)[n - 2].position - (*side)[n - 5].position;
            if (firstEdge.normalized().dot(firstRun.normalized()) < 0.5 ||
                lastEdge.normalized().dot(lastRun.normalized()) < 0.5) {
                std::cerr << "Side contour has a sharp terminal cap connection.\n";
                return 14;
            }
        }
        const double lowerError = (middle.front().position -
            0.5 * (left.front().position + right.back().position)).norm();
        const double upperError = (middle.back().position -
            0.5 * (left.back().position + right.front().position)).norm();
        if (lowerError > 1e-7 || upperError > 1e-7) return 11;
        const auto& boundary = document.boundaryData_;
        Eigen::Vector3d height = boundary.upperBoundary_.center - boundary.lowerBoundary_.center;
        const Eigen::Vector3d normal = boundary.phaseReferencePlaneNormal_.normalized();
        height = (height - normal * height.dot(normal)).normalized();
        const Eigen::Vector3d lateral = height.cross(normal).normalized();
        for (const auto& ends : {std::make_pair(&left.front(), &right.back()),
                                 std::make_pair(&left.back(), &right.front())}) {
            // A cap mistakenly retained in both scan envelopes collapses their
            // endpoints at the global height extreme, despite a nonzero rim width.
            if (std::abs((ends.second->position - ends.first->position).dot(lateral)) <= 1e-4) return 12;
        }
        std::cout << "Open side endpoints: lower midpoint error=" << lowerError
                  << " upper midpoint error=" << upperError << std::endl;
        const auto& nodes = document.boundaryData_.Mainslicingnode;
        const auto& planes = document.slicingData_.slicingLocations_;
        if(nodes.size() != 3 || planes.empty() || planes.size() > 8) return 5;
        for(size_t i = 0; i < nodes[2].size(); ++i) {
            if((nodes[2][i].position - 0.5 * (nodes[0][i].position +
                nodes[1][i].position)).norm() > 1e-7) return 6;
        }
        if(nodes[2].size() > 2) {
            if(planes.size() != nodes[2].size() - 2) return 7;
            for(size_t i = 0; i < planes.size(); ++i) {
                if((planes[i].point - nodes[2][i + 1].position).norm() > 1e-7 ||
                    std::abs(planes[i].normal.dot(nodes[2][i + 1].tangent.normalized())) < 0.99999)
                    return 8;
            }
        }
        std::cout << "Center correspondence and plane tangents valid; planes=" << planes.size() << std::endl;
        if(!plan.generateMATLABTrajectory(document, 12, 3.0, 2.0, 1.0)) return 15;
        const auto& sampled = document.trajData_.sampledBoundaryPoints_;
        if(sampled.empty() || sampled.front().size() < 5) return 16;
        for(const auto& column : sampled) {
            const Eigen::Vector3d lowerDirection = column[1] - column[2];
            const Eigen::Vector3d upperDirection = column[column.size() - 2] -
                column[column.size() - 3];
            if(lowerDirection.norm() <= 1e-8 || upperDirection.norm() <= 1e-8 ||
                (column[0] - column[1]).dot(lowerDirection) <= 0.0 ||
                (column.back() - column[column.size() - 2]).dot(upperDirection) <= 0.0) {
                std::cerr << "Boundary offset direction is incorrect.\n";
                return 17;
            }
        }
        if(!plan.connectTrajectoryPoints(document, ModelTrajPlan::ConnectionStrategy::BowPath)) return 18;
        if(document.trajData_.connectionData_.connectedTrajectory_.empty()) return 19;
        std::cout << "Boundary offsets and generated layer order valid.\n";
        const auto originalPoses = document.trajData_.trajectoryPoints_;
        document.trajData_.generationParams_.normalAxis = 1;
        document.trajData_.generationParams_.forwardAxis = 2;
        if(!plan.generateMATLABTrajectory(document, 12, 3.0, 2.0, 1.0)) return 31;
        const auto& remapped = document.trajData_.trajectoryPoints_;
        if(remapped.size() != originalPoses.size()) return 32;
        for(size_t i = 0; i < remapped.size(); ++i) {
            if(!remapped[i].pose.block<3, 1>(0, 3).isApprox(originalPoses[i].pose.block<3, 1>(0, 3)) ||
                !remapped[i].pose.block<3, 1>(0, 0).isApprox(-originalPoses[i].pose.block<3, 1>(0, 2)) ||
                !remapped[i].pose.block<3, 1>(0, 1).isApprox(originalPoses[i].pose.block<3, 1>(0, 0))) return 33;
        }
        const Eigen::Matrix4d expectedFirst = remapped.front().pose;
        auto previewDocument = document;
        previewDocument.trajData_.generationParams_.previewFirstPoint = true;
        if(!plan.generateMATLABTrajectory(previewDocument, 12, 3.0, 2.0, 1.0) ||
            previewDocument.trajData_.trajectoryPoints_.size() != 1 ||
            !previewDocument.trajData_.trajectoryPoints_.front().pose.isApprox(expectedFirst)) return 34;
        plan.clear(previewDocument);
        if(previewDocument.trajData_.generationParams_.normalAxis != 1 ||
            previewDocument.trajData_.generationParams_.forwardAxis != 2) return 35;
        std::cout << "Axis mappings, unchanged positions, single-point preview and clear preservation valid.\n";
        const auto lines = renderer.makeReferenceLinePreview(document);
        smrobot::visualization::CustomMeshDesc desc;
        desc.ownerId = "freeformTrajPlan.referenceLines";
        desc.meshId = "referenceLines";
        desc.mesh.normalMode = smrobot::visualization::MeshNormalMode::GenerateFlat;
        for(auto segment : lines) {
            segment.startX *= 0.001; segment.startY *= 0.001; segment.startZ *= 0.001;
            segment.endX *= 0.001; segment.endY *= 0.001; segment.endZ *= 0.001;
            freeform_preview_test::appendLine(desc.mesh, segment, 0.0025f);
        }
        smrobot::visualization::CustomMeshDesc normalized;
        const auto result = smrobot::visualization::validateCustomMeshDesc(desc, normalized);
        std::cout << "Reference polylines=" << document.boundaryData_.referenceLines_.size()
            << " segments=" << lines.size() << " mesh accepted=" << result.success
            << " message=" << result.message << std::endl;
        return !lines.empty() && result.success ? 0 : 4;
    }
    FreeformBoundaryResult boundaries;
    boundaries.upper.points = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 1.0, 1.0, 0.0 } };
    boundaries.upper.color = { 1.0, 0.0, 0.0, 1.0 };
    const auto preview = renderer.makeBoundaryPreview(boundaries);
    if(preview.size() != 3 || preview.front().red != 1.0 || preview.back().endX != 0.0) {
        std::cerr << "Boundary line preview conversion failed.\n";
        return 1;
    }
    if(freeform_preview_test::shouldShowViewport(argc, argv)) {
        freeform_preview_test::showPreview(argc, argv, QStringLiteral("Freeform: boundary lines"),
            nullptr, { { "boundaries", preview } });
    }
    return 0;
}
