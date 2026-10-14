#pragma once

#include <SlicingTrajPlan/PTrajData.h>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iomanip>
#include <locale>

namespace smrobot::workbench::freeform
{
    inline std::string serializeExportTrajectory(
        const std::vector<ModelTrajPlan::TrajectoryPoint>& exportPoints, bool matrixFormat)
    {
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output << std::fixed << std::setprecision(matrixFormat ? 9 : 6);
        for(const auto& point : exportPoints) {
            if(matrixFormat) {
                for(int row = 0; row < 4; ++row) {
                    for(int col = 0; col < 4; ++col) {
                        if(col) output << ", ";
                        output << point.pose(row,col);
                    }
                    output << '\n';
                }
                output << point.timestamp << "\n\n";
            } else {
                const auto position = point.pose.block<3,1>(0,3);
                const Eigen::Quaterniond q = Eigen::Quaterniond(point.pose.block<3,3>(0,0)).normalized();
                output << position.x() << ' ' << position.y() << ' ' << position.z() << ' '
                    << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
                    << point.velocity << ' ' << point.acceleration << ' ' << point.timestamp << ' '
                    << point.point_type << ' ' << point.spray_distance << '\n';
            }
        }
        return output.str();
    }

    // Work on an export copy; planning and shared prediction data remain unchanged.
    inline std::vector<ModelTrajPlan::TrajectoryPoint> resampleExportTrajectory(
        const std::vector<ModelTrajPlan::TrajectoryPoint>& input, double interval)
    {
        if(!std::isfinite(interval) || interval <= 0)
            throw std::runtime_error("Invalid interpolation interval");
        std::vector<ModelTrajPlan::TrajectoryPoint> points;
        for(const auto& p : input) {
            if(!std::isfinite(p.timestamp) || !p.pose.allFinite()
                || !std::isfinite(p.velocity) || !std::isfinite(p.acceleration)
                || !std::isfinite(p.spray_distance))
                throw std::runtime_error("Non-finite trajectory data");
            if(!points.empty() && p.timestamp < points.back().timestamp)
                throw std::runtime_error("Trajectory time runs backwards");
            if(!points.empty() && p.timestamp == points.back().timestamp) points.back() = p;
            else points.push_back(p);
        }
        if(points.size() < 2) throw std::runtime_error("At least two distinct timestamps are required");
        const double start = points.front().timestamp, end = points.back().timestamp;
        const double count = (end - start) / interval;
        if(!std::isfinite(count) || count > 10000000)
            throw std::runtime_error("Interpolation would produce more than ten million points");
        std::vector<ModelTrajPlan::TrajectoryPoint> result;
        result.push_back(points.front());
        std::size_t segment = 0;
        for(std::size_t i = 1; ; ++i) {
            const double t = start + static_cast<double>(i) * interval;
            if(t >= end - interval * 1e-9) break;
            while(segment + 1 < points.size() && points[segment + 1].timestamp <= t) ++segment;
            const auto& a = points[segment]; const auto& b = points[segment + 1];
            const double u = (t - a.timestamp) / (b.timestamp - a.timestamp);
            auto p = a; // Discrete process flags follow the left endpoint, switching at the exact boundary.
            p.timestamp = t;
            p.pose = Eigen::Matrix4d::Identity();
            p.pose.block<3,1>(0,3) = (1-u)*a.pose.block<3,1>(0,3) + u*b.pose.block<3,1>(0,3);
            Eigen::Quaterniond qa(a.pose.block<3,3>(0,0)), qb(b.pose.block<3,3>(0,0));
            qa.normalize(); qb.normalize();
            if(qa.dot(qb) < 0) qb.coeffs() *= -1;
            Eigen::Quaterniond orientation;
            if(qa.dot(qb) > 0.9995) {
                orientation.coeffs() = (1-u)*qa.coeffs() + u*qb.coeffs();
                orientation.normalize();
            } else {
                orientation = qa.slerp(u, qb).normalized();
            }
            p.pose.block<3,3>(0,0) = orientation.toRotationMatrix();
            p.velocity = (1-u)*a.velocity + u*b.velocity;
            p.acceleration = (1-u)*a.acceleration + u*b.acceleration;
            p.spray_distance = (1-u)*a.spray_distance + u*b.spray_distance;
            result.push_back(p);
        }
        result.push_back(points.back());
        return result;
    }
}
