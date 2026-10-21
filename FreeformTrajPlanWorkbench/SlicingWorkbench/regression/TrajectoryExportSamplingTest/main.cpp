#include "../../TrajectoryExportSampling.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <SlicingTrajPlan/TrajectoryExporter.h>
#include <limits>

using namespace smrobot::workbench::freeform;
using ModelTrajPlan::TrajectoryPoint;

void require(bool condition, const char* message)
{
    if(!condition) throw std::runtime_error(message);
}

int main()
{
    try {
        TrajectoryPoint a, b;
        a.timestamp = 3.0; b.timestamp = 4.0;
        b.pose(0,3) = 100;
        b.pose.block<3,3>(0,0) = Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        a.velocity = 10; b.velocity = 30;
        a.spray_distance = 50; b.spray_distance = 100;
        a.point_type = 1; b.point_type = 2;
        auto sampled = resampleExportTrajectory({a,b}, 0.25);
        require(sampled.size() == 5, "wrong uniform sample count");
        require(std::abs(sampled[2].timestamp-3.5) < 1e-12, "wrong time origin");
        require(std::abs(sampled[2].pose(0,3)-50) < 1e-12, "wrong position interpolation");
        const Eigen::Matrix3d expected = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        require(sampled[2].pose.block<3,3>(0,0).isApprox(expected), "wrong SLERP");
        require(sampled[2].velocity == 20 && sampled[2].spray_distance == 75, "wrong scalar interpolation");
        require(sampled[3].point_type == 2 && sampled.back().point_type == 2, "wrong discrete state");
        const auto irregular = resampleExportTrajectory({a,b}, 0.3);
        require(irregular.size() == 5 && irregular.back().timestamp == 4.0, "endpoint missing");
        auto duplicate = a; duplicate.pose(0,3) = 20;
        require(resampleExportTrajectory({a,duplicate,b}, 0.5).front().pose(0,3) == 20, "duplicate must keep last");
        auto middle = a; middle.timestamp = 3.5; middle.point_type = 2;
        require(resampleExportTrajectory({a,middle,b}, 0.5)[1].point_type == 2, "boundary state did not switch");
        bool rejected = false;
        try { resampleExportTrajectory({b,a}, 0.1); } catch(...) { rejected = true; }
        require(rejected, "backwards time accepted");
        rejected = false;
        try { resampleExportTrajectory({a,b}, 0); } catch(...) { rejected = true; }
        require(rejected, "zero interval accepted");
        auto invalid = b; invalid.timestamp = std::numeric_limits<double>::quiet_NaN();
        rejected = false;
        try { resampleExportTrajectory({a,invalid}, 0.1); } catch(...) { rejected = true; }
        require(rejected, "non-finite time accepted");
        auto farRotation = b;
        farRotation.pose.block<3,3>(0,0) = Eigen::AngleAxisd(4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const auto shortest = resampleExportTrajectory({a,farRotation}, 0.5);
        const Eigen::Matrix3d shortestExpected = Eigen::AngleAxisd(2.0-M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        require(shortest[1].pose.block<3,3>(0,0).isApprox(shortestExpected), "SLERP did not take shortest path");
        require(resampleExportTrajectory({a,b}, 2.0).size() == 2, "large interval must retain both endpoints");
        const std::vector<TrajectoryPoint> original{a,b};
        const std::string paq = serializeExportTrajectory(original, false);
        std::istringstream project(paq);
        for(const auto& source : original) {
            double columns[12];
            for(auto& value : columns) require(bool(project >> value), "PAQ must have twelve columns");
            require(columns[0] == source.pose(0,3) && columns[9] == source.timestamp, "PAQ position/time changed");
        }
        require(paq.find("3.000000") != std::string::npos, "wrong PAQ precision");
        std::string matrix = serializeExportTrajectory(original, true);
        for(auto& c : matrix) if(c == ',') c = ' ';
        std::istringstream blocks(matrix);
        for(const auto& source : original) {
            for(int row=0; row<4; ++row) for(int col=0; col<4; ++col) {
                double value; require(bool(blocks >> value), "matrix must contain sixteen values");
                require(std::abs(value-source.pose(row,col)) < 1e-9, "matrix element changed");
            }
            double timestamp; require(bool(blocks >> timestamp) && timestamp == source.timestamp, "matrix time changed");
        }
        require(matrix.find("4.000000000") != std::string::npos, "wrong matrix precision");
        require(original[0].pose(0,3) == 0 && original.size() == 2, "input was mutated");
        auto normal = a; normal.point_type = 0;
        auto boundary = b; boundary.point_type = 1;
        const auto tags = resampleExportTrajectory({normal,boundary}, 0.25);
        require(tags[3].point_type == 0 && tags.back().point_type == 1, "boundary label leaked to interpolation span");
        auto offGridBoundary = boundary; offGridBoundary.timestamp = 3.3;
        const auto preserved = resampleExportTrajectory({normal,offGridBoundary,b}, 0.25);
        require(std::any_of(preserved.begin(), preserved.end(), [](const TrajectoryPoint& p) {
            return p.timestamp == 3.3 && p.point_type == 1;
        }), "uniform interpolation dropped an exact boundary sample");
        const auto temp = std::filesystem::temp_directory_path() / "rws_point_type_export_test";
        std::filesystem::create_directories(temp);
        const std::vector<TrajectoryPoint> typed{normal,boundary,b,normal};
        const auto abb = (temp / "points.mod").string();
        const auto fanuc = (temp / "points.ls").string();
        const auto txt = (temp / "points.txt").string();
        require(ModelTrajPlan::TrajectoryExporter::exportTrajectoryToFile(typed,txt), "TXT export failed");
        require(ModelTrajPlan::TrajectoryExporter::exportABBProgram(typed,txt,abb), "ABB export failed");
        require(ModelTrajPlan::TrajectoryExporter::exportFANUCProgram(typed,fanuc), "FANUC export failed");
        auto read = [](const std::string& path) {
            std::ifstream stream(path); return std::string(std::istreambuf_iterator<char>(stream), {});
        };
        const auto abbText = read(abb), fanucText = read(fanuc);
        require(abbText.find("! Boundary point") != std::string::npos &&
            abbText.find("! Transition offset point") != std::string::npos, "ABB annotations missing");
        require(abbText.find("MoveJ p1, v10, z100") != std::string::npos &&
            abbText.find("MoveL p2, v30, z100") != std::string::npos &&
            abbText.find("MoveL p4, v10, z100") != std::string::npos,
            "ABB first point must be joint motion, followed by linear motion");
        require(fanucText.find("L P[2] 30mm/sec CNT100") != std::string::npos,
            "FANUC subsequent points must remain linear motion");
        require(abbText.find("MoveL p3, v30, z100") != std::string::npos,
            "Transition offset point unexpectedly forces an ABB stop");
        require(fanucText.find("3:  J P[1] 10% CNT100") != std::string::npos &&
            fanucText.find("4:  ! P[2] Boundary point") != std::string::npos &&
            fanucText.find("6:  ! P[3] Transition offset point") != std::string::npos &&
            fanucText.find("LINE_COUNT\t= 8;") != std::string::npos, "FANUC annotations or line numbers incorrect");
        std::istringstream typedTxt(read(txt));
        for(const auto& point : typed) {
            double columns[12]; for(auto& value : columns) require(bool(typedTxt >> value), "missing TXT column");
            require(columns[10] == point.point_type, "TXT point type incorrect");
        }
        std::filesystem::remove_all(temp);
        std::cout << "Trajectory export sampling checks passed\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
