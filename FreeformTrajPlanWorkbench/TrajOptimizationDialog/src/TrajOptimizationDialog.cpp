#include <TrajOptimizationDialog/TrajOptimizationDialog.h>
#include "ui_TrajOptimizationDialog.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRegExp>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QProgressBar>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QDateTime>
#include <QXmlStreamReader>
#include <QApplication>
#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <data_path.h>
#include <CollisionDetection/CollisionDetection.h>
#include <SMRobotTangQing/irobot.h>
#include <SMRobotTangQing/iloader.h>
#include <SMRobotTangQing/robot_factory.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <TrajOptimization/TrajOptimization.h>
#include <TrajOptimization/TrajectoryPlaybackController.h>
namespace ModelTrajPlan
{
    namespace
    {
        double averageVelocity(const TolerancedTrajectoryPoint& point)
        {
            const double velocity = (point.vel_range(0) + point.vel_range(1)) * 0.5;
            return velocity > 1e-6 ? velocity : 50.0;
        }

        Eigen::Quaterniond normalizedQuaternionFromPose(const Eigen::Matrix4d& pose)
        {
            Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
            quat.normalize();
            return quat;
        }

        QString formatDouble(double value, int precision = 3)
        {
            return QString::number(value, 'f', precision);
        }

        struct TrajectoryExecutionPoint {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
            double velocity = 50.0;
            double acceleration = 0.0;
            double timestamp = 0.0;
            int pointType = 0;
            double sprayDistance = 0.0;
        };

        std::map<double, TolerancedTrajectoryPoint> selectTrajectoryForExport(
            const TrajOptimizationData& data,
            const Eigen::Matrix4d& matrix,
            QString* sourceLabel = nullptr)
        {
            if (!data.tolerancedTrajectory_.empty()) {
                if (sourceLabel) {
                    *sourceLabel = QStringLiteral("optimized toleranced trajectory");
                }
                return data.tolerancedTrajectory_;
            }

            if (!data.transformedTrajectory_.empty()) {
                if (sourceLabel) {
                    *sourceLabel = QStringLiteral("transformed original trajectory");
                }
                return data.transformedTrajectory_;
            }

            std::map<double, TolerancedTrajectoryPoint> transformedOriginal;
            for (const auto& item : data.originalTrajectory_) {
                TolerancedTrajectoryPoint point = item.second;
                point.pose = matrix * point.pose;
                transformedOriginal[item.first] = point;
            }
            if (sourceLabel) {
                *sourceLabel = QStringLiteral("original trajectory");
            }
            return transformedOriginal;
        }

        std::vector<TrajectoryExecutionPoint> buildExecutionPoints(
            const std::map<double, TolerancedTrajectoryPoint>& trajectory)
        {
            std::vector<TrajectoryExecutionPoint> exportPoints;
            exportPoints.reserve(trajectory.size());

            double accumulatedTime = 0.0;
            Eigen::Vector3d previousPosition = Eigen::Vector3d::Zero();
            bool isFirstPoint = true;

            for (const auto& item : trajectory) {
                const TolerancedTrajectoryPoint& point = item.second;
                TrajectoryExecutionPoint out;
                out.position = point.pose.block<3, 1>(0, 3);
                out.quaternion = normalizedQuaternionFromPose(point.pose);
                out.velocity = averageVelocity(point);
                out.sprayDistance = point.spray_distance;

                if (!isFirstPoint) {
                    const double distance = (out.position - previousPosition).norm();
                    accumulatedTime += distance / std::max(out.velocity, 1e-6);
                }

                out.timestamp = accumulatedTime;
                previousPosition = out.position;
                isFirstPoint = false;
                exportPoints.push_back(out);
            }

            return exportPoints;
        }

        bool writePaqAndModFiles(
            const std::map<double, TolerancedTrajectoryPoint>& trajectory,
            const QString& paqPath,
            const QString& modPath,
            const QString& sourceLabel,
            QStringList& exportedFiles)
        {
            if (trajectory.empty()) {
                return false;
            }

            QDir().mkpath(QFileInfo(paqPath).absolutePath());
            QDir().mkpath(QFileInfo(modPath).absolutePath());

            const std::vector<TrajectoryExecutionPoint> exportPoints = buildExecutionPoints(trajectory);

            std::ofstream paq(paqPath.toStdString());
            if (!paq.is_open()) {
                return false;
            }

            paq << std::fixed;
            for (const TrajectoryExecutionPoint& point : exportPoints) {
                paq << std::setprecision(3)
                    << point.position.x() << " "
                    << point.position.y() << " "
                    << point.position.z() << " "
                    << point.quaternion.w() << " "
                    << point.quaternion.x() << " "
                    << point.quaternion.y() << " "
                    << point.quaternion.z() << " "
                    << point.velocity << " "
                    << point.acceleration << " "
                    << std::setprecision(6) << point.timestamp << " "
                    << point.pointType << " "
                    << std::setprecision(3) << point.sprayDistance << "\n";
            }
            paq.close();
            exportedFiles << paqPath;

            std::ofstream mod(modPath.toStdString());
            if (!mod.is_open()) {
                return false;
            }

            mod << "MODULE MainModule\n";
            mod << "  ! Auto-generated by Spraymaster trajectory optimization\n";
            mod << "  ! Source: " << sourceLabel.toStdString() << "\n";
            mod << "  ! Points: " << exportPoints.size() << "\n";
            mod << "    CONST confdata cfDefault := [0,0,0,0];\n";
            mod << "    CONST extjoint extDefault := [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09];\n";

            for (size_t i = 0; i < exportPoints.size(); ++i) {
                const TrajectoryExecutionPoint& point = exportPoints[i];
                mod << "  CONST robtarget p" << (i + 1) << " := [["
                    << formatDouble(point.position.x()).toStdString() << ","
                    << formatDouble(point.position.y()).toStdString() << ","
                    << formatDouble(point.position.z()).toStdString() << "],["
                    << formatDouble(point.quaternion.w()).toStdString() << ","
                    << formatDouble(point.quaternion.x()).toStdString() << ","
                    << formatDouble(point.quaternion.y()).toStdString() << ","
                    << formatDouble(point.quaternion.z()).toStdString()
                    << "],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
            }

            mod << "\n  PROC main()\n";
            mod << "    ConfL\\Off;\n";
            mod << "    SingArea\\Wrist;\n";
            for (size_t i = 0; i < exportPoints.size(); ++i) {
                const TrajectoryExecutionPoint& point = exportPoints[i];
                int speed = static_cast<int>(std::round(point.velocity));
                if (speed < 1) {
                    speed = 50;
                }
                const bool mustStop = point.pointType == 2 ||
                    i + 1 == exportPoints.size();
                const char* zone = mustStop ? "fine" : "z10";
                mod << "    MoveL p" << (i + 1) << ", v" << speed << ", " << zone << ", tool0;\n";
            }
            mod << "  ENDPROC\n";
            mod << "ENDMODULE\n";
            mod.close();
            exportedFiles << modPath;

            return true;
        }

        Eigen::Matrix4d matrixFromTranslationRotation(
            double tx,
            double ty,
            double tz,
            double rxDeg,
            double ryDeg,
            double rzDeg)
        {
            constexpr double pi = 3.14159265358979323846;
            const double rx = rxDeg * pi / 180.0;
            const double ry = ryDeg * pi / 180.0;
            const double rz = rzDeg * pi / 180.0;

            Eigen::AngleAxisd rotX(rx, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd rotY(ry, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rotZ(rz, Eigen::Vector3d::UnitZ());

            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            matrix.block<3, 3>(0, 0) = (rotZ * rotY * rotX).toRotationMatrix();
            matrix.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);
            return matrix;
        }

        Eigen::Vector3d eulerDegreesFromMatrix(const Eigen::Matrix4d& matrix)
        {
            const Eigen::Vector3d radians = matrix.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
            return Eigen::Vector3d(
                radians[2] * 180.0 / 3.14159265358979323846,
                radians[1] * 180.0 / 3.14159265358979323846,
                radians[0] * 180.0 / 3.14159265358979323846);
        }

        QString jsonString(const QString& value)
        {
            QJsonDocument doc(QJsonArray{ value });
            QString encoded = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
            encoded.chop(1);
            return encoded.mid(1);
        }

        QString relativePathFrom(const QDir& baseDir, const QString& path)
        {
            return QDir::toNativeSeparators(baseDir.relativeFilePath(path));
        }

        bool copyFileReplacing(const QString& source, const QString& destination)
        {
            if (source.isEmpty() || !QFileInfo::exists(source)) {
                return false;
            }

            QDir().mkpath(QFileInfo(destination).absolutePath());
            if (QFileInfo::exists(destination)) {
                QFile::remove(destination);
            }
            return QFile::copy(source, destination);
        }

        QStringList extractGeometryReferences(const QString& xmlPath)
        {
            QStringList geometryFiles;
            QFile file(xmlPath);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                return geometryFiles;
            }

            QXmlStreamReader xml(&file);
            while (!xml.atEnd()) {
                xml.readNext();
                if (xml.isStartElement() && xml.name() == QStringLiteral("GeometryFile")) {
                    const QString name = xml.attributes().value(QStringLiteral("name")).toString();
                    if (!name.isEmpty()) {
                        geometryFiles << name;
                    }
                }
            }

            return geometryFiles;
        }

        bool transformBinaryStlFile(const QString& sourcePath, const QString& destinationPath, const Eigen::Matrix4d& matrix)
        {
            std::ifstream input(sourcePath.toStdString(), std::ios::binary);
            if (!input.is_open()) {
                return false;
            }

            char header[80] = {};
            input.read(header, sizeof(header));
            if (!input.good()) {
                return false;
            }

            uint32_t triangleCount = 0;
            input.read(reinterpret_cast<char*>(&triangleCount), sizeof(triangleCount));
            if (!input.good()) {
                return false;
            }

            QDir().mkpath(QFileInfo(destinationPath).absolutePath());
            std::ofstream output(destinationPath.toStdString(), std::ios::binary | std::ios::trunc);
            if (!output.is_open()) {
                return false;
            }

            output.write(header, sizeof(header));
            output.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));

            const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
            for (uint32_t i = 0; i < triangleCount; ++i) {
                float normal[3] = {};
                float vertices[9] = {};
                uint16_t attributeByteCount = 0;

                input.read(reinterpret_cast<char*>(normal), sizeof(normal));
                input.read(reinterpret_cast<char*>(vertices), sizeof(vertices));
                input.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(attributeByteCount));
                if (!input.good()) {
                    return false;
                }

                Eigen::Vector3d transformedNormal(normal[0], normal[1], normal[2]);
                transformedNormal = rotation * transformedNormal;
                if (transformedNormal.norm() > 1e-9) {
                    transformedNormal.normalize();
                }
                normal[0] = static_cast<float>(transformedNormal.x());
                normal[1] = static_cast<float>(transformedNormal.y());
                normal[2] = static_cast<float>(transformedNormal.z());

                for (int vertex = 0; vertex < 3; ++vertex) {
                    Eigen::Vector4d point(
                        vertices[vertex * 3 + 0],
                        vertices[vertex * 3 + 1],
                        vertices[vertex * 3 + 2],
                        1.0);
                    point = matrix * point;
                    vertices[vertex * 3 + 0] = static_cast<float>(point.x());
                    vertices[vertex * 3 + 1] = static_cast<float>(point.y());
                    vertices[vertex * 3 + 2] = static_cast<float>(point.z());
                }

                output.write(reinterpret_cast<const char*>(normal), sizeof(normal));
                output.write(reinterpret_cast<const char*>(vertices), sizeof(vertices));
                output.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(attributeByteCount));
            }

            return input.good() || input.eof();
        }

        bool isAsciiStlFile(const QString& sourcePath)
        {
            QFile file(sourcePath);
            if (!file.open(QIODevice::ReadOnly)) {
                return false;
            }

            const QByteArray head = file.read(512).trimmed();
            if (!head.startsWith("solid")) {
                return false;
            }

            return head.contains("facet") || head.contains('\n');
        }

        bool transformAsciiStlFile(const QString& sourcePath, const QString& destinationPath, const Eigen::Matrix4d& matrix)
        {
            std::ifstream input(sourcePath.toStdString());
            if (!input.is_open()) {
                return false;
            }

            QDir().mkpath(QFileInfo(destinationPath).absolutePath());
            std::ofstream output(destinationPath.toStdString(), std::ios::trunc);
            if (!output.is_open()) {
                return false;
            }

            const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
            std::string line;
            while (std::getline(input, line)) {
                const std::string leading = line.substr(0, line.find_first_not_of(" \t") == std::string::npos ? 0 : line.find_first_not_of(" \t"));
                std::istringstream iss(line);
                std::string word1;
                iss >> word1;

                if (word1 == "facet") {
                    std::string word2;
                    double nx = 0.0;
                    double ny = 0.0;
                    double nz = 0.0;
                    if (iss >> word2 >> nx >> ny >> nz && word2 == "normal") {
                        Eigen::Vector3d normal = rotation * Eigen::Vector3d(nx, ny, nz);
                        if (normal.norm() > 1e-9) {
                            normal.normalize();
                        }
                        output << leading << "facet normal "
                               << std::setprecision(9) << normal.x() << " "
                               << normal.y() << " "
                               << normal.z() << "\n";
                        continue;
                    }
                }
                else if (word1 == "vertex") {
                    double x = 0.0;
                    double y = 0.0;
                    double z = 0.0;
                    if (iss >> x >> y >> z) {
                        Eigen::Vector4d point(x, y, z, 1.0);
                        point = matrix * point;
                        output << leading << "vertex "
                               << std::setprecision(9) << point.x() << " "
                               << point.y() << " "
                               << point.z() << "\n";
                        continue;
                    }
                }

                output << line << "\n";
            }

            return true;
        }

        bool copyOrTransformGeometryFile(const QString& sourcePath, const QString& destinationPath, const Eigen::Matrix4d& matrix, bool* transformed)
        {
            if (transformed) {
                *transformed = false;
            }

            const QString suffix = QFileInfo(sourcePath).suffix();
            if (suffix.compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
                const bool ok = isAsciiStlFile(sourcePath)
                    ? transformAsciiStlFile(sourcePath, destinationPath, matrix)
                    : transformBinaryStlFile(sourcePath, destinationPath, matrix);
                if (ok) {
                    if (transformed) {
                        *transformed = true;
                    }
                    return true;
                }
            }

            return copyFileReplacing(sourcePath, destinationPath);
        }

        bool copyModelFileSet(
            const QString& sourcePath,
            const QString& destinationDir,
            const QString& destinationFileName,
            QString* copiedPath)
        {
            if (sourcePath.isEmpty() || !QFileInfo::exists(sourcePath)) {
                return false;
            }

            QDir().mkpath(destinationDir);
            const QString destinationPath = QDir(destinationDir).filePath(destinationFileName);
            if (!copyFileReplacing(sourcePath, destinationPath)) {
                return false;
            }

            const QFileInfo sourceInfo(sourcePath);
            if (sourceInfo.suffix().compare(QStringLiteral("xml"), Qt::CaseInsensitive) == 0) {
                const QDir sourceDir(sourceInfo.absolutePath());
                const QStringList references = extractGeometryReferences(sourcePath);
                for (const QString& reference : references) {
                    const QString referencedSource = sourceDir.filePath(reference);
                    const QString referencedDestination = QDir(destinationDir).filePath(QFileInfo(reference).fileName());
                    copyFileReplacing(referencedSource, referencedDestination);
                }
            }

            if (copiedPath) {
                *copiedPath = destinationPath;
            }
            return true;
        }

        bool writeTransformedXml(const QString& sourceXmlPath, const QString& destinationXmlPath, const Eigen::Matrix4d& matrix)
        {
            QFile input(sourceXmlPath);
            if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
                return false;
            }

            QString content = QString::fromUtf8(input.readAll());
            input.close();

            const QString rotation = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9")
                .arg(matrix(0, 0), 0, 'g', 12)
                .arg(matrix(0, 1), 0, 'g', 12)
                .arg(matrix(0, 2), 0, 'g', 12)
                .arg(matrix(1, 0), 0, 'g', 12)
                .arg(matrix(1, 1), 0, 'g', 12)
                .arg(matrix(1, 2), 0, 'g', 12)
                .arg(matrix(2, 0), 0, 'g', 12)
                .arg(matrix(2, 1), 0, 'g', 12)
                .arg(matrix(2, 2), 0, 'g', 12);
            const QString translation = QString("%1 %2 %3")
                .arg(matrix(0, 3), 0, 'g', 12)
                .arg(matrix(1, 3), 0, 'g', 12)
                .arg(matrix(2, 3), 0, 'g', 12);

            const QRegExp rotationRx("<Rotation>[^<]*</Rotation>");
            const QRegExp translationRx("<Translation>[^<]*</Translation>");
            if (content.contains(rotationRx)) {
                content.replace(rotationRx, QString("<Rotation>%1</Rotation>").arg(rotation));
            }
            if (content.contains(translationRx)) {
                content.replace(translationRx, QString("<Translation>%1</Translation>").arg(translation));
            }

            QDir().mkpath(QFileInfo(destinationXmlPath).absolutePath());
            QFile output(destinationXmlPath);
            if (!output.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
                return false;
            }
            output.write(content.toUtf8());
            return true;
        }

        bool writeIdentityXmlTransform(const QString& sourceXmlPath, const QString& destinationXmlPath)
        {
            return writeTransformedXml(sourceXmlPath, destinationXmlPath, Eigen::Matrix4d::Identity());
        }

        void transformTrajectoryMap(
            std::map<double, TolerancedTrajectoryPoint>& trajectory,
            const Eigen::Matrix4d& matrix)
        {
            for (auto& item : trajectory) {
                item.second.pose = matrix * item.second.pose;
            }
        }

        void transformBackupPointMap(
            std::map<double, std::vector<Eigen::Matrix4d>>& backupPoints,
            const Eigen::Matrix4d& matrix)
        {
            for (auto& item : backupPoints) {
                for (auto& pose : item.second) {
                    pose = matrix * pose;
                }
            }
        }
    }

    TrajOptimizationDialog::TrajOptimizationDialog(QWidget* parent)
        : QDialog(parent)
        , ui(new Ui::TrajOptimizationDialogClass())
        , trajOptData_(nullptr)
        , generalData_(nullptr)
    {
        ui->setupUi(this);
    }

    TrajOptimizationDialog::TrajOptimizationDialog(TrajOptimizationData* trajOptData, GeneralData* generalData, QWidget* parent)
        : QDialog(parent)
        , ui(new Ui::TrajOptimizationDialogClass())
        , trajOptData_(trajOptData)
        , generalData_(generalData)
    {
        ui->setupUi(this);
        if (trajOptData_) {
            initializeSystem();
        }
    }

    TrajOptimizationDialog::~TrajOptimizationDialog()
    {
        // Disable collision visualization before teardown so nothing uses trajOptData_ after close
        if (trajOptData_) {
            TrajOptimizationAlgorithm::setCollisionVisualization(*trajOptData_, false);
            trajOptData_->onVisualizationRefresh = nullptr;
        }

        delete ui;
    }

    void TrajOptimizationDialog::setTrajOptData(TrajOptimizationData* trajOptData)
    {
        trajOptData_ = trajOptData;
        if (trajOptData_) {
            initializeSystem();
        }
    }

    void TrajOptimizationDialog::setGeneralData(GeneralData* generalData)
    {
        generalData_ = generalData;
    }

    void TrajOptimizationDialog::initializeSystem()
    {
        // Initialize without loading robots automatically
        // Users can load robots manually via UI

    }


    void TrajOptimizationDialog::updateRobotListDisplay()
    {
        // TODO: Refresh robot list widgets in the UI
    }

    void TrajOptimizationDialog::showTrajectoryTransformDialog(Eigen::Matrix4d matrix)
    {
        auto* dialog = new QDialog(nullptr);
        dialog->setWindowTitle(tr("Trajectory Coordinate Transform"));
        dialog->setMinimumSize(620, 520);
        dialog->setStyleSheet(
            "QDialog { background-color: #dfe5ea; color: #14212b; }"
            "QLabel { color: #14212b; background: transparent; }"
            "QGroupBox { background-color: #f8fafb; color: #14212b; border: 1px solid #b9c5cc; border-top: 3px solid #0f6f8f; margin-top: 18px; padding: 14px 10px 10px 10px; font-weight: 700; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 3px 10px; background-color: #0f6f8f; color: #ffffff; }"
            "QDoubleSpinBox, QLineEdit { background-color: #ffffff; color: #000000; selection-background-color: #0f6f8f; selection-color: #ffffff; border: 1px solid #9daeb8; border-radius: 2px; padding: 4px 6px; min-height: 26px; }"
            "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { background-color: #eef3f6; border-left: 1px solid #9daeb8; width: 16px; }"
            "QDoubleSpinBox::up-arrow, QDoubleSpinBox::down-arrow { background: transparent; }"
            "QTextEdit { background-color: #f8fafb; border: 1px solid #c7d2da; color: #000000; selection-background-color: #0f6f8f; selection-color: #ffffff; }"
            "QPushButton { background-color: #344957; color: #ffffff; border: 1px solid #263945; border-radius: 2px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background-color: #263945; }");

        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowModality(Qt::NonModal);
        dialog->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                               Qt::WindowSystemMenuHint |
                               Qt::WindowMinimizeButtonHint |
                               Qt::WindowMaximizeButtonHint |
                               Qt::WindowCloseButtonHint);

        auto* mainLayout = new QVBoxLayout(dialog);
        mainLayout->setSpacing(12);
        mainLayout->setContentsMargins(14, 14, 14, 14);

        auto* hintLabel = new QLabel(tr("Input the target coordinate transform matrix. It maps the generated world-origin trajectory, workpiece, and TCP/tool model into the real workpiece coordinate system."), dialog);
        hintLabel->setWordWrap(true);
        hintLabel->setStyleSheet("color: #14212b;");
        mainLayout->addWidget(hintLabel);

        auto* trGroup = new QGroupBox(tr("Translation And Rotation"), dialog);
        auto* trLayout = new QGridLayout(trGroup);
        trLayout->setHorizontalSpacing(12);
        trLayout->setVerticalSpacing(8);

        auto* translationTitle = new QLabel(tr("Translation"), trGroup);
        auto* rotationTitle = new QLabel(tr("Rotation"), trGroup);
        translationTitle->setStyleSheet("font-weight: 700; color: #14212b;");
        rotationTitle->setStyleSheet("font-weight: 700; color: #14212b;");
        trLayout->addWidget(translationTitle, 0, 0, 1, 2);
        trLayout->addWidget(rotationTitle, 0, 2, 1, 2);

        std::array<QDoubleSpinBox*, 3> translationSpins = {};
        std::array<QDoubleSpinBox*, 3> rotationSpins = {};
        const QStringList axisLabels = { "X:", "Y:", "Z:" };
        for (int i = 0; i < 3; ++i) {
            auto* transLabel = new QLabel(axisLabels[i], trGroup);
            transLabel->setStyleSheet("color: #14212b;");
            auto* transSpin = new QDoubleSpinBox(trGroup);
            transSpin->setDecimals(3);
            transSpin->setRange(-1000000.0, 1000000.0);
            transSpin->setSingleStep(1.0);
            transSpin->setSuffix(" mm");
            transSpin->setMinimumWidth(150);
            transSpin->setStyleSheet("background-color: #ffffff; color: #000000;");
            translationSpins[i] = transSpin;
            trLayout->addWidget(transLabel, i + 1, 0);
            trLayout->addWidget(transSpin, i + 1, 1);

            auto* rotLabel = new QLabel(axisLabels[i], trGroup);
            rotLabel->setStyleSheet("color: #14212b;");
            auto* rotSpin = new QDoubleSpinBox(trGroup);
            rotSpin->setDecimals(2);
            rotSpin->setRange(-360.0, 360.0);
            rotSpin->setSingleStep(1.0);
            rotSpin->setSuffix(" deg");
            rotSpin->setMinimumWidth(150);
            rotSpin->setStyleSheet("background-color: #ffffff; color: #000000;");
            rotationSpins[i] = rotSpin;
            trLayout->addWidget(rotLabel, i + 1, 2);
            trLayout->addWidget(rotSpin, i + 1, 3);
        }
        mainLayout->addWidget(trGroup);

        auto* matrixGroup = new QGroupBox(tr("4x4 Homogeneous Transform Matrix"), dialog);
        auto* matrixLayout = new QGridLayout(matrixGroup);
        matrixLayout->setHorizontalSpacing(8);
        matrixLayout->setVerticalSpacing(8);

        std::array<std::array<QDoubleSpinBox*, 4>, 4> cells = {};
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                auto* spin = new QDoubleSpinBox(matrixGroup);
                spin->setDecimals(6);
                spin->setRange(-1000000.0, 1000000.0);
                spin->setSingleStep(col == 3 ? 1.0 : 0.01);
                spin->setMinimumWidth(120);
                spin->setValue(matrix(row, col));
                spin->setStyleSheet("background-color: #ffffff; color: #000000;");
                cells[row][col] = spin;
                matrixLayout->addWidget(spin, row, col);
            }
        }
        mainLayout->addWidget(matrixGroup);

        auto* presetLayout = new QGridLayout();
        auto* identityButton = new QPushButton(tr("Identity"), dialog);
        auto* currentButton = new QPushButton(tr("Load Current"), dialog);
        auto* loadJsonButton = new QPushButton(tr("Load Matrix JSON"), dialog);
        auto* pasteButton = new QPushButton(tr("Parse Text Matrix"), dialog);
        presetLayout->addWidget(identityButton, 0, 0);
        presetLayout->addWidget(currentButton, 0, 1);
        presetLayout->addWidget(loadJsonButton, 0, 2);
        presetLayout->addWidget(pasteButton, 0, 3);
        mainLayout->addLayout(presetLayout);

        auto* textEdit = new QTextEdit(dialog);
        textEdit->setPlaceholderText(tr("Optional: paste 16 numbers, separated by spaces/commas/new lines."));
        textEdit->setMaximumHeight(82);
        textEdit->setStyleSheet("background-color: #f8fafb; color: #000000; border: 1px solid #c7d2da;");
        mainLayout->addWidget(textEdit);

        auto* extraStlGroup = new QGroupBox(tr("Optional Extra STL Transform Export"), dialog);
        auto* extraStlLayout = new QGridLayout(extraStlGroup);
        auto* extraStlEdit = new QLineEdit(extraStlGroup);
        extraStlEdit->setReadOnly(true);
        extraStlEdit->setPlaceholderText(tr("Optional: select an additional STL file to export with this transform."));
        extraStlEdit->setStyleSheet("background-color: #ffffff; color: #000000;");
        auto* browseExtraStlButton = new QPushButton(tr("Browse STL..."), extraStlGroup);
        auto* clearExtraStlButton = new QPushButton(tr("Clear"), extraStlGroup);
        auto* extraStlLabel = new QLabel(tr("Extra STL"), extraStlGroup);
        extraStlLabel->setStyleSheet("color: #14212b;");
        extraStlLayout->addWidget(extraStlLabel, 0, 0);
        extraStlLayout->addWidget(extraStlEdit, 0, 1);
        extraStlLayout->addWidget(browseExtraStlButton, 0, 2);
        extraStlLayout->addWidget(clearExtraStlButton, 0, 3);
        mainLayout->addWidget(extraStlGroup);

        auto matrixState = std::make_shared<Eigen::Matrix4d>(matrix);
        auto syncingTransformInputs = std::make_shared<bool>(false);

        auto setCellsFromMatrix = [cells](const Eigen::Matrix4d& source) {
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    cells[row][col]->setValue(source(row, col));
                }
            }
        };

        auto setTranslationRotationFromMatrix = [translationSpins, rotationSpins](const Eigen::Matrix4d& source) {
            const Eigen::Vector3d translation = source.block<3, 1>(0, 3);
            const Eigen::Vector3d rotation = eulerDegreesFromMatrix(source);
            for (int i = 0; i < 3; ++i) {
                translationSpins[i]->setValue(translation[i]);
                rotationSpins[i]->setValue(rotation[i]);
            }
        };

        auto setAllInputsFromMatrix = [syncingTransformInputs, setCellsFromMatrix, setTranslationRotationFromMatrix](const Eigen::Matrix4d& source) {
            *syncingTransformInputs = true;
            setCellsFromMatrix(source);
            setTranslationRotationFromMatrix(source);
            *syncingTransformInputs = false;
        };

        auto matrixFromTranslationRotationInputs = [translationSpins, rotationSpins]() {
            return matrixFromTranslationRotation(
                translationSpins[0]->value(),
                translationSpins[1]->value(),
                translationSpins[2]->value(),
                rotationSpins[0]->value(),
                rotationSpins[1]->value(),
                rotationSpins[2]->value());
        };

        auto syncMatrixFromTranslationRotation = [syncingTransformInputs, setCellsFromMatrix, matrixFromTranslationRotationInputs]() {
            if (*syncingTransformInputs) {
                return;
            }
            *syncingTransformInputs = true;
            setCellsFromMatrix(matrixFromTranslationRotationInputs());
            *syncingTransformInputs = false;
        };

        auto syncTranslationRotationFromCells = [syncingTransformInputs, cells, setTranslationRotationFromMatrix]() {
            if (*syncingTransformInputs) {
                return;
            }
            Eigen::Matrix4d source = Eigen::Matrix4d::Identity();
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    source(row, col) = cells[row][col]->value();
                }
            }
            *syncingTransformInputs = true;
            setTranslationRotationFromMatrix(source);
            *syncingTransformInputs = false;
        };

        setAllInputsFromMatrix(*matrixState);

        for (int i = 0; i < 3; ++i) {
            connect(translationSpins[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), dialog, [syncMatrixFromTranslationRotation](double) {
                syncMatrixFromTranslationRotation();
            });
            connect(rotationSpins[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), dialog, [syncMatrixFromTranslationRotation](double) {
                syncMatrixFromTranslationRotation();
            });
        }

        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                connect(cells[row][col], QOverload<double>::of(&QDoubleSpinBox::valueChanged), dialog, [syncTranslationRotationFromCells](double) {
                    syncTranslationRotationFromCells();
                });
            }
        }

        connect(identityButton, &QPushButton::clicked, dialog, [setAllInputsFromMatrix]() {
            setAllInputsFromMatrix(Eigen::Matrix4d::Identity());
        });

        connect(currentButton, &QPushButton::clicked, dialog, [this, setAllInputsFromMatrix]() {
            setAllInputsFromMatrix(trajOptData_ ? trajOptData_->transformMatrix_ : Eigen::Matrix4d::Identity());
        });

        connect(loadJsonButton, &QPushButton::clicked, dialog, [dialog, setAllInputsFromMatrix]() {
            std::string dataPath = DATA_PATH;
            const QString fileName = QFileDialog::getOpenFileName(
                dialog,
                tr("Select transform_export.json"),
                QString::fromStdString(dataPath),
                tr("Transform JSON (*.json);;All Files (*.*)"));
            if (fileName.isEmpty()) {
                return;
            }

            QFile file(fileName);
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QMessageBox::warning(dialog, tr("Load Failed"), tr("Failed to open selected JSON file."));
                return;
            }

            QJsonParseError parseError;
            const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
            if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
                QMessageBox::warning(dialog, tr("Load Failed"), tr("Invalid JSON file."));
                return;
            }

            const QJsonValue matrixValue = document.object().value(QStringLiteral("transform_matrix"));
            if (!matrixValue.isArray() || matrixValue.toArray().size() != 4) {
                QMessageBox::warning(dialog, tr("Load Failed"), tr("JSON does not contain a valid transform_matrix array."));
                return;
            }

            Eigen::Matrix4d loaded = Eigen::Matrix4d::Identity();
            const QJsonArray rows = matrixValue.toArray();
            for (int row = 0; row < 4; ++row) {
                const QJsonValue rowValue = rows.at(row);
                if (!rowValue.isArray() || rowValue.toArray().size() != 4) {
                    QMessageBox::warning(dialog, tr("Load Failed"), tr("transform_matrix must be a 4x4 numeric array."));
                    return;
                }

                const QJsonArray cols = rowValue.toArray();
                for (int col = 0; col < 4; ++col) {
                    if (!cols.at(col).isDouble()) {
                        QMessageBox::warning(dialog, tr("Load Failed"), tr("transform_matrix contains non-numeric values."));
                        return;
                    }
                    loaded(row, col) = cols.at(col).toDouble();
                }
            }

            if (std::abs(loaded(3, 3)) < 1e-9) {
                QMessageBox::warning(dialog, tr("Invalid Matrix"), tr("The matrix bottom-right value cannot be zero."));
                return;
            }

            setAllInputsFromMatrix(loaded / loaded(3, 3));
        });

        connect(pasteButton, &QPushButton::clicked, dialog, [dialog, textEdit, setAllInputsFromMatrix]() {
            QString normalized = textEdit->toPlainText();
            normalized.replace(',', ' ');
            normalized.replace(';', ' ');
            QStringList tokens = normalized.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
            if (tokens.size() != 16) {
                QMessageBox::warning(dialog, tr("Invalid Matrix"), tr("Please provide exactly 16 numeric values."));
                return;
            }

            Eigen::Matrix4d parsed = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 16; ++i) {
                bool ok = false;
                const double value = tokens[i].toDouble(&ok);
                if (!ok) {
                    QMessageBox::warning(dialog, tr("Invalid Matrix"), tr("Matrix contains non-numeric values."));
                    return;
                }
                parsed(i / 4, i % 4) = value;
            }
            setAllInputsFromMatrix(parsed);
        });

        connect(browseExtraStlButton, &QPushButton::clicked, dialog, [dialog, extraStlEdit]() {
            std::string dataPath = DATA_PATH;
            const QString fileName = QFileDialog::getOpenFileName(
                dialog,
                tr("Select Extra STL File"),
                QString::fromStdString(dataPath),
                tr("STL Files (*.stl *.STL);;All Files (*.*)"));
            if (!fileName.isEmpty()) {
                extraStlEdit->setText(fileName);
            }
        });

        connect(clearExtraStlButton, &QPushButton::clicked, dialog, [extraStlEdit]() {
            extraStlEdit->clear();
        });

        auto* actionLayout = new QHBoxLayout();
        auto* applyButton = new QPushButton(tr("Apply Transform"), dialog);
        auto* exportButton = new QPushButton(tr("Export Transformed Set"), dialog);
        auto* closeButton = new QPushButton(tr("Close"), dialog);
        actionLayout->addStretch();
        actionLayout->addWidget(exportButton);
        actionLayout->addWidget(applyButton);
        actionLayout->addWidget(closeButton);
        mainLayout->addLayout(actionLayout);

        auto readMatrixFromCells = [cells]() {
            Eigen::Matrix4d current = Eigen::Matrix4d::Identity();
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    current(row, col) = cells[row][col]->value();
                }
            }
            return current;
        };

        connect(applyButton, &QPushButton::clicked, dialog, [this, dialog, readMatrixFromCells, matrixState]() {
            *matrixState = readMatrixFromCells();

            if (std::abs((*matrixState)(3, 3)) < 1e-9) {
                QMessageBox::warning(dialog, tr("Invalid Matrix"), tr("The matrix bottom-right value cannot be zero."));
                return;
            }

            applyTrajectoryTransform(*matrixState);
        });
        connect(exportButton, &QPushButton::clicked, dialog, [this, dialog, readMatrixFromCells, matrixState, extraStlEdit]() {
            *matrixState = readMatrixFromCells();

            if (std::abs((*matrixState)(3, 3)) < 1e-9) {
                QMessageBox::warning(dialog, tr("Invalid Matrix"), tr("The matrix bottom-right value cannot be zero."));
                return;
            }

            if (!applyTrajectoryTransform(*matrixState)) {
                return;
            }

            std::string dataPath = DATA_PATH;
            const QString parentDir = QFileDialog::getExistingDirectory(
                dialog,
                tr("Select Export Directory"),
                QString::fromStdString(dataPath),
                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
            if (parentDir.isEmpty()) {
                return;
            }

            const QString exportDir = QDir(parentDir).filePath(
                "TransformedSet_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
            QStringList exportedFiles;
            if (!exportTransformedInputSet(exportDir, *matrixState, extraStlEdit->text(), exportedFiles)) {
                QMessageBox::warning(dialog, tr("Export Failed"), tr("Failed to export transformed workpiece, tool, or trajectory."));
                return;
            }

            QMessageBox::information(dialog, tr("Export Complete"),
                tr("Transformed input set exported:\n%1").arg(QDir::toNativeSeparators(exportDir)));
        });
        connect(closeButton, &QPushButton::clicked, dialog, &QDialog::close);

        dialog->show();
        return;
    }

    bool TrajOptimizationDialog::applyTrajectoryTransform(const Eigen::Matrix4d& matrix)
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData is not initialized."));
            return false;
        }

        const bool hasTrajectory = !trajOptData_->originalTrajectory_.empty();
        const bool hasWorkpiece = static_cast<bool>(trajOptData_->workpieceRobot_);
        const bool hasTool = static_cast<bool>(trajOptData_->toolRobot_);
        if (!hasTrajectory && !hasWorkpiece && !hasTool) {
            QMessageBox::warning(this, tr("Error"), tr("Please load a workpiece, tool, or trajectory before applying coordinate transform."));
            return false;
        }

        const Eigen::Matrix4d previousMatrix = trajOptData_->transformMatrix_;
        TrajOptimizationAlgorithm::setTransformMatrix(*trajOptData_, matrix);
        if (hasTrajectory) {
            if (!TrajOptimizationAlgorithm::transformTrajectory(*trajOptData_)) {
                QMessageBox::warning(this, tr("Error"), tr("Failed to transform trajectory."));
                return false;
            }
        }

        Eigen::Matrix4d delta = matrix;
        if (std::abs(previousMatrix.determinant()) > 1e-9) {
            delta = matrix * previousMatrix.inverse();
        }

        const bool hadOptimizedTrajectory = !trajOptData_->tolerancedTrajectory_.empty();
        if (hadOptimizedTrajectory) {
            transformTrajectoryMap(trajOptData_->tolerancedTrajectory_, delta);
            transformBackupPointMap(trajOptData_->rrtBackupPoints_, delta);
        }

        const glm::mat4 frameDelta = CollisionDetection::eigenMatrixToGlm(delta);
        for (auto& frame : trajOptData_->trajectoryFrameModels_) {
            if (frame) {
                frame->set_matrix(frameDelta * frame->get_matrix());
            }
        }

        const glm::mat4 base = CollisionDetection::eigenMatrixToGlm(matrix);
        if (hasWorkpiece) {
            trajOptData_->workpieceRobot_->set_base_mat(base);
        }
        if (hasTool) {
            trajOptData_->toolRobot_->set_base_mat(base);
        }

        trajOptData_->trajectoryFrameNeedsUpdate_ = hasTrajectory && trajOptData_->trajectoryFrameModels_.empty();
        trajOptData_->trajectoryVisible_ = hasTrajectory;

        updateTrajectoryInfoDisplay();
        if (hadOptimizedTrajectory) {
            updateToleranceStatistics(0.0);
        }
        else {
            ui->safePointsLabel->setText("Safe Points: 0");
            ui->collisionPointsLabel->setText("Collision Points: 0");
            ui->safetyRateLabel->setText("Safety Rate: 0%");
            ui->avgToleranceLabel->setText("Avg Tolerance: Translation +-0.0 mm, Rotation +-0.0 deg");
            ui->avgCalculationTimeLabel->setText("Avg Time: 0.0 ms/point");
        }

        if (forceRefreshHook_) {
            forceRefreshHook_();
        }

        return true;
    }

    void TrajOptimizationDialog::showTrajectoryPlaybackDialog()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData is not initialized."));
            return;
        }

        auto* dialog = new QDialog(nullptr);
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowModality(Qt::NonModal);
        dialog->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                               Qt::WindowSystemMenuHint |
                               Qt::WindowMinimizeButtonHint |
                               Qt::WindowMaximizeButtonHint |
                               Qt::WindowCloseButtonHint);
        dialog->setWindowTitle(tr("Trajectory Playback"));
        dialog->setMinimumSize(720, 420);
        dialog->setStyleSheet(
            "QDialog { background-color: #dfe5ea; }"
            "QGroupBox { background-color: #f8fafb; border: 1px solid #b9c5cc; border-top: 3px solid #0f6f8f; margin-top: 18px; padding: 14px 10px 10px 10px; font-weight: 700; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 3px 10px; background-color: #0f6f8f; color: #ffffff; }"
            "QLabel { color: #14212b; }"
            "QPushButton { background-color: #344957; color: #ffffff; border: 1px solid #263945; border-radius: 2px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background-color: #263945; }"
            "QLineEdit, QComboBox { background-color: #ffffff; color: #14212b; border: 1px solid #9daeb8; border-radius: 2px; padding: 5px 8px; min-height: 26px; }"
            "QComboBox QAbstractItemView { background-color: #ffffff; color: #14212b; selection-background-color: #0f6f8f; selection-color: #ffffff; }"
            "QProgressBar { border: 1px solid #9daeb8; border-radius: 2px; background-color: #dbe4e9; text-align: center; color: #14212b; min-height: 24px; font-weight: 700; }"
            "QProgressBar::chunk { background-color: #0f6f8f; border-radius: 1px; }");

        auto* controller = new TrajectoryPlaybackController();
        auto* timer = new QTimer(dialog);
        timer->setInterval(10);

        connect(dialog, &QDialog::destroyed, this, [controller]() {
            delete controller;
        });

        auto* mainLayout = new QVBoxLayout(dialog);
        mainLayout->setSpacing(12);
        mainLayout->setContentsMargins(14, 14, 14, 14);

        auto* fileGroup = new QGroupBox(tr("01 Playback Trajectory Source"), dialog);
        auto* fileLayout = new QGridLayout(fileGroup);
        auto* pathEdit = new QLineEdit(fileGroup);
        pathEdit->setReadOnly(true);
        pathEdit->setPlaceholderText(tr("Select new_PAQ.txt trajectory file"));
        auto* browseButton = new QPushButton(tr("Browse..."), fileGroup);
        auto* loadButton = new QPushButton(tr("Load PAQ"), fileGroup);
        fileLayout->addWidget(new QLabel(tr("File"), fileGroup), 0, 0);
        fileLayout->addWidget(pathEdit, 0, 1);
        fileLayout->addWidget(browseButton, 0, 2);
        fileLayout->addWidget(loadButton, 0, 3);
        mainLayout->addWidget(fileGroup);

        auto* controlGroup = new QGroupBox(tr("02 Playback Control"), dialog);
        auto* controlLayout = new QGridLayout(controlGroup);
        auto* speedCombo = new QComboBox(controlGroup);
        speedCombo->addItem("0.5x", 0.5);
        speedCombo->addItem("1x", 1.0);
        speedCombo->addItem("2x", 2.0);
        speedCombo->addItem("5x", 5.0);
        speedCombo->addItem("10x", 10.0);
        speedCombo->setCurrentIndex(1);

        auto* playPauseButton = new QPushButton(tr("Play"), controlGroup);
        auto* nextPointButton = new QPushButton(tr("Next Point"), controlGroup);
        auto* resetButton = new QPushButton(tr("Reset"), controlGroup);
        auto* progressBar = new QProgressBar(controlGroup);
        auto* stateLabel = new QLabel(tr("No PAQ loaded"), controlGroup);
        auto* poseLabel = new QLabel(tr("Position: --   Quaternion: --"), controlGroup);
        stateLabel->setMinimumWidth(150);

        controlLayout->addWidget(new QLabel(tr("Speed"), controlGroup), 0, 0);
        controlLayout->addWidget(speedCombo, 0, 1);
        controlLayout->addWidget(playPauseButton, 0, 2);
        controlLayout->addWidget(nextPointButton, 0, 3);
        controlLayout->addWidget(resetButton, 0, 4);
        controlLayout->addWidget(progressBar, 1, 0, 1, 4);
        controlLayout->addWidget(stateLabel, 1, 4);
        controlLayout->addWidget(poseLabel, 2, 0, 1, 5);
        mainLayout->addWidget(controlGroup);

        auto* hintLabel = new QLabel(
            tr("Playback applies each new_PAQ pose to the loaded tool/TCP model. Load workpiece and tool first, then play or step through the trajectory."),
            dialog);
        hintLabel->setWordWrap(true);
        hintLabel->setStyleSheet("color: #14212b;");
        mainLayout->addWidget(hintLabel);

        auto* closeButton = new QPushButton(tr("Close"), dialog);
        auto* closeLayout = new QHBoxLayout();
        closeLayout->addStretch();
        closeLayout->addWidget(closeButton);
        mainLayout->addLayout(closeLayout);

        auto refreshUi = std::make_shared<std::function<void()>>();
        *refreshUi = [=]() {
            const int total = static_cast<int>(controller->pointCount());
            const int index = total > 0 ? static_cast<int>(controller->currentPointIndex()) + 1 : 0;
            progressBar->setValue(static_cast<int>(controller->currentProgress() * 100.0));
            stateLabel->setText(QString("%1 / %2").arg(index).arg(total));

            if (controller->pointCount() > 0) {
                const auto& point = controller->points()[controller->currentPointIndex()];
                poseLabel->setText(QString("Position: %1, %2, %3   Quaternion: %4, %5, %6, %7")
                    .arg(formatDouble(point.position.x()))
                    .arg(formatDouble(point.position.y()))
                    .arg(formatDouble(point.position.z()))
                    .arg(formatDouble(point.orientation.w(), 6))
                    .arg(formatDouble(point.orientation.x(), 6))
                    .arg(formatDouble(point.orientation.y(), 6))
                    .arg(formatDouble(point.orientation.z(), 6)));
            }

            if (forceRefreshHook_) {
                forceRefreshHook_();
            }
        };

        connect(browseButton, &QPushButton::clicked, dialog, [=]() {
            std::string dataPath = DATA_PATH;
            QString fileName = QFileDialog::getOpenFileName(
                dialog,
                tr("Select new_PAQ.txt"),
                QString::fromStdString(dataPath + "/traj"),
                tr("PAQ Trajectory (*.txt);;All Files (*.*)"));
            if (!fileName.isEmpty()) {
                pathEdit->setText(fileName);
            }
        });

        connect(loadButton, &QPushButton::clicked, dialog, [=]() {
            if (pathEdit->text().isEmpty()) {
                QMessageBox::warning(dialog, tr("Error"), tr("Please select new_PAQ.txt first."));
                return;
            }

            std::string error;
            if (!controller->loadPaqFile(pathEdit->text().toStdString(), &error)) {
                QMessageBox::warning(dialog, tr("Load Failed"), QString::fromStdString(error));
                return;
            }

            controller->setSpeedScale(speedCombo->currentData().toDouble());
            if (!controller->applyCurrentPose(*trajOptData_)) {
                QMessageBox::warning(dialog, tr("Error"), tr("Failed to apply pose. Please load the tool model first."));
                return;
            }

            timer->stop();
            playPauseButton->setText(tr("Play"));
            (*refreshUi)();
        });

        connect(speedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), dialog, [=](int) {
            controller->setSpeedScale(speedCombo->currentData().toDouble());
        });

        connect(playPauseButton, &QPushButton::clicked, dialog, [=]() {
            if (!controller->isLoaded()) {
                QMessageBox::warning(dialog, tr("Error"), tr("Please load new_PAQ.txt first."));
                return;
            }

            if (timer->isActive()) {
                timer->stop();
                playPauseButton->setText(tr("Play"));
            }
            else {
                timer->start();
                playPauseButton->setText(tr("Pause"));
            }
        });

        connect(nextPointButton, &QPushButton::clicked, dialog, [=]() {
            timer->stop();
            playPauseButton->setText(tr("Play"));
            if (!controller->stepToNextPoint(*trajOptData_)) {
                QMessageBox::warning(dialog, tr("Error"), tr("Please load new_PAQ.txt first."));
                return;
            }
            (*refreshUi)();
        });

        connect(resetButton, &QPushButton::clicked, dialog, [=]() {
            timer->stop();
            playPauseButton->setText(tr("Play"));
            controller->reset();
            if (controller->isLoaded()) {
                controller->applyCurrentPose(*trajOptData_);
            }
            (*refreshUi)();
        });

        connect(timer, &QTimer::timeout, dialog, [=]() {
            if (!controller->stepByTime(*trajOptData_, timer->interval() / 1000.0)) {
                timer->stop();
                playPauseButton->setText(tr("Play"));
                return;
            }

            if (trajOptData_->m_collision_visualization_enabled) {
                CollisionDetection::updateCollisionColors(*trajOptData_);
            }

            if (controller->isFinished()) {
                timer->stop();
                playPauseButton->setText(tr("Play"));
            }
            (*refreshUi)();
        });

        connect(closeButton, &QPushButton::clicked, dialog, &QDialog::close);
        dialog->show();
    }

    void TrajOptimizationDialog::updateTrajectoryInfoDisplay()
    {
        if (!trajOptData_) {
            return;
        }

        // Update trajectory point count display
        size_t originalCount = trajOptData_->originalTrajectory_.size();
        size_t transformedCount = trajOptData_->transformedTrajectory_.size();
        size_t tolerancedCount = trajOptData_->tolerancedTrajectory_.size();

        // Update trajectory point count label in UI
        QString trajectoryStatus;
        if (originalCount > 0) {
            trajectoryStatus = QString("Trajectory Points: %1  Status: Loaded").arg(originalCount);
        }
        else {
            trajectoryStatus = "Trajectory Points: 0  Status: Not Loaded";
        }
        ui->trajectoryPointCountLabel->setText(trajectoryStatus);

        // Update generated trajectory point count label (after tolerance calculation)
        QString generatedStatus;
        if (tolerancedCount > 0) {
            generatedStatus = QString("Generated Trajectory Points: %1  Status: Generated").arg(tolerancedCount);
        }
        else {
            generatedStatus = "Generated Trajectory Points: 0  Status: Not Generated";
        }
        ui->generatedTrajectoryPointCountLabel->setText(generatedStatus);
        ui->trajectoryStatusLabel->setText(QString("Transform: %1  Optimized: %2")
            .arg(transformedCount > 0 ? "Applied" : "Not Applied")
            .arg(tolerancedCount > 0 ? "Ready" : "Not Ready"));

    }

    // Trajectory related slot functions
    void TrajOptimizationDialog::onTrajectoryFromFileToggled(bool checked)
    {
        ui->trajectoryFileLineEdit->setEnabled(checked);
        ui->browseTrajectoryButton->setEnabled(checked);
        ui->loadTrajectoryFromFileButton->setEnabled(checked && !ui->trajectoryFileLineEdit->text().isEmpty());
    }

    void TrajOptimizationDialog::onTrajectoryFromSystemToggled(bool checked)
    {
        ui->systemTrajectoryComboBox->setEnabled(checked);
        ui->refreshSystemTrajectoryButton->setEnabled(checked);
        ui->loadTrajectoryFromSystemButton->setEnabled(checked);
    }

    void TrajOptimizationDialog::onBrowseTrajectoryClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/traj");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Trajectory File"),
            defaultPath,
            tr("Trajectory Files (*.txt *.csv)"));

        if (!fileName.isEmpty()) {
            ui->trajectoryFileLineEdit->setText(fileName);
            ui->loadTrajectoryFromFileButton->setEnabled(true);
        }
    }

    void TrajOptimizationDialog::onLoadTrajectoryFromFileClicked()
    {
        QString fileName = ui->trajectoryFileLineEdit->text();
        if (!fileName.isEmpty()) {
            if (TrajOptimizationAlgorithm::loadTrajectoryFromFile(*trajOptData_, fileName.toStdString())) {
                trajOptData_->trajectoryFilePath_ = fileName.toStdString();

                updateTrajectoryInfoDisplay();
                ui->trajectoryStatusLabel->setText(QString("Loaded file trajectory: %1 points")
                    .arg(trajOptData_->originalTrajectory_.size()));
                if (forceRefreshHook_) {
                    forceRefreshHook_();
                    qDebug() << "[TrajOptDialog] OpenGL view refreshed after loading tool";
                }
            }
            else {
                QMessageBox::warning(this, "Error", "Failed to load trajectory from file!");
            }
        }
    }

    void TrajOptimizationDialog::onRefreshSystemTrajectoryClicked()
    {
        // TODO: Implement refreshing system trajectory list logic
        QMessageBox::information(this, tr("Information"), tr("Refresh system trajectory list feature to be implemented"));
    }

    void TrajOptimizationDialog::onLoadTrajectoryFromSystemClicked()
    {
        if (!generalData_) {
            QMessageBox::warning(this, "Error", "GeneralData not available!");
            return;
        }

        if (TrajOptimizationAlgorithm::loadTrajectoryFromGeneralData(*trajOptData_, *generalData_)) {
            updateTrajectoryInfoDisplay();
            ui->trajectoryStatusLabel->setText(QString("Loaded system trajectory: %1 points")
                .arg(trajOptData_->originalTrajectory_.size()));
            if (forceRefreshHook_) {
                forceRefreshHook_();
                qDebug() << "[TrajOptDialog] OpenGL view refreshed after loading tool";
            }
        }
        else {
            QMessageBox::warning(this, "Error", "Failed to load trajectory from system!");
        }
    }

    // Robot import slot functions
    void TrajOptimizationDialog::onBrowseRobotFileClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/model");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Robot Model File"),
            defaultPath,
            tr("Robot Models (*.xml)"));

        if (!fileName.isEmpty()) {
            // Add to robot path list
            robotPaths_.push_back(fileName.toStdString());

            // Update display
            updateRobotListDisplay();

            QMessageBox::information(this, tr("Success"),
                tr("Robot model added: %1").arg(fileName));
        }
    }

    void TrajOptimizationDialog::onLoadRobotFromSystemClicked()
    {
        if (robotPaths_.empty()) {
            QMessageBox::warning(this, tr("Error"), tr("Please add robot model files first!"));
            return;
        }

        // Reinitialize robot system
        if (TrajOptimizationAlgorithm::initializeRobotSystem(*trajOptData_, robotPaths_)) {
            QMessageBox::information(this, tr("Success"),
                tr("Robot system loaded successfully!\n%1 robots loaded.").arg(robotPaths_.size()));
            updateRobotListDisplay();
        }
        else {
            QMessageBox::warning(this, tr("Error"), tr("Failed to load robot system!"));
        }
    }

    // Workpiece and tool slot functions
    void TrajOptimizationDialog::onBrowseWorkpieceClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/model");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Workpiece Model File"),
            defaultPath,
            tr("Model Files (*.xml *.rbt *.RBT *.stl *.obj)"));

        if (!fileName.isEmpty()) {
            ui->workpieceFileLineEdit->setText(fileName);
            ui->loadWorkpieceButton->setEnabled(true);
        }
    }

    void TrajOptimizationDialog::onLoadWorkpieceClicked()
    {
        QString fileName = ui->workpieceFileLineEdit->text();
        if (fileName.isEmpty()) {
            QMessageBox::warning(this, tr("Warning"), tr("Please select a workpiece file first!"));
            return;
        }

        if (!trajOptData_ || !trajOptData_->m_spScene) {
            QMessageBox::warning(this, tr("Error"), tr("Scene not initialized!"));
            return;
        }

        if (trajOptData_->loadWorkpieceModel(fileName.toStdString())) {
            trajOptData_->workpieceModelPath_ = fileName.toStdString();

            if (trajOptData_->workpieceRobot_) {
                glm::mat4 identity(1.0f);
                trajOptData_->workpieceRobot_->set_base_mat(identity);
                qDebug() << "[TrajOptDialog] Workpiece reset to initial position (0, 0, 0)";
            }

            // Check if both workpiece and tool are loaded, then set systemInitialized
            if (trajOptData_->workpieceRobot_ && trajOptData_->toolRobot_) {
                trajOptData_->systemInitialized_ = true;
                qDebug() << "[TrajOptDialog] System initialized: both workpiece and tool loaded";
            }

            // Force refresh OpenGL view to show the loaded model
            if (forceRefreshHook_) {
                forceRefreshHook_();
                qDebug() << "[TrajOptDialog] OpenGL view refreshed after loading workpiece";
            }

            QMessageBox::information(this, tr("Success"),
                tr("Workpiece loaded successfully!\nFile: %1").arg(fileName));
        }
        else {
            QMessageBox::warning(this, tr("Error"),
                tr("Failed to load workpiece!\nFile: %1").arg(fileName));
        }
    }

    void TrajOptimizationDialog::onBrowseToolClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/model");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Tool Model File"),
            defaultPath,
            tr("Model Files (*.xml *.rbt *.RBT *.stl *.obj)"));

        if (!fileName.isEmpty()) {
            ui->toolFileLineEdit->setText(fileName);
            ui->loadToolButton->setEnabled(true);
        }
    }

    void TrajOptimizationDialog::onLoadToolClicked()
    {
        QString fileName = ui->toolFileLineEdit->text();
        if (fileName.isEmpty()) {
            QMessageBox::warning(this, tr("Warning"), tr("Please select a tool file first!"));
            return;
        }

        if (!trajOptData_ || !trajOptData_->m_spScene) {
            QMessageBox::warning(this, tr("Error"), tr("Scene not initialized!"));
            return;
        }

        if (trajOptData_->loadToolModel(fileName.toStdString())) {
            if (trajOptData_->toolRobot_) {
                glm::mat4 identity(1.0f);
                trajOptData_->toolRobot_->set_base_mat(identity);
                qDebug() << "[TrajOptDialog] Tool reset to initial position (0, 0, 0)";
            }

            // Check if both workpiece and tool are loaded, then set systemInitialized
            if (trajOptData_->workpieceRobot_ && trajOptData_->toolRobot_) {
                trajOptData_->systemInitialized_ = true;
                qDebug() << "[TrajOptDialog] System initialized: both workpiece and tool loaded";
            }

            // Force refresh OpenGL view to show the loaded model
            if (forceRefreshHook_) {
                forceRefreshHook_();
                qDebug() << "[TrajOptDialog] OpenGL view refreshed after loading tool";
            }

            QMessageBox::information(this, tr("Success"),
                tr("Tool loaded successfully!\nFile: %1").arg(fileName));
        }
        else {
            QMessageBox::warning(this, tr("Error"),
                tr("Failed to load tool!\nFile: %1").arg(fileName));
        }
    }

    void TrajOptimizationDialog::onClearAllModelsClicked()
    {
        QDialog dialog(this);
        dialog.setWindowTitle(tr("Clear Scene Items"));
        dialog.setMinimumSize(420, 260);
        dialog.setStyleSheet(
            "QDialog { background-color: #dfe5ea; }"
            "QLabel { color: #14212b; }"
            "QCheckBox { color: #14212b; min-height: 24px; }"
            "QPushButton { background-color: #344957; color: #ffffff; border: 1px solid #263945; border-radius: 2px; padding: 6px 14px; font-weight: 600; }"
            "QPushButton#cancelClearButton { background-color: #eef3f6; color: #253845; border: 1px solid #aebbc4; }");

        auto* layout = new QVBoxLayout(&dialog);
        layout->setSpacing(10);
        layout->setContentsMargins(16, 16, 16, 16);

        auto* hintLabel = new QLabel(tr("Select the items to delete from data and the viewport. All items are selected by default."), &dialog);
        hintLabel->setWordWrap(true);
        hintLabel->setStyleSheet("color: #14212b;");
        layout->addWidget(hintLabel);

        auto* trajectoryCheck = new QCheckBox(tr("Trajectory and trajectory display frames"), &dialog);
        auto* workpieceCheck = new QCheckBox(tr("Workpiece model"), &dialog);
        auto* toolCheck = new QCheckBox(tr("Tool / TCP model"), &dialog);
        auto* resultCheck = new QCheckBox(tr("Collision and optimized trajectory results"), &dialog);
        trajectoryCheck->setChecked(true);
        workpieceCheck->setChecked(true);
        toolCheck->setChecked(true);
        resultCheck->setChecked(true);
        layout->addWidget(trajectoryCheck);
        layout->addWidget(workpieceCheck);
        layout->addWidget(toolCheck);
        layout->addWidget(resultCheck);
        layout->addStretch();

        auto* buttonLayout = new QHBoxLayout();
        auto* clearButton = new QPushButton(tr("Clear Selected"), &dialog);
        auto* cancelButton = new QPushButton(tr("Cancel"), &dialog);
        cancelButton->setObjectName("cancelClearButton");
        buttonLayout->addStretch();
        buttonLayout->addWidget(clearButton);
        buttonLayout->addWidget(cancelButton);
        layout->addLayout(buttonLayout);

        connect(clearButton, &QPushButton::clicked, &dialog, &QDialog::accept);
        connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

        if (dialog.exec() != QDialog::Accepted) {
            return;
        }

        if (!trajOptData_) {
            return;
        }

        const bool clearTrajectory = trajectoryCheck->isChecked();
        const bool clearWorkpiece = workpieceCheck->isChecked();
        const bool clearTool = toolCheck->isChecked();
        const bool clearResults = resultCheck->isChecked();

        if (!clearTrajectory && !clearWorkpiece && !clearTool && !clearResults) {
            return;
        }

        qDebug() << "[TrajOptDialog] Clearing selected scene items..."
                 << "trajectory=" << clearTrajectory
                 << "workpiece=" << clearWorkpiece
                 << "tool=" << clearTool
                 << "results=" << clearResults;

        trajOptData_->collisionDetectionRunning_ = false;

        if (trajOptData_->m_collision_visualization_enabled) {
            TrajOptimizationAlgorithm::setCollisionVisualization(*trajOptData_, false);
        }

        trajOptData_->onVisualizationRefresh = nullptr;

        auto removeRobot = [&](irobot_spointer& robot) {
            if (!robot) {
                return;
            }

            trajOptData_->m_robots.erase(
                std::remove(trajOptData_->m_robots.begin(), trajOptData_->m_robots.end(), robot),
                trajOptData_->m_robots.end());
            robot.reset();
        };

        auto clearResultData = [&]() {
            trajOptData_->tolerancedTrajectory_.clear();
            trajOptData_->pointCalculationTimes_.clear();
            trajOptData_->pointCollisionCheckCounts_.clear();
            trajOptData_->unavoidableCollisionPoints_.clear();
            trajOptData_->rrtGeneratedPoints_.clear();
            trajOptData_->rrtBackupPoints_.clear();
            trajOptData_->currentPointCollisionCheckCount_ = 0;
            trajOptData_->totalCollisionCheckTime_ = 0.0;
            trajOptData_->totalCollisionCheckCount_ = 0;
            trajOptData_->m_spDetector.reset();
            trajOptData_->m_robot_models.clear();
        };

        if (clearTrajectory) {
            trajOptData_->originalTrajectory_.clear();
            trajOptData_->transformedTrajectory_.clear();
            trajOptData_->trajectoryFrameModels_.clear();
            trajOptData_->trajectoryFrameData_.m_vertices.clear();
            trajOptData_->trajectoryFrameData_.m_indices.clear();
            trajOptData_->trajectoryFrameNeedsUpdate_ = false;
            trajOptData_->trajectoryVisible_ = false;
            trajOptData_->transformMatrix_ = Eigen::Matrix4d::Identity();
            trajOptData_->trajectoryFilePath_.clear();
            ui->trajectoryFileLineEdit->clear();
            ui->loadTrajectoryFromFileButton->setEnabled(false);
        }

        if (clearWorkpiece) {
            removeRobot(trajOptData_->workpieceRobot_);
            trajOptData_->originalModelPath_.clear();
            trajOptData_->workpieceModelPath_.clear();
            trajOptData_->hasOriginalModel_ = false;
            ui->workpieceFileLineEdit->clear();
            ui->loadWorkpieceButton->setEnabled(false);
            ui->workpieceNameLabel->setText(tr("Workpiece: None"));
            ui->workpieceStatusLabel->setText(tr("Status: Not Loaded"));
        }

        if (clearTool) {
            removeRobot(trajOptData_->toolRobot_);
            trajOptData_->actualModelPath_.clear();
            trajOptData_->hasActualModel_ = false;
            ui->toolFileLineEdit->clear();
            ui->loadToolButton->setEnabled(false);
            ui->toolNameLabel->setText(tr("Tool: None"));
            ui->toolStatusLabel->setText(tr("Status: Not Loaded"));
        }

        if (clearResults || clearTrajectory || clearWorkpiece || clearTool) {
            clearResultData();
            ui->safePointsLabel->setText("Safe Points: 0");
            ui->collisionPointsLabel->setText("Collision Points: 0");
            ui->safetyRateLabel->setText("Safety Rate: 0%");
            ui->collisionProgressBar->setValue(0);
            ui->collisionProgressInfoLabel->setText("0 / 0");
            ui->avgToleranceLabel->setText("Avg Tolerance: Translation +-0.0 mm, Rotation +-0.0 deg");
            ui->avgCalculationTimeLabel->setText("Avg Time: 0.0 ms/point");
        }

        trajOptData_->systemInitialized_ = trajOptData_->workpieceRobot_ && trajOptData_->toolRobot_;

        if (forceRefreshHook_) {
            forceRefreshHook_();
        }

        updateTrajectoryInfoDisplay();
        qDebug() << "[TrajOptDialog] Selected scene items cleared successfully";
    }

    // Collision detection slot functions
    void TrajOptimizationDialog::onStartCollisionDetectionClicked()
    {
        qDebug() << "=== [TrajOptDialog] Starting collision detection ===";

        if (!trajOptData_) {
            QMessageBox::warning(this, "Error", "TrajOptimizationData is not initialized!");
            return;
        }

        QStringList algorithms;
        algorithms << tr("Binary Search (Bisection Method)") << tr("Step Search Method");

        bool ok;
        QString selectedAlgorithm = QInputDialog::getItem(this,
            tr("Select Collision Detection Algorithm"),
            tr("Please select the search algorithm:"),
            algorithms,
            0,
            false,
            &ok);

        if (!ok || selectedAlgorithm.isEmpty()) {
            return;
        }

        bool useBinarySearch = (selectedAlgorithm == algorithms[0]);

        trajOptData_->useBinarySearchMethod_ = useBinarySearch;

        if (!trajOptData_->m_robots.empty()) {
            if (!TrajOptimizationAlgorithm::initCollision(*trajOptData_)) {
                qWarning() << "[TrajOptDialog] Failed to initialize collision detector!";
                QMessageBox::critical(this, "Error", "Failed to initialize collision detector!");
                return;
            }
        }
        else {
            qWarning() << "[TrajOptDialog] No robots loaded for collision detection!";
            QMessageBox::critical(this, "Error", "No robots loaded for collision detection!");
            return;
        }

        if (trajOptData_->originalTrajectory_.empty()) {
            qDebug() << "[TrajOptDialog] ERROR: Original trajectory is empty!";
            QMessageBox::warning(this, "Error",
                "No trajectory available!\n\n"
                "Please load trajectory first:\n"
                "1. Load trajectory from file or system\n"
                "2. Then start collision detection");
            return;
        }

        if (trajOptData_->transformedTrajectory_.empty()) {
            qDebug() << "[TrajOptDialog] No transformed trajectory, using original trajectory directly";
            trajOptData_->transformedTrajectory_ = trajOptData_->originalTrajectory_;
        }

        ToleranceLimitConfig tol_limit;

        double maxTranslation = ui->translationLimitSpinBox->value();  // mm
        double maxRotation = ui->rotationLimitSpinBox->value();  // deg
        tol_limit.max_translation << maxTranslation, maxTranslation, maxTranslation;
        tol_limit.max_rotation << maxRotation * M_PI / 180.0, maxRotation* M_PI / 180.0, maxRotation* M_PI / 180.0;

        tol_limit.position_tolerance = ui->positionToleranceSpinBox->value();  // mm
        tol_limit.rotation_tolerance = ui->rotationToleranceSpinBox->value() * M_PI / 180.0;

        trajOptData_->enableVisualization_ = ui->enableVisualizationCheckBox->isChecked();
        trajOptData_->visualizationDelayMs_ = trajOptData_->enableVisualization_ ? 10 : 0;

        TrajOptimizationAlgorithm::setCollisionVisualization(*trajOptData_, trajOptData_->enableVisualization_);

        trajOptData_->onVisualizationRefresh = [this]() {
            if (forceRefreshHook_) {
                forceRefreshHook_();
            }
            QApplication::processEvents();
        };

        ui->collisionProgressBar->setValue(0);
        ui->collisionProgressInfoLabel->setText("0 / 0");
        ui->safePointsLabel->setText("Safe Points: 0");
        ui->collisionPointsLabel->setText("Collision Points: 0");
        ui->safetyRateLabel->setText("Safety Rate: 0%");
        ui->avgToleranceLabel->setText("Avg Tolerance: Translation +-0.0 mm, Rotation +-0.0 deg");

        ui->startCollisionDetectionButton->setEnabled(false);
        ui->stopCollisionDetectionButton->setEnabled(true);

        auto progressCallback = [this](int current, int total, const std::string& message) {
            ui->collisionProgressBar->setMaximum(total);
            ui->collisionProgressBar->setValue(current);
            ui->collisionProgressInfoLabel->setText(QString("%1 / %2").arg(current).arg(total));

            if (!trajOptData_->tolerancedTrajectory_.empty()) {
                double totalTime = 0.0;
                for (const auto& pair : trajOptData_->pointCalculationTimes_) {
                    totalTime += pair.second;
                }
                double avgTime = totalTime / trajOptData_->pointCalculationTimes_.size();
                updateToleranceStatistics(avgTime);
            }

            QApplication::processEvents();
        };

        auto startTime = std::chrono::high_resolution_clock::now();

        trajOptData_->collisionDetectionRunning_ = true;
        bool success = TrajOptimizationAlgorithm::computeTrajectoryToleranceWithProgress(
            *trajOptData_, tol_limit, progressCallback);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        double totalTimeMs = duration.count();
        int totalPoints = trajOptData_->transformedTrajectory_.size();
        double avgTimePerPoint = (totalPoints > 0) ? (totalTimeMs / totalPoints) : 0.0;


        bool wasStoppedByUser = !trajOptData_->collisionDetectionRunning_;

        trajOptData_->collisionDetectionRunning_ = false;

        ui->startCollisionDetectionButton->setEnabled(true);
        ui->stopCollisionDetectionButton->setEnabled(false);

        if (!trajOptData_->tolerancedTrajectory_.empty()) {
            updateToleranceStatistics(avgTimePerPoint);
        }

        if (success) {
            QMessageBox::information(this, "Success",
                QString("Tolerance calculation completed successfully!\n\n"
                    "%1 points processed.\n"
                    "Average time: %2 ms/point\n"
                    "Total time: %3 seconds\n\n"
                    "Check the statistics below for results.")
                .arg(trajOptData_->tolerancedTrajectory_.size())
                .arg(avgTimePerPoint, 0, 'f', 2)
                .arg(totalTimeMs / 1000.0, 0, 'f', 2));
        }
        else {
            if (wasStoppedByUser) {
                QMessageBox::information(this, "Stopped",
                    QString("Tolerance calculation was stopped by user.\n\n"
                        "%1 points processed before stopping.")
                    .arg(trajOptData_->tolerancedTrajectory_.size()));
            }
            else {
                QMessageBox::warning(this, "Error",
                    "Tolerance calculation failed!\n\n"
                    "Possible reasons:\n"
                    "- No collision detection system initialized\n"
                    "- Invalid trajectory data\n"
                    "- Calculation was interrupted");
            }
        }
    }

    void TrajOptimizationDialog::onStopCollisionDetectionClicked()
    {
        if (trajOptData_) {
            trajOptData_->collisionDetectionRunning_ = false;
        }
        ui->stopCollisionDetectionButton->setEnabled(false);
        ui->startCollisionDetectionButton->setEnabled(true);
    }

    void TrajOptimizationDialog::onOptimizationAlgorithmIndexChanged(int index)
    {
        if (!trajOptData_) {
            return;
        }

        // 0: GA+RRT, 1: parallel GA islands + RRT, 2: ACO
        trajOptData_->useParallelGAMethod_ = (index == 1);
        trajOptData_->useACOMethod_ = (index == 2);
        if (index != 1) {
            trajOptData_->useParallelGAMethod_ = (index == 1);
        }

        qDebug() << "[TrajOptDialog] Optimization algorithm changed to"
            << (trajOptData_->useParallelGAMethod_ ? "Parallel GA Islands + RRT"
                : (trajOptData_->useACOMethod_ ? "Ant Colony Optimization (ACO)"
                    : "Classic GA + Local RRT"));
    }

    void TrajOptimizationDialog::updateToleranceStatistics(double avgTimePerPoint)
    {
        if (!trajOptData_ || trajOptData_->tolerancedTrajectory_.empty()) {
            return;
        }

        if (!trajOptData_->pointCalculationTimes_.empty()) {
            double totalTime = 0.0;
            for (const auto& pair : trajOptData_->pointCalculationTimes_) {
                totalTime += pair.second;
            }
            avgTimePerPoint = totalTime / trajOptData_->pointCalculationTimes_.size();
        }

        int totalPoints = trajOptData_->tolerancedTrajectory_.size();
        int safePoints = 0;
        int collisionPoints = 0;

        double avgPosToleranceMin = 0.0;
        double avgPosToleranceMax = 0.0;
        double avgOriToleranceMin = 0.0;
        double avgOriToleranceMax = 0.0;

        int totalCollisionChecks = 0;
        for (const auto& pair : trajOptData_->pointCollisionCheckCounts_) {
            totalCollisionChecks += pair.second;
        }
        double avgCollisionChecksPerPoint = (totalPoints > 0) ? (static_cast<double>(totalCollisionChecks) / totalPoints) : 0.0;

        double avgCollisionCheckTime = (trajOptData_->totalCollisionCheckCount_ > 0) ?
            (trajOptData_->totalCollisionCheckTime_ / trajOptData_->totalCollisionCheckCount_) : 0.0;

        int point_index = 0;
        for (const auto& pair : trajOptData_->tolerancedTrajectory_) {
            double t = pair.first;
            const auto& pt = pair.second;

            bool is_unavoidable = std::find(trajOptData_->unavoidableCollisionPoints_.begin(),
                trajOptData_->unavoidableCollisionPoints_.end(),
                point_index) != trajOptData_->unavoidableCollisionPoints_.end();

            if (is_unavoidable) {
                collisionPoints++;
            }
            else {
                safePoints++;
            }

            avgPosToleranceMin += pt.pos_tolerance.col(0).mean();
            avgPosToleranceMax += pt.pos_tolerance.col(1).mean();
            avgOriToleranceMin += pt.orient_tolerance.col(0).mean();
            avgOriToleranceMax += pt.orient_tolerance.col(1).mean();

            point_index++;
        }

        avgPosToleranceMin /= totalPoints;
        avgPosToleranceMax /= totalPoints;
        avgOriToleranceMin /= totalPoints * M_PI / 180.0;
        avgOriToleranceMax /= totalPoints * M_PI / 180.0;

        double avgPosTolerance = (std::abs(avgPosToleranceMin) + avgPosToleranceMax) / 2.0;
        double avgOriTolerance = (std::abs(avgOriToleranceMin) + avgOriToleranceMax) / 2.0;

        double safetyRate = (totalPoints > 0) ? (100.0 * safePoints / totalPoints) : 0.0;

        // Update UI with English labels
        ui->safePointsLabel->setText(QString("Safe Points: %1").arg(safePoints));
        ui->collisionPointsLabel->setText(QString("Collision Points: %1").arg(collisionPoints));
        ui->safetyRateLabel->setText(QString("Safety Rate: %1%").arg(safetyRate, 0, 'f', 1));
        ui->avgToleranceLabel->setText(QString("Avg Tolerance: Translation +/-%1 mm, Rotation +/-%2 deg")
            .arg(avgPosTolerance, 0, 'f', 2)
            .arg(avgOriTolerance, 0, 'f', 2));
        ui->avgCalculationTimeLabel->setText(QString("Avg Time: %1 ms/point | Collision Checks: %2/point (Total: %3) | Single Check: %4 ms")
            .arg(avgTimePerPoint, 0, 'f', 2)
            .arg(avgCollisionChecksPerPoint, 0, 'f', 1)
            .arg(totalCollisionChecks)
            .arg(avgCollisionCheckTime, 0, 'f', 4));

    }

    bool TrajOptimizationDialog::exportTrajectoryExecutionFiles(const QString& projectDir, QStringList& exportedFiles)
    {
        if (!trajOptData_) {
            return false;
        }

        QDir dir(projectDir);
        if (!dir.exists() && !dir.mkpath(".")) {
            return false;
        }

        const QString paqPath = dir.filePath("new_PAQ.txt");
        const QString modPath = dir.filePath("MainModule.mod");

        QString sourceLabel;
        const auto trajectoryToExport = selectTrajectoryForExport(
            *trajOptData_,
            trajOptData_->transformMatrix_,
            &sourceLabel);
        return writePaqAndModFiles(trajectoryToExport, paqPath, modPath, sourceLabel, exportedFiles);
    }

    bool TrajOptimizationDialog::exportTransformedInputSet(
        const QString& exportDir,
        const Eigen::Matrix4d& matrix,
        const QString& extraStlPath,
        QStringList& exportedFiles)
    {
        if (!trajOptData_) {
            return false;
        }

        const bool hasOptimizedTrajectory = !trajOptData_->tolerancedTrajectory_.empty();
        const bool hasTrajectory = hasOptimizedTrajectory
            || !trajOptData_->transformedTrajectory_.empty()
            || !trajOptData_->originalTrajectory_.empty();
        const bool hasWorkpiece = !trajOptData_->originalModelPath_.empty();
        const bool hasTool = !trajOptData_->actualModelPath_.empty();
        const bool hasExtraStl = !extraStlPath.isEmpty();
        if (!hasTrajectory && !hasWorkpiece && !hasTool && !hasExtraStl) {
            return false;
        }

        QDir root(exportDir);
        if (!root.exists() && !root.mkpath(".")) {
            return false;
        }

        const QString modelDirPath = root.filePath("model");
        const QString trajectoryDirPath = root.filePath("traj");
        const QString workpieceDirPath = QDir(modelDirPath).filePath("workpiece");
        const QString toolDirPath = QDir(modelDirPath).filePath("tool");
        const QString extraStlDirPath = QDir(modelDirPath).filePath("extra_stl");
        QDir().mkpath(workpieceDirPath);
        QDir().mkpath(toolDirPath);
        QDir().mkpath(trajectoryDirPath);

        QString workpiecePath;
        QString toolPath;
        QString extraStlExportPath;
        bool copiedWorkpiece = true;
        bool copiedTool = true;
        bool workpieceGeometryBaked = false;
        bool toolGeometryBaked = false;
        bool extraStlGeometryBaked = false;

        if (!trajOptData_->originalModelPath_.empty()) {
            const QString source = QString::fromStdString(trajOptData_->originalModelPath_);
            const QFileInfo info(source);
            workpiecePath = QDir(workpieceDirPath).filePath("workpiece." + info.suffix());
            if (info.suffix().compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
                copiedWorkpiece = copyOrTransformGeometryFile(source, workpiecePath, matrix, &workpieceGeometryBaked);
            }
            else {
                copiedWorkpiece = copyModelFileSet(source, workpieceDirPath, QFileInfo(workpiecePath).fileName(), &workpiecePath);
            }
            if (copiedWorkpiece && info.suffix().compare(QStringLiteral("xml"), Qt::CaseInsensitive) == 0) {
                copiedWorkpiece = writeTransformedXml(source, workpiecePath, matrix);
            }
            if (copiedWorkpiece) {
                exportedFiles << workpiecePath;
            }
        }

        if (!trajOptData_->actualModelPath_.empty()) {
            const QString source = QString::fromStdString(trajOptData_->actualModelPath_);
            const QFileInfo info(source);
            toolPath = QDir(toolDirPath).filePath("tool." + info.suffix());
            if (info.suffix().compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
                copiedTool = copyOrTransformGeometryFile(source, toolPath, matrix, &toolGeometryBaked);
            }
            else {
                copiedTool = copyModelFileSet(source, toolDirPath, QFileInfo(toolPath).fileName(), &toolPath);
            }
            if (copiedTool && info.suffix().compare(QStringLiteral("xml"), Qt::CaseInsensitive) == 0) {
                copiedTool = writeTransformedXml(source, toolPath, matrix);
            }
            if (copiedTool) {
                exportedFiles << toolPath;
            }
        }

        if (!extraStlPath.isEmpty()) {
            const QFileInfo extraInfo(extraStlPath);
            if (extraInfo.exists() && extraInfo.suffix().compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
                QDir().mkpath(extraStlDirPath);
                extraStlExportPath = QDir(extraStlDirPath).filePath(extraInfo.completeBaseName() + "_transformed.stl");
                if (!copyOrTransformGeometryFile(extraStlPath, extraStlExportPath, matrix, &extraStlGeometryBaked)) {
                    return false;
                }
                exportedFiles << extraStlExportPath;
            }
        }

        QString paqPath;
        QString modPath;
        if (hasTrajectory) {
            QString sourceLabel;
            const auto trajectoryToExport = selectTrajectoryForExport(*trajOptData_, matrix, &sourceLabel);
            paqPath = QDir(trajectoryDirPath).filePath("new_PAQ.txt");
            modPath = QDir(trajectoryDirPath).filePath("MainModule.mod");
            if (!writePaqAndModFiles(trajectoryToExport, paqPath, modPath, sourceLabel, exportedFiles)) {
                return false;
            }
        }

        const QString configPath = root.filePath("transform_export.json");
        QFile config(configPath);
        if (!config.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            return false;
        }

        QTextStream stream(&config);
        stream.setCodec("UTF-8");
        stream << "{\n";
        stream << "  \"version\": \"1.0\",\n";
        stream << "  \"export_type\": \"transformed_input_set\",\n";
        stream << "  \"export_date\": " << jsonString(QDateTime::currentDateTime().toString(Qt::ISODate)) << ",\n";
        stream << "  \"models\": {\n";
        stream << "    \"workpiece\": " << (copiedWorkpiece && !workpiecePath.isEmpty() ? jsonString(relativePathFrom(root, workpiecePath)) : QStringLiteral("null")) << ",\n";
        stream << "    \"tool\": " << (copiedTool && !toolPath.isEmpty() ? jsonString(relativePathFrom(root, toolPath)) : QStringLiteral("null")) << ",\n";
        stream << "    \"extra_stl\": " << (!extraStlExportPath.isEmpty() ? jsonString(relativePathFrom(root, extraStlExportPath)) : QStringLiteral("null")) << "\n";
        stream << "  },\n";
        stream << "  \"geometry_baked\": {\n";
        stream << "    \"workpiece\": " << (workpieceGeometryBaked ? "true" : "false") << ",\n";
        stream << "    \"tool\": " << (toolGeometryBaked ? "true" : "false") << ",\n";
        stream << "    \"extra_stl\": " << (extraStlGeometryBaked ? "true" : "false") << "\n";
        stream << "  },\n";
        stream << "  \"trajectory\": " << (!paqPath.isEmpty() ? jsonString(relativePathFrom(root, paqPath)) : QStringLiteral("null")) << ",\n";
        stream << "  \"abb_module\": " << (!modPath.isEmpty() ? jsonString(relativePathFrom(root, modPath)) : QStringLiteral("null")) << ",\n";
        stream << "  \"transform_matrix\": [\n";
        for (int row = 0; row < 4; ++row) {
            stream << "    [";
            for (int col = 0; col < 4; ++col) {
                stream << QString::number(matrix(row, col), 'g', 12);
                if (col < 3) {
                    stream << ", ";
                }
            }
            stream << "]";
            if (row < 3) {
                stream << ",";
            }
            stream << "\n";
        }
        stream << "  ]\n";
        stream << "}\n";
        config.close();
        exportedFiles << configPath;

        return copiedWorkpiece && copiedTool;
    }

    // General operation slot functions
    void TrajOptimizationDialog::onExportResultClicked()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("No trajectory data available!"));
            return;
        }

        const bool hasOptimizedTrajectory = !trajOptData_->tolerancedTrajectory_.empty();
        const bool hasAnyTrajectory = hasOptimizedTrajectory
            || !trajOptData_->transformedTrajectory_.empty()
            || !trajOptData_->originalTrajectory_.empty();
        const bool hasAnyModel = !trajOptData_->originalModelPath_.empty()
            || !trajOptData_->actualModelPath_.empty();
        if (!hasAnyTrajectory && !hasAnyModel) {
            QMessageBox::warning(this, tr("Error"),
                tr("No trajectory, workpiece, or tool data to export."));
            return;
        }

        bool ok;
        QString projectName = QInputDialog::getText(this,
            tr("Export Project"),
            tr("Enter project name:"),
            QLineEdit::Normal,
            "TrajProject_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"),
            &ok);

        if (!ok || projectName.isEmpty()) {
            return;
        }

        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path);

        QString parentDir = QFileDialog::getExistingDirectory(this,
            tr("Select Export Directory"),
            defaultPath,
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

        if (parentDir.isEmpty()) {
            return;
        }

        QString projectDir = parentDir + "/" + projectName;
        QDir dir(parentDir);
        if (!dir.mkdir(projectName)) {
            QMessageBox::critical(this, tr("Error"),
                tr("Failed to create project directory!\nMaybe the directory already exists?"));
            return;
        }

        QStringList exportedFiles;
        if (hasOptimizedTrajectory) {
            if (!TrajOptimizationAlgorithm::exportProject(*trajOptData_,
                projectDir.toStdString(),
                projectName.toStdString())) {
                QMessageBox::critical(this, tr("Error"), tr("Failed to export project!"));
                return;
            }

            if (!exportTrajectoryExecutionFiles(projectDir, exportedFiles)) {
                QMessageBox::warning(this, tr("Partial Export"),
                    tr("Project export succeeded, but new_PAQ.txt or ABB MainModule.mod export failed."));
            }
        }
        else {
            if (!exportTransformedInputSet(projectDir, trajOptData_->transformMatrix_, QString(), exportedFiles)) {
                QMessageBox::critical(this, tr("Error"), tr("Failed to export transformed project!"));
                return;
            }
        }

        const size_t totalPoints = hasOptimizedTrajectory
            ? trajOptData_->tolerancedTrajectory_.size()
            : (!trajOptData_->transformedTrajectory_.empty()
                ? trajOptData_->transformedTrajectory_.size()
                : trajOptData_->originalTrajectory_.size());

        QString message = tr("Project exported successfully!\n\n") +
            tr("Project name: %1\n").arg(projectName) +
            tr("Export location: %2\n\n").arg(projectDir) +
            tr("Project structure:\n") +
            tr("%1/\n").arg(projectName) +
            (hasOptimizedTrajectory
                ? tr("  - config.json\n"
                     "  - robot/\n"
                     "      - robot1/ (workpiece + STL files)\n"
                     "      - robot2/ (tool + STL files)\n"
                     "  - Trajectory/\n"
                     "      - trajectory.txt\n"
                     "  - new_PAQ.txt\n"
                     "  - MainModule.mod\n\n")
                : tr("  - transform_export.json\n"
                     "  - model/\n"
                     "      - workpiece/\n"
                     "      - tool/\n"
                     "  - traj/\n"
                     "      - new_PAQ.txt\n"
                     "      - MainModule.mod\n\n")) +
            tr("Total points: %1").arg(totalPoints);

        if (!trajOptData_->unavoidableCollisionPoints_.empty()) {
            message += tr("\n\nWarning: %1 points have unavoidable collisions")
                .arg(trajOptData_->unavoidableCollisionPoints_.size());
        }

        QMessageBox::information(this, tr("Success"), message);
    }

    void TrajOptimizationDialog::onSaveConfigClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path);

        QString fileName = QFileDialog::getSaveFileName(this,
            tr("Save Configuration"),
            defaultPath,
            tr("Configuration Files (*.json *.xml)"));

        if (!fileName.isEmpty()) {
            // TODO: Implement saving configuration logic
            QMessageBox::information(this, tr("Information"), tr("Save configuration feature to be implemented"));
        }
    }

    void TrajOptimizationDialog::onOpenTrajectoryTransformClicked()
    {
        Eigen::Matrix4d matrix = trajOptData_ ? trajOptData_->transformMatrix_ : Eigen::Matrix4d::Identity();
        showTrajectoryTransformDialog(matrix);
    }

    void TrajOptimizationDialog::onOpenTrajectoryPlaybackClicked()
    {
        showTrajectoryPlaybackDialog();
    }

}
