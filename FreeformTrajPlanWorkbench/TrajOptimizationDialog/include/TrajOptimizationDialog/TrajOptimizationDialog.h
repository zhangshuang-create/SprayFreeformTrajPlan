#pragma once

#include <QtWidgets/QDialog>
#include <QStringList>
#include "TrajOptimizationDialog_global.h"
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <TrajOptimization/TrajOptimization.h>
#include <GeneralData/GeneralData.h>
#include <memory>
#include <functional>

QT_BEGIN_NAMESPACE
namespace Ui { class TrajOptimizationDialogClass; }
QT_END_NAMESPACE

namespace ModelTrajPlan
{
    class TrajOptimizationDialog_SHAREDLIB TrajOptimizationDialog : public QDialog
    {
        Q_OBJECT

    public:
        TrajOptimizationDialog(QWidget* parent = nullptr);
        TrajOptimizationDialog(TrajOptimizationData* trajOptData, GeneralData* generalData, QWidget* parent = nullptr);
        ~TrajOptimizationDialog();

        // Set data pointers
        void setTrajOptData(TrajOptimizationData* trajOptData);
        void setGeneralData(GeneralData* generalData);

        // Get data manager
        TrajOptimizationData* getDataManager() { return trajOptData_; }

        // Set force-refresh hook (for synchronized OpenGL refresh)
        void setForceRefreshHook(std::function<void()> hook) { forceRefreshHook_ = hook; }

    private slots:
        // Trajectory-related slot functions
        void onTrajectoryFromFileToggled(bool checked);
        void onTrajectoryFromSystemToggled(bool checked);
        void onBrowseTrajectoryClicked();
        void onLoadTrajectoryFromFileClicked();
        void onRefreshSystemTrajectoryClicked();
        void onLoadTrajectoryFromSystemClicked();

        // Robot import slot functions
        void onBrowseRobotFileClicked();
        void onLoadRobotFromSystemClicked();

        // Workpiece and tool slot functions
        void onBrowseWorkpieceClicked();
        void onLoadWorkpieceClicked();
        void onBrowseToolClicked();
        void onLoadToolClicked();

        // Collision detection slot functions
        void onStartCollisionDetectionClicked();
        void onStopCollisionDetectionClicked();
        void onOptimizationAlgorithmIndexChanged(int index);

        // General operation slot functions
        void onExportResultClicked();
        void onSaveConfigClicked();
        void onClearAllModelsClicked();
        void onOpenTrajectoryTransformClicked();
        void onOpenTrajectoryPlaybackClicked();

    private:
        // Helper functions
        void updateToleranceStatistics(double avgTimePerPoint = 0.0);
        void showTrajectoryTransformDialog(Eigen::Matrix4d matrix);
        bool applyTrajectoryTransform(const Eigen::Matrix4d& matrix);
        void showTrajectoryPlaybackDialog();
        bool exportTrajectoryExecutionFiles(const QString& projectDir, QStringList& exportedFiles);
        bool exportTransformedInputSet(const QString& exportDir, const Eigen::Matrix4d& matrix, const QString& extraStlPath, QStringList& exportedFiles);

        Ui::TrajOptimizationDialogClass* ui;

        // === Data management ===
        TrajOptimizationData* trajOptData_;  // Data manager pointer (passed externally and instantiated in host application)
        GeneralData* generalData_;            // GeneralData pointer (passed externally)

        // === Robot path configuration ===
        std::vector<std::string> robotPaths_;  // Robot model path list

        // === Force-refresh hook ===
        std::function<void()> forceRefreshHook_;  // Used for synchronized OpenGL refresh

        // === Helper methods ===
        void initializeSystem();                              // Initialize system
        void updateRobotListDisplay();                        // Update robot list display
        void updateTrajectoryInfoDisplay();                   // Update trajectory info display
    };

}

