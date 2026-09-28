#pragma once

#include <QtWidgets/QDialog>
#include "ui_ProcessOptimizationDialog.h"
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <TrajOptimization/TrajOptimization.h>
#include <GeneralData/GeneralData.h>
#include <memory>
#include <functional>

    QT_BEGIN_NAMESPACE
        namespace Ui { class ProcessOptimizationDialogClass; };
    QT_END_NAMESPACE
namespace ModelTrajPlan
    {
        class ProcessOptimizationDialog : public QDialog
    {
        Q_OBJECT

    public:
        ProcessOptimizationDialog(QWidget* parent = nullptr);
        ProcessOptimizationDialog(TrajOptimizationData* trajOptData, GeneralData* generalData, QWidget* parent = nullptr);
        ~ProcessOptimizationDialog();

        // Data pointer setters
        void setTrajOptData(TrajOptimizationData* trajOptData);
        void setGeneralData(GeneralData* generalData);

        // Data manager accessor
        TrajOptimizationData* getDataManager() { return trajOptData_; }

        // Optional hook to force refresh (e.g. sync OpenGL view)
        void setForceRefreshHook(std::function<void()> hook) { forceRefreshHook_ = hook; }

    private slots:
        // Original model slots
        void onBrowseOriginalModelClicked();
        void onLoadOriginalModelClicked();

        // Actual / scanned model slots
        void onBrowseActualModelClicked();
        void onLoadActualModelClicked();

        // Transform matrix slots
        void onAutoCalculateToggled(bool checked);
        void onManualInputToggled(bool checked);
        void onCalculateTransformClicked();
        void onResetTransformClicked();
        void onLoadTransformFromFileClicked();
        void onSaveTransformToFileClicked();
        void onApplyManualTransformClicked();

        // Trajectory transform and generation slots
        void onTransformTrajectoryClicked();
        void onPreviewTransformClicked();
        void onGenerateTrajectoryClicked();
        void onExportGeneratedTrajectoryClicked();
        void onVisualizeTrajectoryClicked();

        // Process optimization slots
        void onStartProcessOptimizationClicked();
        void onStopProcessOptimizationClicked();

    private:
        void initializeSystem();
        void updateTransformMatrixDisplay();

        Ui::ProcessOptimizationDialogClass* ui;

        TrajOptimizationData* trajOptData_;  // Optimization data (owned by caller)
        GeneralData* generalData_;            // Shared general data (owned by caller)

        std::function<void()> forceRefreshHook_;  // Optional view refresh callback
    };
}