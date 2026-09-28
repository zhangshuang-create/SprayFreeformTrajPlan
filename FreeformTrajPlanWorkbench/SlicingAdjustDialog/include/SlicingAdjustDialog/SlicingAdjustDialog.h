#pragma once

#include <QtWidgets/QDialog>
#include "SlicingAdjustDialog_global.h"
#include <BurTrajPlan/IBurTrajPlan.h>  // Trajectory planning model interface
#include <QOpenGLWidget>               // OpenGL view interface
#include <QTableWidgetItem>            // For table widget item
#include <functional>                  // For std::function
#include <map>                         // For storing original centers
#include <Eigen/Dense>                 // For Eigen::Vector3d
#include <GeneralData/GeneralData.h>   // For GeneralData
#include <RenderData/RenderData.h>     // For RenderData
#include <ModelData/ModelData.h>       // For PublicModelData
#include <TrajData/TrajData.h>         // For PublicTrajData
#include <PublicDataRender/PublicDataRender.h>  // For PublicDataRender

QT_BEGIN_NAMESPACE
namespace Ui { class SlicingAdjustDialogClass; };
QT_END_NAMESPACE

namespace ModelTrajPlan
{
    class TrajOptimizationData;

    class SlicingAdjustDialog_SHAREDLIB SlicingAdjustDialog : public QDialog
    {
        Q_OBJECT

    public:
        SlicingAdjustDialog(QWidget *parent = nullptr);
        ~SlicingAdjustDialog();

        // Set trajectory planning model
        void setTrajPlanModel(ModelTrajPlan::IBurTrajPlan_spointer model);

        // Get trajectory planning model
        ModelTrajPlan::IBurTrajPlan_spointer getTrajPlanModel() const;

        // Set general data
        void setGeneralData(ModelTrajPlan::GeneralData* generalData);

        // Get general data
        ModelTrajPlan::GeneralData* getGeneralData();
        const ModelTrajPlan::GeneralData* getGeneralData() const;

        // Set render data
        void setRenderData(ModelTrajPlan::RenderData* renderData);

        // Get render data
        ModelTrajPlan::RenderData* getRenderData();
        const ModelTrajPlan::RenderData* getRenderData() const;

        // Set public model data
        void setPublicModelData(PublicData::PublicModelData* publicModelData);

        // Get public model data
        PublicData::PublicModelData* getPublicModelData();
        const PublicData::PublicModelData* getPublicModelData() const;

        // Set public trajectory data
        void setPublicTrajData(PublicData::TrajData* publicTrajData);

        // Get public trajectory data
        PublicData::TrajData* getPublicTrajData();
        const PublicData::TrajData* getPublicTrajData() const;

        // Set trajectory optimization data
        void setTrajOptData(ModelTrajPlan::TrajOptimizationData* trajOptData);

        // Get trajectory optimization data
        ModelTrajPlan::TrajOptimizationData* getTrajOptData();
        const ModelTrajPlan::TrajOptimizationData* getTrajOptData() const;

        // Set public data render
        void setPublicDataRender(PublicData::PublicDataRender* publicDataRender);

        // Get public data render
        PublicData::PublicDataRender* getPublicDataRender();
        const PublicData::PublicDataRender* getPublicDataRender() const;

        // Set OpenGL view reference
        void setOpenGLView(QOpenGLWidget* openGLView);

        // Get OpenGL view reference
        QOpenGLWidget* getOpenGLView();
        const QOpenGLWidget* getOpenGLView() const;

        // Optional: force-refresh hook to trigger OpenGL repaint from host
        void setForceRefreshHook(const std::function<void()>& cb);

        // Set whether a project/workspace is currently active
        void setWorkspaceActive(bool active);

        // Force UI controls reload from current model/data
        void reloadUiFromCurrentData();

    public slots:
        // Reset dialog-only UI/cache state after slicing workspace is cleared
        void resetToInitialState();

        // Slicing table control slots
        void onSliceTableItemChanged(QTableWidgetItem* item);
        void onSliceTableRowClicked(int row, int column);

        // Layer navigation slots
        void onDecreaseButtonClicked();
        void onIncreaseButtonClicked();
        void onSetTargetButtonClicked();
        void onResetUniformButtonClicked();

        // Translation control slots
        void onXTranslationDecreaseClicked();
        void onXTranslationIncreaseClicked();
        void onYTranslationDecreaseClicked();
        void onYTranslationIncreaseClicked();
        void onZTranslationDecreaseClicked();
        void onZTranslationIncreaseClicked();

        // Rotation control slots
        void onXRotationDecreaseClicked();
        void onXRotationIncreaseClicked();
        void onYRotationDecreaseClicked();
        void onYRotationIncreaseClicked();
        void onZRotationDecreaseClicked();
        void onZRotationIncreaseClicked();

        // Transform operation slots
        void onResetTransformClicked();
        void onApplyTransformClicked();

        // Transformation parameter change slot
        void onTransformationParameterChanged();

    private:
        Ui::SlicingAdjustDialogClass *ui;

        // Data members
        ModelTrajPlan::IBurTrajPlan_spointer trajPlanModel_;
        QOpenGLWidget* openGLView_ = nullptr;
        ModelTrajPlan::GeneralData* generalData_ = nullptr;
        ModelTrajPlan::RenderData* renderData_ = nullptr;
        PublicData::PublicModelData* publicModelData_ = nullptr;
        PublicData::TrajData* publicTrajData_ = nullptr;
        ModelTrajPlan::TrajOptimizationData* trajOptData_ = nullptr;
        PublicData::PublicDataRender* publicDataRender_ = nullptr;

        // Optional external refresh hook
        std::function<void()> forceRefreshHook_;

        // Workspace state
        bool m_hasActiveWorkspace = false;

        // Flag to block table signals during itemChanged event processing
        bool blockTableSignals_ = false;

        // Current selected row for transformation parameters
        int currentSelectedRow_ = -1;

        // Store original center for each slicing layer (center at first slicing generation)
        std::map<int, Eigen::Vector3d> originalCenters_;

        // Guard flag: suppress transformation update while loading row values into UI
        bool blockTransformationSignals_ = false;

        // Initialize UI signal-slot connections
        void initializeConnections();

        // Update UI from model data
        void updateUIFromModel();

        // Update slices table display
        void updateSlicesTable();

        // Apply transformation to selected slice
        void applyTransformationToSelectedSlice();

        // Keep the dialog selection and the renderer's selected cutting plane in sync.
        bool selectSliceLayer(int row);

        // Request a cutting-plane mesh rebuild and repaint after a data change.
        void refreshSlicingRender();
    };
}
