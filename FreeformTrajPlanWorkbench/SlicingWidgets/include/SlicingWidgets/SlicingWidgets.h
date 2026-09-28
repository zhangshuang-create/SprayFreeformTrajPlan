#pragma once

#include <QtWidgets/QWidget>
#include "SlicingWidgets_global.h"
#include <BurTrajPlan/IBurTrajPlan.h>  // Trajectory planning model interface
#include <QStringListModel>  // String list model for data display
#include <QOpenGLWidget>     // OpenGL view interface
#include <QTableWidget>      // Table widget for slicing controls
#include <QDateTime>         // For timestamp generation
#include <QDir>              // For directory operations
#include <memory>            // For smart pointers
#include <functional>
#include <map>               // For storing original centers
#include <Eigen/Dense>       // For Eigen::Vector3d
#include <GeneralData/GeneralData.h>  // For GeneralData
#include <RenderData/RenderData.h>    // For RenderData
#include <ModelData/ModelData.h>      // For PublicModelData
#include <TrajData/TrajData.h>        // For Public TrajData
#include <PublicDataRender/PublicDataRender.h>    // For PublicDataRender
    QT_BEGIN_NAMESPACE
        namespace Ui { class SlicingWidgets; };
    QT_END_NAMESPACE

namespace ModelTrajPlan
    {
    class TrajOptimizationData;

    class SlicingWidgets_SHAREDLIB SlicingWidgets : public QWidget
    {
        Q_OBJECT

    public:
        SlicingWidgets(QWidget* parent = nullptr);
        // Constructor with trajectory planning model, general data and render data
        SlicingWidgets(IBurTrajPlan_spointer model,
            GeneralData* generalData,
            RenderData* renderData,
            PublicData::PublicModelData* publicModelData,
            PublicData::TrajData* publicTrajData,
            TrajOptimizationData* trajOptData,
            PublicData::PublicDataRender* publicDataRender,
            QWidget* parent = nullptr);
        ~SlicingWidgets();

        // Set trajectory planning model
        void setTrajPlanModel(IBurTrajPlan_spointer model);

        // Get trajectory planning model
        IBurTrajPlan_spointer getTrajPlanModel() const;

        // Set general data
        void setGeneralData(GeneralData* generalData);

        // Get general data
        GeneralData* getGeneralData();
        const GeneralData* getGeneralData() const;

        // Set public data render
        void setPublicDataRender(PublicData::PublicDataRender* publicDataRender);
        // Get public data render
        PublicData::PublicDataRender* getPublicDataRender();
        const PublicData::PublicDataRender* getPublicDataRender() const;
        // Set render data
        void setRenderData(RenderData* renderData);
        // Get render data
        RenderData* getRenderData();
        const RenderData* getRenderData() const;

        // Set public model data
        void setPublicModelData(PublicData::PublicModelData* publicModelData);

        // Get public model data
        PublicData::PublicModelData* getPublicModelData();
        const PublicData::PublicModelData* getPublicModelData() const;
        // Set public trajectory data
        void setPublicTrajData(PublicData::TrajData* publicTrajData);

        // Set trajectory optimization data
        void setTrajOptData(TrajOptimizationData* trajOptData);

        // set trajectory optimization data
        TrajOptimizationData* getTrajOptData();
        const TrajOptimizationData* getTrajOptData() const;

        // Set OpenGL view reference
        void setOpenGLView(QOpenGLWidget* openGLView);

        // optional: force-refresh hook to trigger OpenGL repaint from host
        void setForceRefreshHook(const std::function<void()>& cb) { forceRefreshHook_ = cb; }

        // Set whether a project/workspace is currently active.
        void setWorkspaceActive(bool active);

        // Force UI controls reload from current model/data.
        Q_INVOKABLE void reloadUiFromCurrentData();

    public slots:
        // Existing slot functions
        void onBrowseButtonClicked();
        void onCoarseButtonClicked();
        void onTrajOpticlicked();
        void onResetButtonClicked();
        void onConfirmButtonClicked();
        void onCancelButtonClicked();
        void onDecreaseButtonClicked();
        void onIncreaseButtonClicked();
        void onSetTargetButtonClicked();
        void onResetUniformButtonClicked();
        void onSliceTableItemChanged(::QTableWidgetItem* item);
        void onSliceTableRowClicked(int row, int column);
        void onTransformationParameterChanged();
        // Trajectory generation button slot function
        void onGenerateTrajectoryButtonClicked();
        // Trajectory connection button slot function
        void onConnectTrajectoryButtonClicked();
        void onExportContourLengthReportClicked();

    private slots:
        // Addition: context-menu slot for file-path line edit
        void onFilePathLineEditContextMenuRequested(const QPoint& pos);
        void onHideModelClicked();
        void onShowModelClicked();
        void onHideTrajectoryClicked();
        void onShowTrajectoryClicked();
        void onHideSlicingPlaneClicked();
        void onShowSlicingPlaneClicked();

        // Addition: XYZ-axis translation control slot functions
        void onXTranslationDecreaseClicked();
        void onXTranslationIncreaseClicked();
        void onYTranslationDecreaseClicked();
        void onYTranslationIncreaseClicked();
        void onZTranslationDecreaseClicked();
        void onZTranslationIncreaseClicked();

        // Addition: XYZ-axis rotation control slot functions
        void onXRotationDecreaseClicked();
        void onXRotationIncreaseClicked();
        void onYRotationDecreaseClicked();
        void onYRotationIncreaseClicked();
        void onZRotationDecreaseClicked();
        void onZRotationIncreaseClicked();

        // Addition: transform operation slot functions
        void onResetTransformClicked();


    private:
        Ui::SlicingWidgets* ui;
        IBurTrajPlan_spointer trajPlanModel_;
        QOpenGLWidget* openGLView_ = nullptr;

        // General data storage - use pointer to avoid data copying
        GeneralData* generalData_;

        // Render data storage - use pointer to avoid data copying
        RenderData* renderData_;

        // [Addition] Public model data storage - use pointer to avoid data copying
        PublicData::PublicModelData* publicModelData_;

        PublicData::TrajData* publicTrajData_;
        // [Addition] Trajectory optimization data storage - use pointer to avoid data copying
        TrajOptimizationData* trajOptData_;
        PublicData::PublicDataRender* publicDataRender_;

        // String list model for data display
        QStringListModel listModel_;
        // optional external refresh hook
        std::function<void()> forceRefreshHook_;


        // Initialize UI signal-slot connections
        void initializeConnections();

        // Update slices table display
        void updateSlicesTable();

        // Apply UI parameters to model
        void applyParametersToModel();

        // UI-Model synchronization methods
        void updateUIFromModel();        // Update UI from model data
        void onParameterChanged();       // Parameter change handler

        // Refresh data display
        void refreshDataDisplay();

        // Flag to block table signals during itemChanged event processing
        bool blockTableSignals_ = false;

        // Current selected row for transformation parameters
        int currentSelectedRow_ = -1;

        // [Addition] Store original center for each slicing layer (center at first slicing generation)
        std::map<int, Eigen::Vector3d> originalCenters_;

        // Guard flag: suppress transformation update while loading row values into UI.
        bool blockTransformationSignals_ = false;

        bool m_hasActiveWorkspace = false;
    };

}
