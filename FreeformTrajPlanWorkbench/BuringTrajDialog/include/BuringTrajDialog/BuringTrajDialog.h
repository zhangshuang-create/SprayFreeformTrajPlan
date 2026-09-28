#pragma once

#include <QtWidgets/QMainWindow>
#include "BuringTrajDialog_global.h"
#include <BurTrajPlan/IBurTrajPlan.h>
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <ModelData/ModelData.h>
#include <QOpenGLWidget>
#include <functional>
#include <PublicDataRender/PublicDataRender.h>    // For PublicDataRender
    // Forward declarations
    namespace ModelTrajPlan { class TrajOptimizationData; }

    QT_BEGIN_NAMESPACE
        namespace Ui { class BuringTrajDialogClass; };
    QT_END_NAMESPACE

 namespace ModelTrajPlan
    {
        class BuringTrajDialog_SHAREDLIB BuringTrajDialog : public QMainWindow
    {

        Q_OBJECT

    public:
        // Addition: support passing BurTrajPlan model, GeneralData, and RenderData from Controller
        BuringTrajDialog(IBurTrajPlan_spointer TrajAlgorithm,
            GeneralData* Data,
            RenderData* RenderData,
            PublicData::PublicModelData* publicModelData,
            TrajOptimizationData* trajOptData,  // [Addition]
            PublicData::TrajData* publicTrajData,
            PublicData::PublicDataRender* publicDataRender,
            QWidget* parent = nullptr);
        // Keep original constructor for backward compatibility
        BuringTrajDialog(QWidget* parent = nullptr);
        ~BuringTrajDialog();

        // Set OpenGL view and refresh callback (forwarded to internal SlicingWidgets)
        void setOpenGLView(QOpenGLWidget* openGLView);
        void setForceRefreshHook(const std::function<void()>& cb);
        Q_INVOKABLE void reloadUiFromCurrentData();

    private:
        Ui::BuringTrajDialogClass* ui;
        // Addition: store BurTrajPlan data pointer
        IBurTrajPlan_spointer burModel_;
        // Addition: store GeneralData pointer
        GeneralData* generalData_;
        // Addition: store RenderData pointer
        RenderData* renderData_;
        // [Addition] store PublicModelData pointer
        PublicData::PublicModelData* publicModelData_;
        // [Addition] store PublicTrajData pointer
        PublicData::TrajData* publicTrajData_;
        // [Addition] store PublicDataRender pointer
        PublicData::PublicDataRender* publicDataRender_;
        // [Addition] store TrajOptimizationData pointer
        TrajOptimizationData* trajOptData_;
    };

}