#include <BuringTrajDialog/BuringTrajDialog.h>
#include "ui_BuringTrajDialog.h"
namespace ModelTrajPlan
{
    BuringTrajDialog::BuringTrajDialog(IBurTrajPlan_spointer TrajAlgorithm,
        GeneralData* GData,
        RenderData* RData,
        PublicData::PublicModelData* publicModelData,
        TrajOptimizationData* trajOptData,  // [Addition]
        PublicData::TrajData* publicTrajData,
        PublicData::PublicDataRender* publicDataRender,
        QWidget* parent)
        : QMainWindow(parent)
        , ui(new Ui::BuringTrajDialogClass())
        , burModel_(TrajAlgorithm)
        , generalData_(GData)
        , renderData_(RData)
        , publicModelData_(publicModelData)
        , publicTrajData_(publicTrajData)
        , trajOptData_(trajOptData) 
        , publicDataRender_(publicDataRender)
    {
        ui->setupUi(this);

        // Pass dependencies first, then bind model and trigger one explicit UI reload.
        if (ui->widget) {
            ui->widget->setGeneralData(generalData_);
            ui->widget->setRenderData(renderData_);
            ui->widget->setPublicModelData(publicModelData_);
            ui->widget->setPublicTrajData(publicTrajData_);
            ui->widget->setPublicDataRender(publicDataRender_);
            ui->widget->setTrajOptData(trajOptData_);  // [Addition]
            ui->widget->setTrajPlanModel(burModel_);
            ui->widget->reloadUiFromCurrentData();
        }
    }

    BuringTrajDialog::BuringTrajDialog(QWidget* parent)
        : QMainWindow(parent)
        , ui(new Ui::BuringTrajDialogClass())
        , generalData_(nullptr)
        , trajOptData_(nullptr)  // [Addition]
        , renderData_(nullptr)
        , publicModelData_(nullptr)  // [Addition]
        , publicTrajData_(nullptr)
        , publicDataRender_(nullptr)
    {
        ui->setupUi(this);
    }

    BuringTrajDialog::~BuringTrajDialog()
    {
        delete ui;
    }

    void BuringTrajDialog::setOpenGLView(QOpenGLWidget* openGLView)
    {
        // Forward to internal SlicingWidgets
        if (ui->widget) {
            ui->widget->setOpenGLView(openGLView);
        }
    }

    void BuringTrajDialog::setForceRefreshHook(const std::function<void()>& cb)
    {
        // Forward to internal SlicingWidgets
        if (ui->widget) {
            ui->widget->setForceRefreshHook(cb);
        }
    }

    void BuringTrajDialog::reloadUiFromCurrentData()
    {
        if (ui->widget)
        {
            ui->widget->reloadUiFromCurrentData();
        }
    }

}