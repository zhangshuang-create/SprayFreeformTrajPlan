#include "SlicingAdjustDialog/SlicingAdjustDialog.h"
#include "ui_SlicingAdjustDialog.h"
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QAbstractItemView>
#include <QThread>
#include <QMetaObject>

namespace ModelTrajPlan
{

SlicingAdjustDialog::SlicingAdjustDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SlicingAdjustDialogClass())
{
    ui->setupUi(this);
    setWindowTitle(tr("Slicing Control Dialog"));

    initializeConnections();
}

SlicingAdjustDialog::~SlicingAdjustDialog()
{
    delete ui;
}

void SlicingAdjustDialog::setTrajPlanModel(ModelTrajPlan::IBurTrajPlan_spointer model)
{
    trajPlanModel_ = model;
    if (trajPlanModel_ && generalData_) {
        updateUIFromModel();
    }
}

ModelTrajPlan::IBurTrajPlan_spointer SlicingAdjustDialog::getTrajPlanModel() const
{
    return trajPlanModel_;
}

void SlicingAdjustDialog::setGeneralData(ModelTrajPlan::GeneralData* generalData)
{
    generalData_ = generalData;
    if (trajPlanModel_ && generalData_) {
        updateUIFromModel();
    }
}

ModelTrajPlan::GeneralData* SlicingAdjustDialog::getGeneralData()
{
    return generalData_;
}

const ModelTrajPlan::GeneralData* SlicingAdjustDialog::getGeneralData() const
{
    return generalData_;
}

void SlicingAdjustDialog::setRenderData(ModelTrajPlan::RenderData* renderData)
{
    renderData_ = renderData;
}

ModelTrajPlan::RenderData* SlicingAdjustDialog::getRenderData()
{
    return renderData_;
}

const ModelTrajPlan::RenderData* SlicingAdjustDialog::getRenderData() const
{
    return renderData_;
}

void SlicingAdjustDialog::setPublicModelData(PublicData::PublicModelData* publicModelData)
{
    publicModelData_ = publicModelData;
}

PublicData::PublicModelData* SlicingAdjustDialog::getPublicModelData()
{
    return publicModelData_;
}

const PublicData::PublicModelData* SlicingAdjustDialog::getPublicModelData() const
{
    return publicModelData_;
}

void SlicingAdjustDialog::setPublicTrajData(PublicData::TrajData* publicTrajData)
{
    publicTrajData_ = publicTrajData;
}

PublicData::TrajData* SlicingAdjustDialog::getPublicTrajData()
{
    return publicTrajData_;
}

const PublicData::TrajData* SlicingAdjustDialog::getPublicTrajData() const
{
    return publicTrajData_;
}

void SlicingAdjustDialog::setTrajOptData(ModelTrajPlan::TrajOptimizationData* trajOptData)
{
    trajOptData_ = trajOptData;
}

ModelTrajPlan::TrajOptimizationData* SlicingAdjustDialog::getTrajOptData()
{
    return trajOptData_;
}

const ModelTrajPlan::TrajOptimizationData* SlicingAdjustDialog::getTrajOptData() const
{
    return trajOptData_;
}

void SlicingAdjustDialog::setPublicDataRender(PublicData::PublicDataRender* publicDataRender)
{
    publicDataRender_ = publicDataRender;
}

PublicData::PublicDataRender* SlicingAdjustDialog::getPublicDataRender()
{
    return publicDataRender_;
}

const PublicData::PublicDataRender* SlicingAdjustDialog::getPublicDataRender() const
{
    return publicDataRender_;
}

void SlicingAdjustDialog::setOpenGLView(QOpenGLWidget* openGLView)
{
    openGLView_ = openGLView;
}

QOpenGLWidget* SlicingAdjustDialog::getOpenGLView()
{
    return openGLView_;
}

const QOpenGLWidget* SlicingAdjustDialog::getOpenGLView() const
{
    return openGLView_;
}

void SlicingAdjustDialog::setForceRefreshHook(const std::function<void()>& cb)
{
    forceRefreshHook_ = cb;
}

void SlicingAdjustDialog::setWorkspaceActive(bool active)
{
    m_hasActiveWorkspace = active;
}

void SlicingAdjustDialog::reloadUiFromCurrentData()
{
    if (!m_hasActiveWorkspace)
        return;

    updateUIFromModel();
}

void SlicingAdjustDialog::initializeConnections()
{
    // Table widget connections
    if (ui && ui->slicesTableWidget) {
        ui->slicesTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        ui->slicesTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);

        connect(ui->slicesTableWidget, &QTableWidget::itemChanged,
                this, &SlicingAdjustDialog::onSliceTableItemChanged);
        connect(ui->slicesTableWidget, &QTableWidget::cellClicked,
                this, &SlicingAdjustDialog::onSliceTableRowClicked);
    }

    // Layer navigation buttons
    connect(ui->decreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onDecreaseButtonClicked);
    connect(ui->increaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onIncreaseButtonClicked);
    connect(ui->setTargetButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onSetTargetButtonClicked);
    connect(ui->resetUniformButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onResetUniformButtonClicked);

    // Translation control buttons
    connect(ui->xDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onXTranslationDecreaseClicked);
    connect(ui->xIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onXTranslationIncreaseClicked);
    connect(ui->yDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onYTranslationDecreaseClicked);
    connect(ui->yIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onYTranslationIncreaseClicked);
    connect(ui->zDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onZTranslationDecreaseClicked);
    connect(ui->zIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onZTranslationIncreaseClicked);

    // Rotation control buttons
    connect(ui->xRotDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onXRotationDecreaseClicked);
    connect(ui->xRotIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onXRotationIncreaseClicked);
    connect(ui->yRotDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onYRotationDecreaseClicked);
    connect(ui->yRotIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onYRotationIncreaseClicked);
    connect(ui->zRotDecreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onZRotationDecreaseClicked);
    connect(ui->zRotIncreaseButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onZRotationIncreaseClicked);

    // Transform operation buttons
    connect(ui->resetTransformButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onResetTransformClicked);
    connect(ui->applyTransformButton, &QPushButton::clicked,
            this, &SlicingAdjustDialog::onApplyTransformClicked);

    // Transformation parameter spinboxes
    connect(ui->xTranslationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
    connect(ui->yTranslationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
    connect(ui->zTranslationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
    connect(ui->xRotationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
    connect(ui->yRotationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
    connect(ui->zRotationSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingAdjustDialog::onTransformationParameterChanged);
}

void SlicingAdjustDialog::updateUIFromModel()
{
    if (!trajPlanModel_ || !generalData_) {
        resetToInitialState();
        return;
    }

    // Update slices table
    updateSlicesTable();
}

void SlicingAdjustDialog::resetToInitialState()
{
    if (!ui) {
        return;
    }

    blockTableSignals_ = true;
    if (ui->slicesTableWidget) {
        ui->slicesTableWidget->clearContents();
        ui->slicesTableWidget->setRowCount(0);
        ui->slicesTableWidget->clearSelection();
        ui->slicesTableWidget->setCurrentCell(-1, -1);
    }
    blockTableSignals_ = false;

    currentSelectedRow_ = -1;
    originalCenters_.clear();

    if (ui->keepVisibleCheckBox) {
        ui->keepVisibleCheckBox->setChecked(false);
    }
    if (ui->stepLineEdit) {
        ui->stepLineEdit->setText("10");
    }
    if (ui->targetZLineEdit) {
        ui->targetZLineEdit->clear();
    }
    if (ui->rangeLabel) {
        ui->rangeLabel->setText(tr("Range: [0.0, 0.0]"));
    }

    blockTransformationSignals_ = true;
    if (ui->xTranslationSpinBox) ui->xTranslationSpinBox->setValue(0.0);
    if (ui->yTranslationSpinBox) ui->yTranslationSpinBox->setValue(0.0);
    if (ui->zTranslationSpinBox) ui->zTranslationSpinBox->setValue(0.0);
    if (ui->xRotationSpinBox) ui->xRotationSpinBox->setValue(0.0);
    if (ui->yRotationSpinBox) ui->yRotationSpinBox->setValue(0.0);
    if (ui->zRotationSpinBox) ui->zRotationSpinBox->setValue(0.0);
    if (ui->xStepSpinBox) ui->xStepSpinBox->setValue(1.0);
    if (ui->yStepSpinBox) ui->yStepSpinBox->setValue(1.0);
    if (ui->zStepSpinBox) ui->zStepSpinBox->setValue(1.0);
    if (ui->xRotStepSpinBox) ui->xRotStepSpinBox->setValue(1.0);
    if (ui->yRotStepSpinBox) ui->yRotStepSpinBox->setValue(1.0);
    if (ui->zRotStepSpinBox) ui->zRotStepSpinBox->setValue(1.0);
    blockTransformationSignals_ = false;
}

void SlicingAdjustDialog::updateSlicesTable()
{
    if (!trajPlanModel_ || !generalData_) {
        // If no model or data, clear the table
        if (ui && ui->slicesTableWidget) {
            ui->slicesTableWidget->clearContents();
            ui->slicesTableWidget->setRowCount(0);
        }
        return;
    }

    std::vector<CuttingPlane> cuttingPlanes;
    trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

    // Get current slice result data
    const auto& sliceResult = generalData_->slicingData_.currentSliceResult_;

    // Block table signals to prevent triggering itemChanged
    blockTableSignals_ = true;

    // Clear and setup table
    ui->slicesTableWidget->clearContents();
    ui->slicesTableWidget->setRowCount(static_cast<int>(cuttingPlanes.size()));
    ui->slicesTableWidget->setColumnCount(4);

    QStringList headers;
    headers << "Layer" << "Z Position" << "Thickness" << "Contour Points";
    ui->slicesTableWidget->setHorizontalHeaderLabels(headers);

    for (size_t i = 0; i < cuttingPlanes.size(); ++i) {
        const auto& plane = cuttingPlanes[i];

        // Get contour point count for this slice
        int contourPointCount = 0;
        if (i < sliceResult.internalBoundaries.size()) {
            contourPointCount = static_cast<int>(sliceResult.internalBoundaries[i].points.size());
        }

        ui->slicesTableWidget->setItem(static_cast<int>(i), 0,
            new QTableWidgetItem(QString::number(plane.layer_index)));
        // Calculate final Z position including transformation: center.z + translation_z
        double actualZ = plane.center.z() + plane.translation_z;
        ui->slicesTableWidget->setItem(static_cast<int>(i), 1,
            new QTableWidgetItem(QString::number(actualZ, 'f', 3)));
        ui->slicesTableWidget->setItem(static_cast<int>(i), 3,
            new QTableWidgetItem(QString::number(contourPointCount)));
    }

    // Auto-resize columns to fit content
    ui->slicesTableWidget->resizeColumnsToContents();

    // Preserve selected row highlight after table rebuild
    if (currentSelectedRow_ >= 0 && currentSelectedRow_ < ui->slicesTableWidget->rowCount()) {
        ui->slicesTableWidget->setCurrentCell(currentSelectedRow_, 0);
        ui->slicesTableWidget->selectRow(currentSelectedRow_);
    }

    blockTableSignals_ = false;
}

void SlicingAdjustDialog::applyTransformationToSelectedSlice()
{
    if (currentSelectedRow_ < 0 || !trajPlanModel_ || !generalData_) {
        return;
    }

    // Get current cutting planes
    std::vector<CuttingPlane> cuttingPlanes;
    trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

    if (currentSelectedRow_ >= static_cast<int>(cuttingPlanes.size())) {
        return;  // Row index out of range
    }

    // Apply UI parameter values to the cutting plane
    CuttingPlane& plane = cuttingPlanes[currentSelectedRow_];

    // Get original center position (before any transformation)
    Eigen::Vector3d originalCenter;
    if (originalCenters_.find(currentSelectedRow_) != originalCenters_.end()) {
        originalCenter = originalCenters_[currentSelectedRow_];
    } else {
        // First time editing, save current center as original
        originalCenter = plane.center;
        originalCenters_[currentSelectedRow_] = originalCenter;
    }

    // Update translation - UI shows final position after transformation
    Eigen::Vector3d newCenter;
    newCenter.x() = ui->xTranslationSpinBox->value();
    newCenter.y() = ui->yTranslationSpinBox->value();
    newCenter.z() = ui->zTranslationSpinBox->value();

    // Calculate translation offset from original center
    plane.translation_x = newCenter.x() - originalCenter.x();
    plane.translation_y = newCenter.y() - originalCenter.y();
    plane.translation_z = newCenter.z() - originalCenter.z();

    // Update rotation
    plane.rotation_x = ui->xRotationSpinBox->value();
    plane.rotation_y = ui->yRotationSpinBox->value();
    plane.rotation_z = ui->zRotationSpinBox->value();

    // Update model with modified cutting planes by updating generalData directly
    generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;

    if (generalData_->processingState_.isSTLProcessed) {
        trajPlanModel_->updateSingleCuttingPlaneSize(*generalData_, currentSelectedRow_);
        // updateSingleCuttingPlaneSize selects the layer internally; restore the dialog state explicitly.
        selectSliceLayer(currentSelectedRow_);
    }

    refreshSlicingRender();

    // Update table to reflect changes
    updateSlicesTable();
}

bool SlicingAdjustDialog::selectSliceLayer(int row)
{
    if (!trajPlanModel_ || !generalData_ || row < 0) {
        return false;
    }

    std::vector<CuttingPlane> cuttingPlanes;
    trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
    if (row >= static_cast<int>(cuttingPlanes.size())) {
        return false;
    }

    for (size_t index = 0; index < cuttingPlanes.size(); ++index) {
        cuttingPlanes[index].isSelected = (static_cast<int>(index) == row);
    }
    generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;
    generalData_->slicingData_.lastSelectedLayerIndex = row;
    currentSelectedRow_ = row;
    return true;
}

void SlicingAdjustDialog::refreshSlicingRender()
{
    if (renderData_) {
        renderData_->slicing().needsUpdate = true;
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
    }

    if (openGLView_) {
        openGLView_->update();
    }
    if (forceRefreshHook_) {
        forceRefreshHook_();
    }
}

// ==================== Slot Implementations ====================

void SlicingAdjustDialog::onSliceTableItemChanged(QTableWidgetItem* item)
{
    if (!item) return;
    if (blockTableSignals_) return;
    if (!trajPlanModel_) return;

    int editedRow = item->row();
    int editedColumn = item->column();

    // Only allow editing Z position column (column 1)
    if (editedColumn != 1) {
        return;
    }

    // Get edited Z value
    bool ok;
    double editedZ = item->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, tr("Invalid Input"),
            tr("Please enter a valid number for Z position."));
        // Restore original value
        blockTableSignals_ = true;
        updateSlicesTable();
        blockTableSignals_ = false;
        return;
    }

    // Update cutting plane Z position
    std::vector<CuttingPlane> cuttingPlanes;
    trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

    if (editedRow >= 0 && editedRow < static_cast<int>(cuttingPlanes.size())) {
        CuttingPlane& plane = cuttingPlanes[editedRow];

        // Get original center
        Eigen::Vector3d originalCenter;
        if (originalCenters_.find(editedRow) != originalCenters_.end()) {
            originalCenter = originalCenters_[editedRow];
        } else {
            originalCenter = plane.center;
            originalCenters_[editedRow] = originalCenter;
        }

        // Update translation_z to achieve the edited Z position
        plane.translation_z = editedZ - originalCenter.z();

        // Update model by updating generalData directly
        generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;

        if (generalData_->processingState_.isSTLProcessed) {
            trajPlanModel_->updateSingleCuttingPlaneSize(*generalData_, editedRow);
        }
        selectSliceLayer(editedRow);

        // If this row is selected, update transformation UI
        if (editedRow == currentSelectedRow_) {
            if (ui->zTranslationSpinBox) {
                blockTransformationSignals_ = true;
                ui->zTranslationSpinBox->setValue(editedZ);
                blockTransformationSignals_ = false;
            }
        }

        refreshSlicingRender();
    }
}

void SlicingAdjustDialog::onSliceTableRowClicked(int row, int column)
{
    Q_UNUSED(column);

    if (row < 0 || row >= ui->slicesTableWidget->rowCount()) {
        return;  // Invalid row number
    }

    ui->slicesTableWidget->setCurrentCell(row, 0);
    ui->slicesTableWidget->selectRow(row);

    // Load transformation parameters for selected row
    if (!trajPlanModel_ || !generalData_) {
        return;
    }

    if (!selectSliceLayer(row)) {
        return;
    }

    std::vector<CuttingPlane> cuttingPlanes;
    trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

    const CuttingPlane& plane = cuttingPlanes[row];

    // Get original center for this slice
    Eigen::Vector3d originalCenter;
    if (originalCenters_.find(row) != originalCenters_.end()) {
        originalCenter = originalCenters_[row];
    } else {
        originalCenter = plane.center;
        originalCenters_[row] = originalCenter;
    }

    // Block transformation signals while loading values
    blockTransformationSignals_ = true;

    // Load translation values (absolute position after transformation)
    ui->xTranslationSpinBox->setValue(plane.center.x() + plane.translation_x);
    ui->yTranslationSpinBox->setValue(plane.center.y() + plane.translation_y);
    ui->zTranslationSpinBox->setValue(plane.center.z() + plane.translation_z);

    // Load rotation values
    ui->xRotationSpinBox->setValue(plane.rotation_x);
    ui->yRotationSpinBox->setValue(plane.rotation_y);
    ui->zRotationSpinBox->setValue(plane.rotation_z);

    blockTransformationSignals_ = false;
    refreshSlicingRender();
}

void SlicingAdjustDialog::onDecreaseButtonClicked()
{
    int step = ui->stepLineEdit->text().toInt();
    int currentRow = ui->slicesTableWidget->currentRow();

    if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
        onSliceTableRowClicked(currentRow, 0);
        QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
        if (zItem) {
            double currentZ = zItem->text().toDouble();
            double newZ = currentZ - step;
            zItem->setText(QString::number(newZ, 'f', 2));
        }
    }
}

void SlicingAdjustDialog::onIncreaseButtonClicked()
{
    int step = ui->stepLineEdit->text().toInt();
    int currentRow = ui->slicesTableWidget->currentRow();

    if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
        onSliceTableRowClicked(currentRow, 0);
        QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
        if (zItem) {
            double currentZ = zItem->text().toDouble();
            double newZ = currentZ + step;
            zItem->setText(QString::number(newZ, 'f', 2));
        }
    }
}

void SlicingAdjustDialog::onSetTargetButtonClicked()
{
    QString targetZText = ui->targetZLineEdit->text();
    bool ok;
    double targetZ = targetZText.toDouble(&ok);

    if (ok) {
        int currentRow = ui->slicesTableWidget->currentRow();
        if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
            onSliceTableRowClicked(currentRow, 0);
            QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
            if (zItem) {
                zItem->setText(QString::number(targetZ, 'f', 2));
            }
        }
    } else {
        QMessageBox::warning(this, tr("Warning"), tr("Please enter a valid Z value!"));
    }
}

void SlicingAdjustDialog::onResetUniformButtonClicked()
{
    if (!trajPlanModel_ || !generalData_) {
        QMessageBox::warning(this, tr("Warning"),
            tr("No model data available. Please load a model first."));
        return;
    }

    int intervals = ui->slicesTableWidget->rowCount();
    if (intervals <= 0) {
        intervals = static_cast<int>(generalData_->slicingData_.cuttingPlanes_.size());
    }
    if (intervals <= 0) {
        intervals = 20;
    }

    // Get actual Z range from cutting planes or model data
    double minZ = 0.0;
    double maxZ = 0.0;
    bool hasValidRange = false;

    // Try to get Z range from existing cutting planes
    if (!generalData_->slicingData_.cuttingPlanes_.empty()) {
        minZ = std::numeric_limits<double>::max();
        maxZ = std::numeric_limits<double>::lowest();

        for (const auto& plane : generalData_->slicingData_.cuttingPlanes_) {
            double z = plane.center.z();
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);
        }
        hasValidRange = true;
    }

    // If no cutting planes, try to get from model bounding box
    if (!hasValidRange && publicModelData_ && publicModelData_->hasValidData()) {
        // Try to get model bounds from public model data
        // This is a fallback - actual implementation depends on PublicModelData API
        minZ = 0.0;
        maxZ = 100.0; // Default fallback value
        hasValidRange = true;
    }

    // If still no valid range, use a reasonable default
    if (!hasValidRange) {
        QMessageBox::warning(this, tr("Warning"),
            tr("Cannot determine Z range. Using default range [0, 100]."));
        minZ = 0.0;
        maxZ = 100.0;
    }

    // Prevent division by zero when intervals = 1
    if (intervals == 1) {
        QMessageBox::warning(this, tr("Warning"),
            tr("Cannot create uniform distribution with only 1 layer. Please use at least 2 layers."));
        return;
    }

    auto& cuttingPlanes = generalData_->slicingData_.cuttingPlanes_;
    if (static_cast<int>(cuttingPlanes.size()) != intervals) {
        QMessageBox::warning(this, tr("Warning"),
            tr("Uniform reset can only reposition the %1 generated slicing layers.")
                .arg(cuttingPlanes.size()));
        return;
    }

    const int selectedRow = currentSelectedRow_;
    for (int i = 0; i < intervals; ++i) {
        const double t = static_cast<double>(i) / (intervals - 1);
        const double z = minZ + t * (maxZ - minZ);
        cuttingPlanes[i].translation_z = z - cuttingPlanes[i].center.z();
    }

    if (generalData_->processingState_.isSTLProcessed) {
        for (int i = 0; i < intervals; ++i) {
            trajPlanModel_->updateSingleCuttingPlaneSize(*generalData_, i);
        }
    }

    if (selectedRow >= 0 && selectedRow < intervals) {
        selectSliceLayer(selectedRow);
    } else {
        for (auto& plane : generalData_->slicingData_.cuttingPlanes_) {
            plane.isSelected = false;
        }
        generalData_->slicingData_.lastSelectedLayerIndex = -1;
    }
    updateSlicesTable();
    refreshSlicingRender();

    QMessageBox::information(this, tr("Reset Uniform"),
        tr("Reset to uniform distribution with %1 layers in range [%2, %3]")
        .arg(intervals).arg(minZ, 0, 'f', 2).arg(maxZ, 0, 'f', 2));
}

void SlicingAdjustDialog::onXTranslationDecreaseClicked()
{
    double step = ui->xStepSpinBox->value();
    ui->xTranslationSpinBox->setValue(ui->xTranslationSpinBox->value() - step);
}

void SlicingAdjustDialog::onXTranslationIncreaseClicked()
{
    double step = ui->xStepSpinBox->value();
    ui->xTranslationSpinBox->setValue(ui->xTranslationSpinBox->value() + step);
}

void SlicingAdjustDialog::onYTranslationDecreaseClicked()
{
    double step = ui->yStepSpinBox->value();
    ui->yTranslationSpinBox->setValue(ui->yTranslationSpinBox->value() - step);
}

void SlicingAdjustDialog::onYTranslationIncreaseClicked()
{
    double step = ui->yStepSpinBox->value();
    ui->yTranslationSpinBox->setValue(ui->yTranslationSpinBox->value() + step);
}

void SlicingAdjustDialog::onZTranslationDecreaseClicked()
{
    double step = ui->zStepSpinBox->value();
    ui->zTranslationSpinBox->setValue(ui->zTranslationSpinBox->value() - step);
}

void SlicingAdjustDialog::onZTranslationIncreaseClicked()
{
    double step = ui->zStepSpinBox->value();
    ui->zTranslationSpinBox->setValue(ui->zTranslationSpinBox->value() + step);
}

void SlicingAdjustDialog::onXRotationDecreaseClicked()
{
    double step = ui->xRotStepSpinBox->value();
    ui->xRotationSpinBox->setValue(ui->xRotationSpinBox->value() - step);
}

void SlicingAdjustDialog::onXRotationIncreaseClicked()
{
    double step = ui->xRotStepSpinBox->value();
    ui->xRotationSpinBox->setValue(ui->xRotationSpinBox->value() + step);
}

void SlicingAdjustDialog::onYRotationDecreaseClicked()
{
    double step = ui->yRotStepSpinBox->value();
    ui->yRotationSpinBox->setValue(ui->yRotationSpinBox->value() - step);
}

void SlicingAdjustDialog::onYRotationIncreaseClicked()
{
    double step = ui->yRotStepSpinBox->value();
    ui->yRotationSpinBox->setValue(ui->yRotationSpinBox->value() + step);
}

void SlicingAdjustDialog::onZRotationDecreaseClicked()
{
    double step = ui->zRotStepSpinBox->value();
    ui->zRotationSpinBox->setValue(ui->zRotationSpinBox->value() - step);
}

void SlicingAdjustDialog::onZRotationIncreaseClicked()
{
    double step = ui->zRotStepSpinBox->value();
    ui->zRotationSpinBox->setValue(ui->zRotationSpinBox->value() + step);
}

void SlicingAdjustDialog::onResetTransformClicked()
{
    // Reset all transformation parameters to zero
    blockTransformationSignals_ = true;

    ui->xTranslationSpinBox->setValue(0.0);
    ui->yTranslationSpinBox->setValue(0.0);
    ui->zTranslationSpinBox->setValue(0.0);
    ui->xRotationSpinBox->setValue(0.0);
    ui->yRotationSpinBox->setValue(0.0);
    ui->zRotationSpinBox->setValue(0.0);

    blockTransformationSignals_ = false;

    QMessageBox::information(this, tr("Reset Transformation"),
        tr("All transformation parameters have been reset to 0."));
}

void SlicingAdjustDialog::onApplyTransformClicked()
{
    applyTransformationToSelectedSlice();
}

void SlicingAdjustDialog::onTransformationParameterChanged()
{
    if (blockTransformationSignals_) {
        return;
    }

    // Real-time transformation update
    applyTransformationToSelectedSlice();
}

} // namespace ModelTrajPlan
