// ================== SlicingWidgets.cpp ==================
#include <SlicingWidgets/SlicingWidgets.h>
#include "ui_SlicingWidgets.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QDir>
#include <Qdebug>
#include <QThread>
#include <QMetaObject>
#include <QApplication>
#include <QMenu>
#include <QAction>
#include <QCursor>
#include <QAbstractItemView>
#include <QPushButton>
#include <QSaveFile>
#include <QTextStream>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QStandardPaths>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <data_path.h>
#include <TrajOptimizationDialog/TrajOptimizationDialog.h>
namespace ModelTrajPlan
{
namespace
{
    QPushButton* optionalButton(QWidget* parent, const char* objectName)
    {
        return parent ? parent->findChild<QPushButton*>(objectName) : nullptr;
    }
    void syncConnectedTrajectoryToPublic(
        const GeneralData& generalData,
        PublicData::TrajData* publicTrajData)
    {
        if (!publicTrajData) {
            return;
        }

        const auto& connected = generalData.trajData_.connectionData_.connectedTrajectory_;
        if (connected.empty()) {
            return;
        }

        auto& publicPoints = publicTrajData->getTrajectoryPoints();
        publicPoints.clear();
        publicTrajData->getSpiralTrajectoryPoints().clear();

        double timestamp = 0.0;
        for (const auto& src : connected) {
            PublicData::TrajectoryPoint dst;
            dst.pose = src.pose;
            dst.spray_distance = src.spray_distance;
            dst.velocity = src.velocity;
            publicPoints[timestamp] = dst;
            timestamp += 1.0;
        }
    }
}


SlicingWidgets::SlicingWidgets(QWidget *parent)
    : SlicingWidgets(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, parent)
{}

// Constructor with model, generalData, and renderData parameters havnt been used
SlicingWidgets::SlicingWidgets(IBurTrajPlan_spointer model,
    GeneralData* generalData,
    RenderData* renderData,
    PublicData::PublicModelData* publicModelData,
    PublicData::TrajData* publicTrajData,
    TrajOptimizationData* trajOptData,
    PublicData::PublicDataRender* publicDataRender,
    QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SlicingWidgets())
    , trajPlanModel_(model)
    , openGLView_(nullptr)
    , generalData_(generalData)  // Store general data
    , renderData_(renderData)    // Store render data
    , publicModelData_(publicModelData)
    , publicTrajData_(publicTrajData)
    , trajOptData_(trajOptData)
    , publicDataRender_(publicDataRender)
{
    ui->setupUi(this);
    initializeConnections();

    auto* exportContourReportButton = new QPushButton(tr("Export Contour Length Report"), this);
    exportContourReportButton->setObjectName("exportContourLengthReportButton");
    exportContourReportButton->setToolTip(tr("Export contour lengths, trend analysis, annotated contours, and adjacent-track spacing"));
    ui->verticalLayout->addWidget(exportContourReportButton);
    connect(exportContourReportButton, &QPushButton::clicked,
        this, &SlicingWidgets::onExportContourLengthReportClicked);
    
    // If model is valid, initialize UI from model data
    if (trajPlanModel_) {
        updateUIFromModel();  // Update UI from model data
    }
}

// Set trajectory planning model
void SlicingWidgets::setTrajPlanModel(IBurTrajPlan_spointer model)
{
    trajPlanModel_ = model;
    if (trajPlanModel_ && generalData_) {
        updateUIFromModel();  // Update UI

    } else {
        // Clear UI
        ui->filePathLineEdit->clear();
        ui->slicesTableWidget->setRowCount(0);

    }
}

void SlicingWidgets::reloadUiFromCurrentData()
{
    updateUIFromModel();
}

void SlicingWidgets::setWorkspaceActive(bool active)
{
    m_hasActiveWorkspace = active;
}

// Set general data
void SlicingWidgets::setGeneralData(GeneralData* generalData)
{
    generalData_ = generalData;  // Store general data
    if (trajPlanModel_ && generalData_) {
        updateUIFromModel();
    }
}

// Get general data
GeneralData* SlicingWidgets::getGeneralData()
{
    return generalData_;
}

const GeneralData* SlicingWidgets::getGeneralData() const
{
    return generalData_;
}

// Set render data
void SlicingWidgets::setRenderData(RenderData* renderData)
{
    renderData_ = renderData;  // Store render data
}

// Get render data
RenderData* SlicingWidgets::getRenderData()
{
    return renderData_;
}

const RenderData* SlicingWidgets::getRenderData() const
{
    return renderData_;
}

// ��������Set public model data
void SlicingWidgets::setPublicModelData(PublicData::PublicModelData* publicModelData)
{
    publicModelData_ = publicModelData;
}

// ��������Get public model data
PublicData::PublicModelData* SlicingWidgets::getPublicModelData()
    {
        return publicModelData_;
    }

const PublicData::PublicModelData* SlicingWidgets::getPublicModelData() const
    {
        return publicModelData_;
    }

void SlicingWidgets::setPublicTrajData(PublicData::TrajData* publicTrajData)
{
    publicTrajData_ = publicTrajData;
}

// ��������Set trajectory optimization data
void SlicingWidgets::setTrajOptData(TrajOptimizationData* trajOptData)
{
    trajOptData_ = trajOptData;
}

void SlicingWidgets::setPublicDataRender(PublicData::PublicDataRender* publicDataRender)
{
    publicDataRender_ = publicDataRender;
}

PublicData::PublicDataRender* SlicingWidgets::getPublicDataRender()
{
    return publicDataRender_;
}

const PublicData::PublicDataRender* SlicingWidgets::getPublicDataRender() const
{
    return publicDataRender_;
}
// ��������Get trajectory optimization data
TrajOptimizationData* SlicingWidgets::getTrajOptData()
{
    return trajOptData_;
}

const TrajOptimizationData* SlicingWidgets::getTrajOptData() const
{
    return trajOptData_;
}

SlicingWidgets::~SlicingWidgets()
{
    delete ui;
}


// Get trajectory planning model
IBurTrajPlan_spointer SlicingWidgets::getTrajPlanModel() const
{
    return trajPlanModel_;
}

// Set OpenGL view
void SlicingWidgets::setOpenGLView(QOpenGLWidget* openGLView)
{
    openGLView_ = openGLView;
}



// Above methods handle parameter passing

// Initialize signal-slot connections
void SlicingWidgets::initializeConnections()
{
    if (ui && ui->slicesTableWidget) {
        // Use row-based single selection so clicked row stays highlighted.
        ui->slicesTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        ui->slicesTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    }

    // Connect parameter control value change signals
    connect(ui->sizeSpinBox,       QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    connect(ui->epsilonSpinBox,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    connect(ui->minStepSpinBox,  QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    connect(ui->z1OffsetSpinBox,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    // Add z2Offset control signal connection
    connect(ui->z2OffsetSpinBox,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    connect(ui->offsetDistanceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SlicingWidgets::onParameterChanged);
    
    // Connect table item change signal
    connect(ui->slicesTableWidget, &QTableWidget::itemChanged,
            this, &SlicingWidgets::onSliceTableItemChanged);
    
    // Connect table row click signal - for selecting rows in table
    connect(ui->slicesTableWidget, &QTableWidget::cellClicked,
            this, &SlicingWidgets::onSliceTableRowClicked);
    
    // Note: Transformation parameter signals are connected in UI file (.ui)
    // No need to connect them manually here
            
    
}

// ================== Slot Function Implementation ==================
// Slot function for when slice table item is modified
void SlicingWidgets::onSliceTableItemChanged(QTableWidgetItem* item)
{
    if (!item) return;
    if (blockTableSignals_) return;
    if (!trajPlanModel_) return;

    // Check if editing Z Position column (column 1), handle modification
    if (item->column() == 1) {
        bool ok = false;
        const double editedZ = item->text().toDouble(&ok);
        if (!ok) {
            QMessageBox::warning(this, "Warning", "Please enter a valid Z value!");
            // If invalid, restore original value
            blockTableSignals_ = true;
            updateSlicesTable();
            blockTableSignals_ = false;
            return;
        }
        // Handle modification: user edits the final Z position (center.z() + translation_z)
        int editedRow = item->row();
        if (editedRow >= 0 && editedRow < ui->slicesTableWidget->rowCount()) {
            std::vector<CuttingPlane> cuttingPlanes;
            trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
            if (editedRow < static_cast<int>(cuttingPlanes.size())) {
                // Get original center
                Eigen::Vector3d originalCenter;
                if (originalCenters_.find(editedRow) != originalCenters_.end()) {
                    originalCenter = originalCenters_[editedRow];
                } else {
                    originalCenter = cuttingPlanes[editedRow].center;
                    originalCenters_[editedRow] = originalCenter;
                } 
                // Calculate new translation_z based on center
                // center is the original position, translation is the offset
                cuttingPlanes[editedRow].translation_z = editedZ - originalCenter.z();
                
                // Update data
                generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;
                
                qDebug() << QString("Updated slice layer %1: Z=%2 [User Edit]")
                    .arg(editedRow + 1)
                    .arg(editedZ, 0, 'f', 3);
                
                // Update UI zTranslationSpinBox if this is the currently selected row
                if (currentSelectedRow_ == editedRow) {
                    bool oldBlockState = blockSignals(true);
                    if (ui->zTranslationSpinBox) {
                        ui->zTranslationSpinBox->setValue(editedZ);
                    }
                    blockSignals(oldBlockState);
                }
            }
        }
        
        // Update table to reflect changes after modifying Z position
        blockTableSignals_ = true;
        updateSlicesTable();
        blockTableSignals_ = false;
           
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
        refreshDataDisplay(); // Refresh OpenGL view
    }
}

// Handle slice table row click event
void SlicingWidgets::onSliceTableRowClicked(int row, int column)
{
    Q_UNUSED(column);  // Column parameter not used here
    
    if (row < 0 || row >= ui->slicesTableWidget->rowCount()) {
        return;  // Invalid row number
    }

    // Keep full-row highlight on user click.
    ui->slicesTableWidget->setCurrentCell(row, 0);
    ui->slicesTableWidget->selectRow(row);
    
    // Get data from the selected row
    QTableWidgetItem* layerItem = ui->slicesTableWidget->item(row, 0);      // Layer column
    QTableWidgetItem* zItem = ui->slicesTableWidget->item(row, 1);          // Z Position column
    QTableWidgetItem* thicknessItem = ui->slicesTableWidget->item(row, 2);  // Thickness column
    QTableWidgetItem* contourItem = ui->slicesTableWidget->item(row, 3);    // Contour Points column
    
    if (!layerItem || !zItem) {
        return;  // Missing required data
    }
    
    // Parse data
    int layerIndex = layerItem->text().toInt();
    double zPosition = zItem->text().toDouble();
    double thickness = thicknessItem ? thicknessItem->text().toDouble() : 0.0;
    int contourPoints = contourItem ? contourItem->text().toInt() : 0;
    
    // Load transformation parameters from generalData_ and update UI controls
    if (trajPlanModel_ && generalData_) {
        std::vector<CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
        
        if (row >= 0 && row < static_cast<int>(cuttingPlanes.size())) {
            // Update currently selected row
            currentSelectedRow_ = row;

            // Mark selected cutting plane for yellow highlight in renderer.
            for (size_t i = 0; i < cuttingPlanes.size(); ++i) {
                cuttingPlanes[i].isSelected = (static_cast<int>(i) == row);
            }
            generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;
            generalData_->slicingData_.lastSelectedLayerIndex = row;
            
            const CuttingPlane& plane = cuttingPlanes[row];

            // Block transformation updates while filling UI controls from selected row.
            blockTransformationSignals_ = true;

            // 1. Update "Translation" x/y/zTranslationSpinBox - show final transformed position
            // Final position = original center + translation
            if (ui->xTranslationSpinBox) ui->xTranslationSpinBox->setValue(plane.center.x() + plane.translation_x);
            if (ui->yTranslationSpinBox) ui->yTranslationSpinBox->setValue(plane.center.y() + plane.translation_y);
            if (ui->zTranslationSpinBox) ui->zTranslationSpinBox->setValue(plane.center.z() + plane.translation_z);            
            // 2. Update "Rotation" x/y/zRotationSpinBox
            if (ui->xRotationSpinBox) ui->xRotationSpinBox->setValue(plane.rotation_x);
            if (ui->yRotationSpinBox) ui->yRotationSpinBox->setValue(plane.rotation_y);
            if (ui->zRotationSpinBox) ui->zRotationSpinBox->setValue(plane.rotation_z);

            blockTransformationSignals_ = false;
            
            qDebug() << QString("Loaded transformation parameters for layer %1 into UI controls").arg(row + 1);
        }
    }
    
    
    // 1. Highlight the selected cutting plane
    if (trajPlanModel_ && renderData_) {
        // Optional: highlight the selected cutting plane in 3D view
        // trajPlanModel_->highlightCuttingPlane(*renderData_, layerIndex);
    }
    
    // 2. Display slice information in status bar or UI
    QString statusInfo = QString("Selected layer %1: Z position %2 mm")
        .arg(layerIndex)
        .arg(zPosition, 0, 'f', 3);
    
    // Optional: show in status bar
    // if (statusBar()) {
    //     statusBar()->showMessage(statusInfo, 3000);  // Show for 3 seconds
    // }
    
    // 3. Adjust 3D view camera if needed
    // Can implement camera focus on selected plane
    
    // 4. Update OpenGL view to show visual feedback
    if (renderData_) {
        renderData_->slicing().needsUpdate = true;
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
    }

    if (openGLView_) {
        openGLView_->update();
    }
    
    // 5. Call custom refresh hook if available
    if (forceRefreshHook_) {
        forceRefreshHook_();
    }
}

// Transformation parameter changed handler - update from UI to model
void SlicingWidgets::onTransformationParameterChanged()
{
    if (blockTransformationSignals_) {
        return;
    }

    // Check if row is selected and valid
    if (currentSelectedRow_ < 0 || !trajPlanModel_ || !generalData_) {
        return;
    }
    
    // Get current cutting planes
    std::vector<CuttingPlane> cuttingPlanes;//�յ�
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
    if (ui->xTranslationSpinBox) newCenter.x() = ui->xTranslationSpinBox->value();
    if (ui->yTranslationSpinBox) newCenter.y() = ui->yTranslationSpinBox->value();
    if (ui->zTranslationSpinBox) newCenter.z() = ui->zTranslationSpinBox->value();
    
    // Calculate translation offset from original center
    plane.translation_x = newCenter.x() - originalCenter.x();
    plane.translation_y = newCenter.y() - originalCenter.y();
    plane.translation_z = newCenter.z() - originalCenter.z();
    
    // Keep center unchanged, only modify translation_x/y/z
    // The transformation is applied via translation_x/y/z in UniformSlicer when rendering
    // plane.center = newCenter;  // Don't modify center here
    
    // Update rotation
    if (ui->xRotationSpinBox) plane.rotation_x = ui->xRotationSpinBox->value();
    if (ui->yRotationSpinBox) plane.rotation_y = ui->yRotationSpinBox->value();
    if (ui->zRotationSpinBox) plane.rotation_z = ui->zRotationSpinBox->value();
    
    // Update cutting planes in generalData_
    // This ensures UI modifications to transformation parameters are reflected in data
    generalData_->slicingData_.cuttingPlanes_ = cuttingPlanes;
    
    
    // Recalculate cutting plane size after transformation (if STL is already processed)
    if (generalData_->processingState_.isSTLProcessed && trajPlanModel_) {
        // Recalculate slice plane size and intersection
        trajPlanModel_->updateSingleCuttingPlaneSize(*generalData_, currentSelectedRow_);
    }
    
    // Update rendering. After transformation, must re-render for visual feedback
    // Transformation parameters need to trigger "Confirm Slicing" logic again
    if (renderData_) {
        renderData_->slicing().needsUpdate = true;
        
        
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
        
        // Refresh display
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
    }
}

void SlicingWidgets::onCancelButtonClicked()
{
    if (trajPlanModel_) {
        // Confirm with user before clearing
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, 
            "Confirm Clear Workspace", 
            "Are you sure you want to clear slicing workspace data?\n\nThis will remove generated slicing planes, private trajectory data, and temporary rendered slicing results.\n\nThe public workpiece model will be kept.",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        
        if (reply == QMessageBox::Yes) {   
            // **Important**: Use clear() to clear all data, not just clearCuttingPlanes()
            // This also resets processing state flags like isSTLProcessed_ and isBoundaryDetected_
            trajPlanModel_->clear(*generalData_);
            
            // **Step 1**: Keep PublicModelData intact. Cancel only clears private slicing workspace data.
            
            // **Step 2**: Clear internal state variables
            originalCenters_.clear();  // Clear stored original centers
            currentSelectedRow_ = -1;  // Reset selected row
            
            // **Step 3**: Hide all rendered data
            if (renderData_) {
                // 1. Hide all visible elements
                trajPlanModel_->setConnectionVisibility(*renderData_, false);
                trajPlanModel_->setTrajectoryVisibility(*renderData_, false);
                trajPlanModel_->setCuttingPlanesVisibility(*renderData_, false);
                trajPlanModel_->setBoundaryVisibility(*renderData_, false);
                trajPlanModel_->setOriginalDataVisibility(*renderData_, false);
                
                // 2. Mark all render data for update
                renderData_->slicing().needsUpdate = true;
                renderData_->trajectory().needsUpdate = true;
                renderData_->boundary().needsUpdate = true;
                renderData_->connection().needsUpdate = true;
                renderData_->stl().needsUpdate = true;
            }
            
            // **Step 4**: Reset UI file path display
            ui->filePathLineEdit->clear();
            ui->filePathLineEdit->setPlaceholderText("Please select files...");
            
            // **Step 5**: Clear table
            blockTableSignals_ = true;
            ui->slicesTableWidget->clearContents();
            ui->slicesTableWidget->setRowCount(0);
            blockTableSignals_ = false;
            
            // **Step 6**: Reset parameter controls
            ui->sizeSpinBox->setValue(50);
            ui->epsilonSpinBox->setValue(5.0);
            ui->minStepSpinBox->setValue(50.0);
            ui->z1OffsetSpinBox->setValue(0.0);
            ui->z2OffsetSpinBox->setValue(0.0);
            ui->offsetDistanceSpinBox->setValue(0.0);
            
            // **Step 7**: Reset transformation parameters
            if (ui->xTranslationSpinBox) ui->xTranslationSpinBox->setValue(0.0);
            if (ui->yTranslationSpinBox) ui->yTranslationSpinBox->setValue(0.0);
            if (ui->zTranslationSpinBox) ui->zTranslationSpinBox->setValue(0.0);
            if (ui->xRotationSpinBox) ui->xRotationSpinBox->setValue(0.0);
            if (ui->yRotationSpinBox) ui->yRotationSpinBox->setValue(0.0);
            if (ui->zRotationSpinBox) ui->zRotationSpinBox->setValue(0.0);
            
            // **Step 8**: Reset buttons
            if (auto* button = optionalButton(this, "generateTrajectoryButton")) button->setEnabled(false);
            if (auto* button = optionalButton(this, "connectTrajectoryButton")) button->setEnabled(false);
            
            // **Step 9**: Use updateUIFromModel() to sync UI with cleared model state
            updateUIFromModel();
            
            
            refreshDataDisplay();
            if (openGLView_) {
                openGLView_->update();
            }
            if (forceRefreshHook_) forceRefreshHook_();
            
            QMessageBox::information(this, "Workspace Cleared", 
                "Slicing workspace data has been cleared.\n\n"
                "The public workpiece model is still available.");
        }
    }
    
    // Cancel button click handled
}

void SlicingWidgets::onDecreaseButtonClicked()
{
    int step = ui->stepLineEdit->text().toInt();
    int currentRow = ui->slicesTableWidget->currentRow();
    
    if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
        QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
        if (zItem) {
            double currentZ = zItem->text().toDouble();
            double newZ = currentZ - step;
            zItem->setText(QString::number(newZ, 'f', 2));
        }
    }
}

void SlicingWidgets::onIncreaseButtonClicked()
{
    int step = ui->stepLineEdit->text().toInt();
    int currentRow = ui->slicesTableWidget->currentRow();
    
    if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
        QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
        if (zItem) {
            double currentZ = zItem->text().toDouble();
            double newZ = currentZ + step;
            zItem->setText(QString::number(newZ, 'f', 2));
        }
    }
}

void SlicingWidgets::onSetTargetButtonClicked()
{
    QString targetZText = ui->targetZLineEdit->text();
    bool ok;
    double targetZ = targetZText.toDouble(&ok);
    
    if (ok) {
        int currentRow = ui->slicesTableWidget->currentRow();
        if (currentRow >= 0 && currentRow < ui->slicesTableWidget->rowCount()) {
            QTableWidgetItem* zItem = ui->slicesTableWidget->item(currentRow, 1);
            if (zItem) {
                zItem->setText(QString::number(targetZ, 'f', 2));
            }
        }
    } else {
        QMessageBox::warning(this, "Warning", "Please enter a valid Z value!");
    }
}

// Update slices table display
void SlicingWidgets::updateSlicesTable()
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
        // Calculate final Z position including transformation: center.z + translation_z (not just center.z)
        double actualZ = plane.center.z() + plane.translation_z;
        ui->slicesTableWidget->setItem(static_cast<int>(i), 1,
            new QTableWidgetItem(QString::number(actualZ, 'f', 3)));
        ui->slicesTableWidget->setItem(static_cast<int>(i), 3,
            new QTableWidgetItem(QString::number(contourPointCount)));
    }

    // Auto-resize columns to fit content
    ui->slicesTableWidget->resizeColumnsToContents();

    // Preserve selected row highlight after table rebuild.
    if (currentSelectedRow_ >= 0 && currentSelectedRow_ < ui->slicesTableWidget->rowCount()) {
        ui->slicesTableWidget->setCurrentCell(currentSelectedRow_, 0);
        ui->slicesTableWidget->selectRow(currentSelectedRow_);
    }
    
    blockTableSignals_ = false;
    
}

// Apply UI parameters to model
void SlicingWidgets::applyParametersToModel()
{
    if (!trajPlanModel_ || !ui) return;
    
    // Get values from UI controls and apply to model (this will trigger valueChanged signals)
    generalData_->slicingData_.managedParams_.trajectorySpacing = ui->sizeSpinBox->value();
    trajPlanModel_->setZ1Offset(*generalData_, ui->z1OffsetSpinBox->value());
    trajPlanModel_->setZ2Offset(*generalData_, ui->z2OffsetSpinBox->value());
    trajPlanModel_->setOffsetDistance(*generalData_, ui->offsetDistanceSpinBox->value());
    trajPlanModel_->setnormalVelocity_(*generalData_, ui->normalVelocity_SpinBox->value());
    trajPlanModel_->settransitionVelocity_(*generalData_, ui->normalVelocity_SpinBox->value());
    generalData_->boundaryData_.epsilon_ = ui->epsilonSpinBox->value();
    generalData_->boundaryData_.minStep_ = ui->minStepSpinBox->value();

}


// Refresh data display
void SlicingWidgets::refreshDataDisplay()
{
    // Ensure we're on the GUI thread
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, "refreshDataDisplay", Qt::QueuedConnection);
        return;
    }

    // === Update from BurTrajPlan model ===
    if (trajPlanModel_ && generalData_) {
        // Enable/disable "Connect Trajectory" button based on trajectory data
        if (auto* button = optionalButton(this, "connectTrajectoryButton")) {
            size_t trajCount;
            trajPlanModel_->getTrajectoryPointCount(*generalData_, trajCount);
            button->setEnabled(trajCount > 0);
        }
    }
    // Update OpenGL widget display��key
    if (openGLView_) {
        openGLView_->update();

    } else {

    }
    // Call custom refresh hook if available for OpenGL updates
    if (forceRefreshHook_) {
        forceRefreshHook_();
    }
    

}

// Reset button clicked - reset all parameters to default
void SlicingWidgets::onResetButtonClicked()
{
    if (!ui) {
        return;
    }
    // Reset UI controls to default values
    ui->sizeSpinBox->setValue(1);
    ui->epsilonSpinBox->setValue(5.0);
    ui->minStepSpinBox->setValue(50.0);
    ui->z1OffsetSpinBox->setValue(0.0);
    ui->z2OffsetSpinBox->setValue(0.0);
    ui->offsetDistanceSpinBox->setValue(0.0);
    
    // Clear file path display
    ui->filePathLineEdit->clear();
    ui->filePathLineEdit->setPlaceholderText("Please select files...");
    
    // Clear table
    blockTableSignals_ = true;
    ui->slicesTableWidget->clearContents();
    ui->slicesTableWidget->setRowCount(0);
    blockTableSignals_ = false;
    
    // Reset buttons
    if (auto* button = optionalButton(this, "generateTrajectoryButton")) button->setEnabled(false);
    if (auto* button = optionalButton(this, "connectTrajectoryButton")) button->setEnabled(false);
    
    // Reset model parameters to default values
    if (trajPlanModel_) {
        // Reset model parameters
        trajPlanModel_->setSlicingSize(*generalData_, 1.0);
        trajPlanModel_->setZ1Offset(*generalData_, 0.0);
        trajPlanModel_->setZ2Offset(*generalData_, 0.0);
        trajPlanModel_->setOffsetDistance(*generalData_, 0.0); // Reset offsetDistance parameter
        
        // Clear custom Z positions
        std::vector<double> emptyZs;
        trajPlanModel_->setCustomZPositions(*generalData_, emptyZs);
        
        // Refresh display and OpenGL view
        refreshDataDisplay();
    }
    
    QMessageBox::information(this, "Info", "All parameters have been reset to default values!");
}

// Browse button clicked - select files to load
void SlicingWidgets::onBrowseButtonClicked()
{
    if (!m_hasActiveWorkspace) {
        QMessageBox::warning(this,
            "No Active Project",
            "Please create or load a project first.");
        return;
    }

    // ʹ��DATA_PATH��ΪĬ��·��
    std::string data_path = DATA_PATH;
    QString defaultPath = QString::fromStdString(data_path + "/model");
    
    // ���޸ġ�ֻѡ��STL�ļ�
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Select STL File",
        defaultPath,
        "STL Files (*.stl);;All Files (*.*)");
    
    if (fileName.isEmpty()) {
        return;
    }
    
    // Check required data pointers
    if (!publicModelData_) {
        QMessageBox::warning(this, "Error", "PublicModelData is not initialized");
        return;
    }
    
    if (!generalData_) {
        QMessageBox::warning(this, "Error", "GeneralData is not initialized");
        return;
    }
    
    // 1. Load STL file using PublicModelData
    bool loadSuccess = publicModelData_->loadModel(fileName.toStdString());
    if (!loadSuccess) {
        QMessageBox::warning(this, "Error", "Failed to load STL file");
        return;
    }
    
    // 2. Copy from PublicModel to PrivateModel (GeneralData)
    const PublicData::PublicModel& publicModel = publicModelData_->getModel();
    
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Error", "TrajPlanModel is not initialized");
        return;
    }
    
    bool copySuccess = trajPlanModel_->loadModelFromPublicData(publicModel, *generalData_);
    
    if (!copySuccess) {
        QMessageBox::warning(this, "Error", "Failed to copy model data");
        return;
    }
    
    // 3a. Public STL overlay: import on the main view's GL context (meshModel belongs to that scene).
    if (publicDataRender_ && publicModelData_) {
        publicDataRender_->m_PublicSTLRenderData.needsUpdate = true;
        publicDataRender_->m_PublicSTLRenderData.isVisible = true;
        if (openGLView_) {
            openGLView_->makeCurrent();
            PublicData::PublicDataRender::drawPublicModel(
                publicDataRender_->m_PublicSTLRenderData, *publicModelData_);
            openGLView_->doneCurrent();
        }
    }

    // 3b. BurTrajPlan pipeline (STL / slice views in RenderData)
    if (renderData_ && trajPlanModel_) {
        trajPlanModel_->initializeRenderer(*renderData_);
        trajPlanModel_->setOriginalDataVisibility(*renderData_, true);
    }
    
    // 4. Synchronize UI state (including action button enabled states),
    // then refresh OpenGL display.
    updateUIFromModel();
    refreshDataDisplay();
    
    // 5. Keep file path text aligned with explicit user selection.
    ui->filePathLineEdit->setText(fileName);
    
    // 6. Show success message
    QMessageBox::information(this, "Success", 
        QString("Model loaded successfully\nFile: %1\nTriangles: %2")
            .arg(QString::fromStdString(publicModel.m_file_name))
            .arg(publicModel.m_indices.size() / 3));
}

// Coarse button clicked - apply coarse preset������Ԥ��
void SlicingWidgets::onCoarseButtonClicked()
{
    if (trajPlanModel_) {
        trajPlanModel_->applyCoarsePreset(*generalData_);
        updateUIFromModel();
        
        // Refresh display and OpenGL view
        refreshDataDisplay();
        
        QMessageBox::information(this, "Coarse Preset", "Coarse preset parameters applied.");
    } else {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
    }
}

// Update UI from model data
void SlicingWidgets::updateUIFromModel()
{
    if (!trajPlanModel_ || !generalData_) {
        // If no model or data available, clear UI
        if (ui) {
            ui->filePathLineEdit->clear();
            ui->filePathLineEdit->setPlaceholderText("Please select files...");
            ui->slicesTableWidget->clearContents();
            ui->slicesTableWidget->setRowCount(0);
        }
        return;
    }
    
    // Block signals to prevent triggering UI update events
    const bool oldState = blockSignals(true);
    
    try {
        // Get parameters from model and update UI controls
        int slicingSize;
        double z1Offset, z2Offset, offsetDistance;
        
        trajPlanModel_->getSlicingSize(*generalData_, slicingSize);
        trajPlanModel_->getZ1Offset(*generalData_, z1Offset);
        trajPlanModel_->getZ2Offset(*generalData_, z2Offset);
        trajPlanModel_->getOffsetDistance(*generalData_, offsetDistance);
        
        ui->sizeSpinBox->setValue(generalData_->slicingData_.managedParams_.trajectorySpacing);
        ui->epsilonSpinBox->setValue(generalData_->boundaryData_.epsilon_);
        ui->minStepSpinBox->setValue(generalData_->boundaryData_.minStep_);
        ui->z1OffsetSpinBox->setValue(z1Offset);
        ui->z2OffsetSpinBox->setValue(z2Offset);
        ui->offsetDistanceSpinBox->setValue(offsetDistance);
        
        // Update file path display
        std::vector<std::string> filePaths;
        trajPlanModel_->getFilePaths(*generalData_, filePaths);
        if (!filePaths.empty()) {
            QStringList pathList;
            for (const auto& path : filePaths) {
                pathList << QString::fromStdString(path);
            }
            ui->filePathLineEdit->setText(pathList.join("; "));
        } else {
            ui->filePathLineEdit->clear();
            ui->filePathLineEdit->setPlaceholderText("Please select files...");
        }
        
        // Update slices table
        updateSlicesTable();

        // Re-evaluate action buttons after loading/restoring data.
        const bool hasModelData =
            generalData_->processingState_.isSTLProcessed ||
            generalData_->modelDataManager_.hasValidModelData_ ||
            !generalData_->modelDataManager_.mainModelData_.m_vertices.empty();
        const bool hasSliceData =
            !generalData_->slicingData_.cuttingPlanes_.empty() ||
            !generalData_->slicingData_.sliceLines_.empty();
        const bool hasTrajectoryData =
            !generalData_->trajData_.trajectoryPoints_.empty();

        if (auto* button = optionalButton(this, "confirmButton")) {
            button->setEnabled(hasModelData);
        }
        if (auto* button = optionalButton(this, "generateTrajectoryButton")) {
            button->setEnabled(hasSliceData);
        }
        if (auto* button = optionalButton(this, "connectTrajectoryButton")) {
            button->setEnabled(hasTrajectoryData);
        }
        if (auto* button = optionalButton(this, "cancelButton")) {
            button->setEnabled(true);
        }
        if (auto* button = optionalButton(this, "TrajOptiButton")) {
            button->setEnabled(true);
        }
        
        // Refresh display
        refreshDataDisplay();
        
        // Update range label if data exists
        size_t totalPointCount;
        trajPlanModel_->getTotalPointCount(*generalData_, totalPointCount);
        if (totalPointCount > 0) {
            // Set range label with actual data range
            ui->rangeLabel->setText("Range: [0.0, 1123.4]");
        }      
    } catch (const std::exception& e) {

        QMessageBox::warning(this, "Warning", 
            QString("Failed to load parameters from model: %1").arg(e.what()));
    }
    
    // Restore signal state
    blockSignals(oldState);
}



// Parameter changed event handler
void SlicingWidgets::onParameterChanged()
{
    if (!trajPlanModel_ || !generalData_) return;
    
    // Apply UI parameters to model
    generalData_->slicingData_.managedParams_.trajectorySpacing = ui->sizeSpinBox->value();
    trajPlanModel_->setZ1Offset(*generalData_, ui->z1OffsetSpinBox->value());
    trajPlanModel_->setZ2Offset(*generalData_, ui->z2OffsetSpinBox->value());
    trajPlanModel_->setOffsetDistance(*generalData_, ui->offsetDistanceSpinBox->value());
    generalData_->boundaryData_.epsilon_ = ui->epsilonSpinBox->value();
    generalData_->boundaryData_.minStep_ = ui->minStepSpinBox->value();
    
    // Note: Parameters changed, may need to regenerate cutting planes
    
    // Optionally disable generate button until slicing is confirmed again
    // if (auto* button = optionalButton(this, "generateTrajectoryButton")) button->setEnabled(false); // Force re-confirmation
}


// Confirm slicing button clicked - generate cutting planes
void SlicingWidgets::onConfirmButtonClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }
    
    // Apply UI parameters to model

    applyParametersToModel();

    // Clear any custom Z positions from UI (use uniform intervals instead)
    // Custom Z positions would come from table editing

    std::vector<double> emptyZs;
    trajPlanModel_->setCustomZPositions(*generalData_, emptyZs); // Clear custom positions
    
    // Check if STL file has been processed
    if (!trajPlanModel_->isSTLProcessed(*generalData_)) {
        QMessageBox::warning(this, "Warning", "STL file not processed yet. Please browse and load an STL file first.");
        return;
    }
    

    
    // Keep original data visible during slicing
    trajPlanModel_->setKeepOriginalVisibleAfterPreprocess(true);
    trajPlanModel_->setOriginalDataVisibility(*renderData_, true);

    // Clear previous cutting planes before generating new ones

    trajPlanModel_->clearCuttingPlanes(*generalData_);

    // A regenerated slice set invalidates all trajectory data derived from
    // the former planes, including the public render cache.
    if (publicTrajData_) {
        publicTrajData_->clear();
    }
    if (publicDataRender_) {
        auto& publicTrajectoryRender = publicDataRender_->m_PublicTrajectoryRenderData;
        publicTrajectoryRender.isVisible = false;
        publicTrajectoryRender.needsUpdate = true;
        PublicData::PublicDataRender::clearMeshData(publicTrajectoryRender.trajectoryData);
    }
    if (renderData_) {
        trajPlanModel_->setTrajectoryVisibility(*renderData_, false);
        trajPlanModel_->setConnectionVisibility(*renderData_, false);
        renderData_->slicing().needsUpdate = true;
        renderData_->trajectory().needsUpdate = true;
        renderData_->connection().needsUpdate = true;
    }
    
    // Generate new cutting planes

    if (trajPlanModel_->generateCuttingPlanes(*generalData_, renderData_)) {
        std::vector<CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);

        // Save original centers for all cutting planes (for transformation reference)
        originalCenters_.clear();
        for (size_t i = 0; i < cuttingPlanes.size(); ++i) {
            originalCenters_[static_cast<int>(i)] = cuttingPlanes[i].center;
        }
        
        // Show cutting planes and boundaries in render view
        if (renderData_) {
            trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
            // **Important**: Show boundary if detected during generateCuttingPlanes
            if (trajPlanModel_->isBoundaryDetected(*generalData_)) {
                trajPlanModel_->setBoundaryVisibility(*renderData_, true);
            }
        }
        
        // Update slices table with newly generated planes
        blockTableSignals_ = true;  // Prevent table signals during update
        updateSlicesTable();
        blockTableSignals_ = false;
        
        // Render data might need an update
         //if (trajPlanModel_->needsRenderUpdate()) {
         //    trajPlanModel_->updateRenderData();
         //}
        
        // Refresh OpenGL display
        refreshDataDisplay();

        // Enable trajectory generation button
        if (auto* button = optionalButton(this, "generateTrajectoryButton")) button->setEnabled(true);

        QMessageBox::information(this, "Success",
            QString("Slicing generation completed! Generated %1 cutting planes with updated parameters.\nYou can now click the trajectory generation button to create the final trajectory.").arg(cuttingPlanes.size()));
    } else {

        QMessageBox::critical(this, "Error", "Uniform slicing failed! Please check if the file is loaded correctly and parameters are valid.");
    }
}

void SlicingWidgets::onExportContourLengthReportClicked()
{
    if (!generalData_) {
        QMessageBox::warning(this, tr("Export Contour Length Report"), tr("Slicing data is unavailable."));
        return;
    }

    struct ContourRecord {
        QString label;
        double level = 0.0;
        double length = 0.0;
        std::vector<Eigen::Vector3d> points;
    };

    const auto contourLength = [](const BoundaryLine& contour) {
        if (contour.points.size() < 2) return 0.0;
        double length = 0.0;
        for (size_t index = 1; index < contour.points.size(); ++index) {
            length += (contour.points[index].position - contour.points[index - 1].position).norm();
        }
        return length + (contour.points.front().position - contour.points.back().position).norm();
    };
    const auto appendContour = [&contourLength](std::vector<ContourRecord>& records,
        const QString& label, const BoundaryLine& contour) {
        if (contour.points.size() < 2) return;
        ContourRecord record;
        record.label = label;
        record.level = contour.z_level;
        record.length = contourLength(contour);
        record.points.reserve(contour.points.size());
        for (const auto& point : contour.points) record.points.push_back(point.position);
        records.push_back(std::move(record));
    };

    std::vector<ContourRecord> records;
    appendContour(records, tr("Lower boundary"), generalData_->boundaryData_.lowerBoundary_);
    const auto& sliceLines = generalData_->slicingData_.sliceLines_.empty()
        ? generalData_->slicingData_.currentSliceResult_.internalBoundaries
        : generalData_->slicingData_.sliceLines_;
    for (size_t index = 0; index < sliceLines.size(); ++index) {
        appendContour(records, tr("Slice %1").arg(index + 1), sliceLines[index]);
    }
    appendContour(records, tr("Upper boundary"), generalData_->boundaryData_.upperBoundary_);
    if (records.size() < 2) {
        QMessageBox::warning(this, tr("Export Contour Length Report"),
            tr("Generate slicing results before exporting the contour report."));
        return;
    }

    const QString outputDirectory = QFileDialog::getExistingDirectory(
        this, tr("Select Contour Report Directory"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (outputDirectory.isEmpty()) return;

    const QString reportDirectory = QDir(outputDirectory).filePath(
        QStringLiteral("contour_length_report_") + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    if (!QDir().mkpath(reportDirectory)) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot create the report directory."));
        return;
    }

    const QString csvPath = QDir(reportDirectory).filePath(QStringLiteral("contour_lengths.csv"));
    QSaveFile csvFile(csvPath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot write the CSV file."));
        return;
    }
    QTextStream csv(&csvFile);
    csv.setCodec("UTF-8");
    csv << "sequence,label,level,length_mm\n";
    for (size_t index = 0; index < records.size(); ++index) {
        csv << (index + 1) << ',' << records[index].label << ','
            << QString::number(records[index].level, 'f', 6) << ','
            << QString::number(records[index].length, 'f', 6) << '\n';
    }
    if (!csvFile.commit()) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Failed to save the CSV file."));
        return;
    }

    // Each generated track has one sampled point per slice layer.  Compare
    // matching layer samples only, so transition moves and inter-layer motion
    // do not contaminate the adjacent-track spacing distribution.
    std::vector<double> adjacentTrackSpacings;
    const auto& sampledTracks = generalData_->trajData_.sampledBoundaryPoints_;
    for (size_t trackIndex = 1; trackIndex < sampledTracks.size(); ++trackIndex) {
        const auto& previousTrack = sampledTracks[trackIndex - 1];
        const auto& currentTrack = sampledTracks[trackIndex];
        const size_t layerCount = std::min(previousTrack.size(), currentTrack.size());
        for (size_t layerIndex = 0; layerIndex < layerCount; ++layerIndex) {
            const double spacing = (currentTrack[layerIndex] - previousTrack[layerIndex]).norm();
            if (std::isfinite(spacing)) adjacentTrackSpacings.push_back(spacing);
        }
    }

    QString spacingImageName;
    QString spacingCsvName;
    double spacingMean = 0.0;
    double spacingP05 = 0.0;
    double spacingP95 = 0.0;
    if (!adjacentTrackSpacings.empty()) {
        std::sort(adjacentTrackSpacings.begin(), adjacentTrackSpacings.end());
        spacingMean = std::accumulate(adjacentTrackSpacings.begin(), adjacentTrackSpacings.end(), 0.0)
            / static_cast<double>(adjacentTrackSpacings.size());
        const auto percentile = [&adjacentTrackSpacings](double fraction) {
            const double position = fraction * static_cast<double>(adjacentTrackSpacings.size() - 1);
            const size_t lower = static_cast<size_t>(std::floor(position));
            const size_t upper = static_cast<size_t>(std::ceil(position));
            const double weight = position - static_cast<double>(lower);
            return adjacentTrackSpacings[lower] * (1.0 - weight) + adjacentTrackSpacings[upper] * weight;
        };
        spacingP05 = percentile(0.05);
        spacingP95 = percentile(0.95);

        spacingCsvName = QStringLiteral("adjacent_track_spacing_samples.csv");
        QSaveFile spacingCsvFile(QDir(reportDirectory).filePath(spacingCsvName));
        if (!spacingCsvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot write the spacing CSV file."));
            return;
        }
        QTextStream spacingCsv(&spacingCsvFile);
        spacingCsv.setCodec("UTF-8");
        spacingCsv << "sample,adjacent_track_spacing_mm\n";
        for (size_t index = 0; index < adjacentTrackSpacings.size(); ++index) {
            spacingCsv << (index + 1) << ',' << QString::number(adjacentTrackSpacings[index], 'f', 6) << '\n';
        }
        if (!spacingCsvFile.commit()) {
            QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Failed to save the spacing CSV file."));
            return;
        }

        const double minimum = adjacentTrackSpacings.front();
        const double maximum = adjacentTrackSpacings.back();
        const double dataRange = std::max(1e-6, maximum - minimum);
        const int binCount = std::clamp(static_cast<int>(std::sqrt(adjacentTrackSpacings.size())), 12, 60);
        std::vector<int> bins(static_cast<size_t>(binCount), 0);
        for (const double value : adjacentTrackSpacings) {
            const int bin = std::min(binCount - 1,
                static_cast<int>((value - minimum) / dataRange * static_cast<double>(binCount)));
            ++bins[static_cast<size_t>(bin)];
        }
        const int maximumCount = *std::max_element(bins.begin(), bins.end());
        constexpr int histogramWidth = 1600, histogramHeight = 900, histogramMarginLeft = 110,
            histogramMarginRight = 50, histogramMarginTop = 85, histogramMarginBottom = 100;
        QImage histogram(histogramWidth, histogramHeight, QImage::Format_ARGB32_Premultiplied);
        histogram.fill(Qt::white);
        QPainter histogramPainter(&histogram);
        histogramPainter.setRenderHint(QPainter::Antialiasing);
        const QRectF plotArea(histogramMarginLeft, histogramMarginTop,
            histogramWidth - histogramMarginLeft - histogramMarginRight,
            histogramHeight - histogramMarginTop - histogramMarginBottom);
        const auto xForSpacing = [&](double value) {
            return plotArea.left() + (value - minimum) / dataRange * plotArea.width();
        };
        histogramPainter.setPen(QPen(QColor(220, 225, 230), 1));
        for (int line = 0; line <= 5; ++line) {
            const double y = plotArea.bottom() - plotArea.height() * static_cast<double>(line) / 5.0;
            histogramPainter.drawLine(QPointF(plotArea.left(), y), QPointF(plotArea.right(), y));
            histogramPainter.setPen(QPen(QColor(70, 70, 70), 1));
            histogramPainter.drawText(QRectF(5, y - 12, histogramMarginLeft - 12, 24), Qt::AlignRight | Qt::AlignVCenter,
                QString::number(static_cast<int>(std::round(maximumCount * static_cast<double>(line) / 5.0))));
            histogramPainter.setPen(QPen(QColor(220, 225, 230), 1));
        }
        const double bandLeft = xForSpacing(std::clamp(spacingP05, minimum, maximum));
        const double bandRight = xForSpacing(std::clamp(spacingP95, minimum, maximum));
        histogramPainter.fillRect(QRectF(QPointF(bandLeft, plotArea.top()), QPointF(bandRight, plotArea.bottom())),
            QColor(255, 215, 80, 75));
        const double binWidth = plotArea.width() / static_cast<double>(binCount);
        histogramPainter.setPen(QPen(QColor(30, 100, 120), 1));
        histogramPainter.setBrush(QColor(65, 145, 165, 190));
        for (int bin = 0; bin < binCount; ++bin) {
            const double height = plotArea.height() * static_cast<double>(bins[static_cast<size_t>(bin)])
                / static_cast<double>(std::max(1, maximumCount));
            histogramPainter.drawRect(QRectF(plotArea.left() + bin * binWidth, plotArea.bottom() - height,
                std::max(1.0, binWidth - 1.0), height));
        }
        const double targetSpacing = generalData_->slicingData_.managedParams_.trajectorySpacing;
        const auto drawReference = [&histogramPainter, &plotArea, &xForSpacing](double value, const QColor& color,
            Qt::PenStyle style) {
            const double x = xForSpacing(value);
            histogramPainter.setPen(QPen(color, 3, style));
            histogramPainter.drawLine(QPointF(x, plotArea.top()), QPointF(x, plotArea.bottom()));
        };
        if (targetSpacing >= minimum && targetSpacing <= maximum) drawReference(targetSpacing, QColor(230, 60, 55), Qt::SolidLine);
        drawReference(spacingMean, QColor(35, 35, 35), Qt::DashLine);
        histogramPainter.setPen(QPen(Qt::black, 1));
        histogramPainter.setBrush(Qt::NoBrush);
        histogramPainter.drawRect(plotArea);
        histogramPainter.drawText(QRectF(0, 20, histogramWidth, 35), Qt::AlignCenter,
            tr("Adjacent-track 3D Spacing Distribution"));
        histogramPainter.drawText(QRectF(plotArea.left(), histogramHeight - 62, plotArea.width(), 28), Qt::AlignCenter,
            tr("Actual adjacent-track spacing (mm)"));
        histogramPainter.save();
        histogramPainter.translate(28, plotArea.center().y());
        histogramPainter.rotate(-90);
        histogramPainter.drawText(QRectF(-plotArea.height() / 2.0, -14, plotArea.height(), 28), Qt::AlignCenter, tr("Sample count"));
        histogramPainter.restore();
        histogramPainter.drawText(QRectF(plotArea.right() - 390, plotArea.top() + 12, 370, 78), Qt::AlignRight | Qt::AlignTop,
            tr("Target d = %1 mm\nMean = %2 mm\nP05-P95 = %3 - %4 mm")
                .arg(targetSpacing, 0, 'f', 3).arg(spacingMean, 0, 'f', 4)
                .arg(spacingP05, 0, 'f', 4).arg(spacingP95, 0, 'f', 4));
        histogramPainter.end();
        spacingImageName = QStringLiteral("adjacent_track_spacing_distribution.png");
        if (!histogram.save(QDir(reportDirectory).filePath(spacingImageName))) {
            QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot save the spacing distribution image."));
            return;
        }
    }

    // Project all contours onto their principal two-dimensional plane so the
    // diagram remains meaningful for both parallel and adaptive slicing.
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    size_t pointCount = 0;
    for (const auto& record : records) {
        for (const auto& point : record.points) { mean += point; ++pointCount; }
    }
    mean /= static_cast<double>(pointCount);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& record : records) {
        for (const auto& point : record.points) {
            const Eigen::Vector3d delta = point - mean;
            covariance += delta * delta.transpose();
        }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const Eigen::Matrix3d axes = solver.info() == Eigen::Success
        ? solver.eigenvectors() : Eigen::Matrix3d::Identity();
    const Eigen::Vector3d axisU = axes.col(2);
    const Eigen::Vector3d axisV = axes.col(1);
    double minU = std::numeric_limits<double>::max(), maxU = std::numeric_limits<double>::lowest();
    double minV = minU, maxV = maxU;
    for (const auto& record : records) for (const auto& point : record.points) {
        const Eigen::Vector3d delta = point - mean;
        const double u = delta.dot(axisU), v = delta.dot(axisV);
        minU = std::min(minU, u); maxU = std::max(maxU, u);
        minV = std::min(minV, v); maxV = std::max(maxV, v);
    }
    const double rangeU = std::max(1e-8, maxU - minU);
    const double rangeV = std::max(1e-8, maxV - minV);
    constexpr int imageWidth = 1600, imageHeight = 1200, margin = 90;
    QImage image(imageWidth, imageHeight, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::white);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(Qt::black, 1));
    painter.drawText(QRect(margin, 20, imageWidth - 2 * margin, 35), Qt::AlignCenter,
        tr("Slicing Contour Length Map - PCA Projection"));
    const auto project = [&](const Eigen::Vector3d& point) {
        const Eigen::Vector3d delta = point - mean;
        const double u = delta.dot(axisU), v = delta.dot(axisV);
        return QPointF(margin + (u - minU) / rangeU * (imageWidth - 2.0 * margin),
            imageHeight - margin - (v - minV) / rangeV * (imageHeight - 2.0 * margin));
    };
    for (size_t index = 0; index < records.size(); ++index) {
        const QColor color = index == 0 ? QColor(30, 90, 220)
            : index + 1 == records.size() ? QColor(220, 50, 45)
            : QColor::fromHsv(static_cast<int>((index * 280) / std::max<size_t>(1, records.size() - 1)), 210, 180);
        QPainterPath path;
        const QPointF first = project(records[index].points.front());
        path.moveTo(first);
        QPointF labelPoint = first;
        for (size_t pointIndex = 1; pointIndex < records[index].points.size(); ++pointIndex) {
            const QPointF p = project(records[index].points[pointIndex]);
            path.lineTo(p);
            labelPoint += p;
        }
        path.closeSubpath();
        painter.setPen(QPen(color, 2.2));
        painter.drawPath(path);
        labelPoint /= static_cast<qreal>(records[index].points.size());
        painter.setPen(QPen(color.darker(160), 1));
        painter.drawText(labelPoint + QPointF(6, -6),
            QStringLiteral("%1: %2 mm").arg(records[index].label).arg(records[index].length, 0, 'f', 2));
    }
    painter.end();
    const QString imagePath = QDir(reportDirectory).filePath(QStringLiteral("contour_length_map.png"));
    if (!image.save(imagePath)) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot save the contour map image."));
        return;
    }

    const double firstLength = records.front().length;
    const double lastLength = records.back().length;
    double maximumAdjacentChange = 0.0;
    for (size_t index = 1; index < records.size(); ++index) {
        maximumAdjacentChange = std::max(maximumAdjacentChange,
            std::abs(records[index].length - records[index - 1].length));
    }
    const double relativeChange = std::abs(firstLength) > 1e-8
        ? (lastLength - firstLength) / firstLength * 100.0 : 0.0;
    const QString trend = relativeChange > 1.0 ? tr("Increasing")
        : relativeChange < -1.0 ? tr("Decreasing") : tr("Stable");

    const QString htmlPath = QDir(reportDirectory).filePath(QStringLiteral("contour_length_report.html"));
    QSaveFile htmlFile(htmlPath);
    if (!htmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Cannot write the HTML report."));
        return;
    }
    QTextStream html(&htmlFile);
    html.setCodec("UTF-8");
    html << "<!doctype html><meta charset=\"utf-8\"><title>Contour Length Report</title>"
         << "<style>body{font-family:Arial,sans-serif;margin:32px;color:#172033}table{border-collapse:collapse;width:100%}th,td{border:1px solid #cbd5e1;padding:7px;text-align:right}th{background:#e2e8f0}td:nth-child(2){text-align:left}img{max-width:100%;border:1px solid #cbd5e1}</style>"
         << "<h1>Slicing Contour Length Report</h1><p>Trend: <b>" << trend << "</b>; total change: "
         << QString::number(relativeChange, 'f', 2) << "% ; maximum adjacent change: "
         << QString::number(maximumAdjacentChange, 'f', 3) << " mm.</p>"
         << "<table><tr><th>#</th><th>Contour</th><th>Level</th><th>Length (mm)</th></tr>";
    for (size_t index = 0; index < records.size(); ++index) {
        html << "<tr><td>" << index + 1 << "</td><td>" << records[index].label
             << "</td><td>" << QString::number(records[index].level, 'f', 4)
             << "</td><td>" << QString::number(records[index].length, 'f', 4) << "</td></tr>";
    }
    html << "</table>";
    if (!spacingImageName.isEmpty()) {
        html << "<h2>Adjacent-track 3D Spacing Distribution</h2><p>Samples: "
             << adjacentTrackSpacings.size() << "; mean: " << QString::number(spacingMean, 'f', 4)
             << " mm; P05-P95: " << QString::number(spacingP05, 'f', 4) << " - "
             << QString::number(spacingP95, 'f', 4) << " mm. Raw data: <a href=\""
             << spacingCsvName << "\">" << spacingCsvName << "</a>.</p><img src=\""
             << spacingImageName << "\">";
    } else {
        html << "<h2>Adjacent-track 3D Spacing Distribution</h2><p>Unavailable: generate trajectories before exporting this distribution.</p>";
    }
    html << "<h2>Annotated Contours</h2><img src=\"contour_length_map.png\"></body>";
    if (!htmlFile.commit()) {
        QMessageBox::critical(this, tr("Export Contour Length Report"), tr("Failed to save the HTML report."));
        return;
    }

    QMessageBox::information(this, tr("Contour Length Report Exported"),
        tr("CSV data, HTML analysis, annotated contours, and the available spacing distribution were exported to:\n%1")
            .arg(reportDirectory));
}


// Generate Trajectory button clicked - corresponds to MATLAB's main_uigetfile.m
void SlicingWidgets::onGenerateTrajectoryButtonClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }   
    try {
        // Check if slice data exists
        std::vector<CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
        if (cuttingPlanes.empty()) {
            QMessageBox::warning(this, "Warning", "No slice data! Please click confirm slicing button to generate slices first.");
            return;
        }
        
        // Get parameters from UI
        double trajectorySpacing = ui->sizeSpinBox->value();
        double z1Offset = ui->z1OffsetSpinBox->value();
        double z2Offset = ui->z2OffsetSpinBox->value();
        double offsetDistance = ui->offsetDistanceSpinBox->value();
        
        // Use BurTrajPlan's MATLAB trajectory generation functionality
        if (!trajPlanModel_->generateMATLABTrajectory(*generalData_, static_cast<int>(std::lround(trajectorySpacing)), z1Offset, z2Offset, offsetDistance,*publicTrajData_)) {
            QMessageBox::critical(this, "Error", "MATLAB-style trajectory generation failed! Please check the debug output for details.");
            return;
        }
             
        // Get generated trajectory points
        std::vector<TrajectoryPoint> trajectoryPoints;
        trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
        if (publicDataRender_ && publicTrajData_) {
            publicDataRender_->m_PublicTrajectoryRenderData.isVisible = true;
            publicDataRender_->m_PublicTrajectoryRenderData.needsUpdate = true;
            if (openGLView_) {
                openGLView_->makeCurrent();
                PublicData::PublicDataRender::drawPublicTrajectory(
                    publicDataRender_->m_PublicTrajectoryRenderData, *publicTrajData_);
                openGLView_->doneCurrent();
            }
        }

        if (trajectoryPoints.empty()) {
            QMessageBox::critical(this, "Error", "Trajectory generation succeeded but no trajectory points were created!");
            return;
        }   
        // Trajectory generated
        // Show trajectory in the rendering view
        if (renderData_) {
            trajPlanModel_->setTrajectoryVisibility(*renderData_, true);
        }
        
        // Enable connect trajectory button
        if (auto* button = optionalButton(this, "connectTrajectoryButton")) {
            button->setEnabled(true);
        }
        if (auto* button = optionalButton(this, "TrajOptiButton")) {
            button->setEnabled(true);
        }
        
        // Refresh display
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }       
        // Save trajectory files to data/traj directory
        // ʹ��DATA_PATH���ȡ����Ŀ¼
        std::string data_path = DATA_PATH;
        QString trajDir = QString::fromStdString(data_path + "/traj");
        
        // Create directory if it doesn't exist
        QDir().mkpath(trajDir);

        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString suggestedName = "trajectory_output_" + timestamp;
        
        QString selectedDir = QFileDialog::getExistingDirectory(
            this,
            "Select Directory to Save Trajectory Files",
            trajDir,  // Default to data/traj directory
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        
        if (selectedDir.isEmpty()) {

            return; // User cancelled
        }
        
        QString baseDir = selectedDir + "/" + suggestedName;
        QDir().mkpath(baseDir);
        

        
        // 9. Export position and quaternion to file (corresponding to MATLAB writePositionsAndQuaternionsToFile)
        QString paqFile = baseDir + "/new_PAQ.txt";
        if (trajPlanModel_->exportTrajectoryToFile(*generalData_, paqFile.toStdString())) {

        }
        
        // 10. Generate FANUC program (corresponding to MATLAB generateFANUCProgram)
        QString fanucFile = baseDir + "/gj2.ls";
        if (trajPlanModel_->exportFANUCProgram(*generalData_, fanucFile.toStdString())) {

        }
        
        // 11. Export to Excel (basic version - position and Euler angles only)
        QString excelFile = baseDir + "/combined_data_with_eulerAngles.csv";
        if (trajPlanModel_->exportToExcel(*generalData_, excelFile.toStdString())) {

        }
        
        // 11.5 ��������Export to Excel with velocity (complete version - includes velocity, acceleration, etc.)
        QString excelFileWithVelocity = baseDir + "/trajectory_complete_data.csv";
        if (trajPlanModel_->exportToExcelWithVelocity(*generalData_, excelFileWithVelocity.toStdString())) {

        }
        
        // 12. Generate ABB program (corresponding to MATLAB generateABBProgram)
        QString abbFile = baseDir + "/MainModule.mod";
        if (trajPlanModel_->exportABBProgram(*generalData_, paqFile.toStdString(), abbFile.toStdString())) {

        }  
        // Display success message
        QString successMsg = QString("Trajectory generation completed!\n\nGenerated files saved in:\n%1\n\nContains the following files:\n")
                             .arg(baseDir);
        successMsg += " new_PAQ.txt - Position and quaternion data\n";
        successMsg += " gj2.ls - FANUC robot program\n";
        successMsg += " MainModule.mod - ABB robot program\n";
        successMsg += " combined_data_with_eulerAngles.csv - Excel data (basic)\n";
        successMsg += " trajectory_complete_data.csv - Excel data (with velocity & acceleration)\n";
        
        QMessageBox::information(this, "Trajectory Generation Complete", successMsg);
        
        // Optional: Disable button after generation to prevent re-running
        // if (auto* button = optionalButton(this, "generateTrajectoryButton")) button->setEnabled(false); // Disable button
        // Trajectory generated state is reflected by the host action controls.
        

        
    } catch (const std::exception& e) {

        QMessageBox::critical(this, "Error", QString("Trajectory generation failed: %1").arg(e.what()));
    }
}

// ==================== Context Menu Handlers ====================

// Context menu request handler for file path line edit
void SlicingWidgets::onFilePathLineEditContextMenuRequested(const QPoint& pos)
{
    Q_UNUSED(pos);
    // Create context menu
    QMenu contextMenu(this);
    
    // Add actions
    QAction* hideModelAction = contextMenu.addAction("Hidden Model");
    QAction* showModelAction = contextMenu.addAction("Visible Model");
    contextMenu.addSeparator();
    QAction* hideTrajectoryAction = contextMenu.addAction("Hidden Trajectory");
    QAction* showTrajectoryAction = contextMenu.addAction("Visible Trajectory");
    contextMenu.addSeparator();
    QAction* hideSlicingPlaneAction = contextMenu.addAction("Hide Slicing Plane");
    QAction* showSlicingPlaneAction = contextMenu.addAction("Show Slicing Plane");
    
    // Connect actions to slots
    connect(hideModelAction, &QAction::triggered, this, &SlicingWidgets::onHideModelClicked);
    connect(showModelAction, &QAction::triggered, this, &SlicingWidgets::onShowModelClicked);
    connect(hideTrajectoryAction, &QAction::triggered, this, &SlicingWidgets::onHideTrajectoryClicked);
    connect(showTrajectoryAction, &QAction::triggered, this, &SlicingWidgets::onShowTrajectoryClicked);
    connect(hideSlicingPlaneAction, &QAction::triggered, this, &SlicingWidgets::onHideSlicingPlaneClicked);
    connect(showSlicingPlaneAction, &QAction::triggered, this, &SlicingWidgets::onShowSlicingPlaneClicked);
    
    // Show menu
    contextMenu.exec(QCursor::pos());
}

// Hide model clicked
void SlicingWidgets::onHideModelClicked()
{
    if (trajPlanModel_ && renderData_) {
        // Hide original STL model
        trajPlanModel_->setOriginalDataVisibility(*renderData_, false);
        
        // Hide cutting planes
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, false);
        
        // Hide boundary lines
        trajPlanModel_->setBoundaryVisibility(*renderData_, false);
      
        // Refresh display
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
        

    }
}

// Show model clicked
void SlicingWidgets::onShowModelClicked()
{
    if (trajPlanModel_ && renderData_) {
        // Show original STL model
        trajPlanModel_->setOriginalDataVisibility(*renderData_, true);
        
        // Show cutting planes (if they exist)
        std::vector<CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
        if (!cuttingPlanes.empty()) {
            trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
        }
        
        // Show boundary (if detected)
        if (trajPlanModel_->isBoundaryDetected(*generalData_)) {
            trajPlanModel_->setBoundaryVisibility(*renderData_, true);
        }
        
        // Refresh display
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
        

    }
}

// Hide trajectory clicked
void SlicingWidgets::onHideTrajectoryClicked()
{
    if (trajPlanModel_ && renderData_) {
        // Hide trajectory lines
        trajPlanModel_->setTrajectoryVisibility(*renderData_, false);
        
        // Refresh display
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
        

    }
}

// Show trajectory clicked
void SlicingWidgets::onShowTrajectoryClicked()
{
    if (trajPlanModel_ && renderData_) {
        // Show trajectory lines
        trajPlanModel_->setTrajectoryVisibility(*renderData_, true);
        
        // Check if trajectory data exists
        std::vector<TrajectoryPoint> trajectoryPoints;
        trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
        if (!trajectoryPoints.empty()) {

            // Optionally add logic to focus camera on trajectory
        } else {

        }
        
        // Refresh display
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
        

    }
}

void SlicingWidgets::onHideSlicingPlaneClicked()
{
    if (trajPlanModel_ && renderData_) {
        trajPlanModel_->setCuttingPlanesVisibility(*renderData_, false);
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
    }
}

void SlicingWidgets::onShowSlicingPlaneClicked()
{
    if (trajPlanModel_ && renderData_ && generalData_) {
        std::vector<CuttingPlane> cuttingPlanes;
        trajPlanModel_->getCuttingPlanes(*generalData_, cuttingPlanes);
        if (!cuttingPlanes.empty()) {
            trajPlanModel_->setCuttingPlanesVisibility(*renderData_, true);
        }
        refreshDataDisplay();
        if (openGLView_) {
            openGLView_->update();
        }
        if (forceRefreshHook_) {
            forceRefreshHook_();
        }
    }
}



// X Translation Decrease button clicked
void SlicingWidgets::onXTranslationDecreaseClicked()
{
    if (ui->xTranslationSpinBox && ui->xStepSpinBox) {
        double currentValue = ui->xTranslationSpinBox->value();
        double stepValue = ui->xStepSpinBox->value();
        ui->xTranslationSpinBox->setValue(currentValue - stepValue);
    }
}

// X Translation Increase button clicked
void SlicingWidgets::onXTranslationIncreaseClicked()
{
    if (ui->xTranslationSpinBox && ui->xStepSpinBox) {
        double currentValue = ui->xTranslationSpinBox->value();
        double stepValue = ui->xStepSpinBox->value();
        ui->xTranslationSpinBox->setValue(currentValue + stepValue);
    }
}

// Y Translation Decrease button clicked
void SlicingWidgets::onYTranslationDecreaseClicked()
{
    if (ui->yTranslationSpinBox && ui->yStepSpinBox) {
        double currentValue = ui->yTranslationSpinBox->value();
        double stepValue = ui->yStepSpinBox->value();
        ui->yTranslationSpinBox->setValue(currentValue - stepValue);
    }
}

// Y Translation Increase button clicked
void SlicingWidgets::onYTranslationIncreaseClicked()
{
    if (ui->yTranslationSpinBox && ui->yStepSpinBox) {
        double currentValue = ui->yTranslationSpinBox->value();
        double stepValue = ui->yStepSpinBox->value();
        ui->yTranslationSpinBox->setValue(currentValue + stepValue);
    }
}

// Z Translation Decrease button clicked
void SlicingWidgets::onZTranslationDecreaseClicked()
{
    if (ui->zTranslationSpinBox && ui->zStepSpinBox) {
        double currentValue = ui->zTranslationSpinBox->value();
        double stepValue = ui->zStepSpinBox->value();
        ui->zTranslationSpinBox->setValue(currentValue - stepValue);
    }
}

// Z Translation Increase button clicked
void SlicingWidgets::onZTranslationIncreaseClicked()
{
    if (ui->zTranslationSpinBox && ui->zStepSpinBox) {
        double currentValue = ui->zTranslationSpinBox->value();
        double stepValue = ui->zStepSpinBox->value();
        ui->zTranslationSpinBox->setValue(currentValue + stepValue);
    }
}

// X Rotation Decrease button clicked
void SlicingWidgets::onXRotationDecreaseClicked()
{
    if (ui->xRotationSpinBox && ui->xRotStepSpinBox) {
        double currentValue = ui->xRotationSpinBox->value();
        double stepValue = ui->xRotStepSpinBox->value();
        ui->xRotationSpinBox->setValue(currentValue - stepValue);
    }
}

// X Rotation Increase button clicked
void SlicingWidgets::onXRotationIncreaseClicked()
{
    if (ui->xRotationSpinBox && ui->xRotStepSpinBox) {
        double currentValue = ui->xRotationSpinBox->value();
        double stepValue = ui->xRotStepSpinBox->value();
        ui->xRotationSpinBox->setValue(currentValue + stepValue);
    }
}

// Y Rotation Decrease button clicked
void SlicingWidgets::onYRotationDecreaseClicked()
{
    // Make sure Y rotation spin box and step spin box are available
    if (ui->yRotationSpinBox && ui->yRotStepSpinBox) {
        double currentValue = ui->yRotationSpinBox->value();  // Get current Y rotation
        double stepValue = ui->yRotStepSpinBox->value();      // Get Y rotation step value
        double newValue = currentValue - stepValue;
        
        qDebug() << QString("Y Rotation: current=%1, step=%2, new=%3")
            .arg(currentValue, 0, 'f', 2)
            .arg(stepValue, 0, 'f', 2)
            .arg(newValue, 0, 'f', 2);
        
        ui->yRotationSpinBox->setValue(newValue);  // new value = current - step
    }

}

// Y Rotation Increase button clicked
void SlicingWidgets::onYRotationIncreaseClicked()
{
    // Make sure Y rotation spin box and step spin box are available
    if (ui->yRotationSpinBox && ui->yRotStepSpinBox) {
        double currentValue = ui->yRotationSpinBox->value();  // Get current Y rotation
        double stepValue = ui->yRotStepSpinBox->value();      // Get Y rotation step value
        ui->yRotationSpinBox->setValue(currentValue + stepValue);  // new value = current + step
    }

}

// Z Rotation Decrease button clicked
void SlicingWidgets::onZRotationDecreaseClicked()
{
    if (ui->zRotationSpinBox && ui->zRotStepSpinBox) {
        double currentValue = ui->zRotationSpinBox->value();  // Get current Z rotation
        double stepValue = ui->zRotStepSpinBox->value();      // Get Z rotation step value
        ui->zRotationSpinBox->setValue(currentValue - stepValue);  // new value = current + step
    }
}

// Z Rotation Increase button clicked
void SlicingWidgets::onZRotationIncreaseClicked()
{
    if (ui->zRotationSpinBox && ui->zRotStepSpinBox) {
        double currentValue = ui->zRotationSpinBox->value();  // Get current Z rotation
        double stepValue = ui->zRotStepSpinBox->value();      // Get Z rotation step value
        ui->zRotationSpinBox->setValue(currentValue + stepValue);  // new value = current + step
    }
}

// Reset Transformation button clicked
void SlicingWidgets::onResetTransformClicked()
{
    // Reset all translation and rotation spin boxes to 0
    if (ui->xTranslationSpinBox) ui->xTranslationSpinBox->setValue(0.0);
    if (ui->yTranslationSpinBox) ui->yTranslationSpinBox->setValue(0.0);
    if (ui->zTranslationSpinBox) ui->zTranslationSpinBox->setValue(0.0);
    if (ui->xRotationSpinBox) ui->xRotationSpinBox->setValue(0.0);
    if (ui->yRotationSpinBox) ui->yRotationSpinBox->setValue(0.0);
    if (ui->zRotationSpinBox) ui->zRotationSpinBox->setValue(0.0);
    

    QMessageBox::information(this, "Reset Transformation", "All transformation parameters have been reset to 0.");
}


// ================== Trajectory Connection ==================
void SlicingWidgets::onConnectTrajectoryButtonClicked()
{
    if (!trajPlanModel_) {
        QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
        return;
    }

    // Check if trajectory points exist
    std::vector<TrajectoryPoint> trajPts;
    trajPlanModel_->getTrajectoryPoints(*generalData_, trajPts);
    if (trajPts.empty()) {
        QMessageBox::warning(this, "Warning", "No trajectory points. Please generate trajectory first.");
        return;
    }

    // Connect trajectory points using BowPath strategy
    if (!trajPlanModel_->connectTrajectoryPoints(*generalData_, ConnectionStrategy::BowPath)) {
        QMessageBox::critical(this, "Error", "Trajectory connection failed.");
        return;
    }

    syncConnectedTrajectoryToPublic(*generalData_, publicTrajData_);

    // Update rendering
    if (renderData_) {
        // 1. Initialize renderer for connection lines
        trajPlanModel_->initializeRenderer(*renderData_);
        
        // 2. Set connection lines visible
        trajPlanModel_->setConnectionVisibility(*renderData_, true);
        
    }

    // Refresh display and OpenGL view
    refreshDataDisplay();
    if (openGLView_) {
        openGLView_->update();
    }
    
    // Call custom refresh hook
    if (forceRefreshHook_) {
        forceRefreshHook_();
    }
    
    // Show success message
    QMessageBox::information(this, "Success", "Trajectory connection completed successfully!");

}


void SlicingWidgets::onResetUniformButtonClicked()
{
    int intervals = ui->slicesTableWidget->rowCount();
    if (intervals <= 0) {
        intervals = static_cast<int>(generalData_->slicingData_.cuttingPlanes_.size());
    }
    if (intervals <= 0) {
        intervals = 20;
    }
    
    // Reset table rows to match interval count
    ui->slicesTableWidget->setRowCount(intervals);
    
    for (int i = 0; i < intervals; ++i) {
        // Layer
        QTableWidgetItem* layerItem = new QTableWidgetItem(QString::number(i + 1));
        ui->slicesTableWidget->setItem(i, 0, layerItem);
        
        // Z position, distribute uniformly
        double z = (double)i / (intervals - 1) * 1123.4; // Assuming data range is [0, 1123.4]
        QTableWidgetItem* zItem = new QTableWidgetItem(QString::number(z, 'f', 2));
        ui->slicesTableWidget->setItem(i, 1, zItem);
    }
}

// �켣�Ż���ť����ۺ��� - ����SlicingWidgets��ĳ�Ա����
void SlicingWidgets::onTrajOpticlicked()
{
    //if (!trajPlanModel_) {
    //    QMessageBox::warning(this, "Warning", "No trajectory planning model available.");
    //    return;
    //}

    //// ����Ƿ��й켣����
    //std::vector<TrajectoryPoint> trajectoryPoints;
    //trajPlanModel_->getTrajectoryPoints(*generalData_, trajectoryPoints);
    //if (trajectoryPoints.empty()) {
    //    QMessageBox::warning(this, "Warning", "No trajectory points. Please generate trajectory first.");
    //    return;
    //}

    // ����TrajOptimizationDialog�Ի���ʵ������������ָ��
    TrajOptimizationDialog* dialog = new TrajOptimizationDialog(
        trajOptData_,      // ����TrajOptimizationDataָ��
        generalData_,      // ����GeneralDataָ��
        nullptr);

    // ���ؼ��޸�������ǿ��ˢ�»ص�������ʹOpenGL��ͼ�ܹ��Զ�����
    if (forceRefreshHook_) {
        dialog->setForceRefreshHook(forceRefreshHook_);
        qDebug() << "[SlicingWidgets] TrajOptimizationDialog: forceRefreshHook set successfully";
    } else {
        qWarning() << "[SlicingWidgets] TrajOptimizationDialog: forceRefreshHook is not set in SlicingWidgets!";
    }

    // ���öԻ���Ϊ��ģ̬
    dialog->setWindowModality(Qt::NonModal);
    dialog->setAttribute(Qt::WA_DeleteOnClose);  // �ر�ʱ�Զ�ɾ��
    dialog->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
                           Qt::WindowSystemMenuHint |
                           Qt::WindowMinimizeButtonHint |
                           Qt::WindowMaximizeButtonHint |
                           Qt::WindowCloseButtonHint);
    dialog->show();
}


}
