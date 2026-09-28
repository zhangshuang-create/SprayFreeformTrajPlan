#include <ProcessOptimizationDialog/ProcessOptimizationDialog.h>
#include "ui_ProcessOptimizationDialog.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <data_path.h>
namespace ModelTrajPlan
{
    ProcessOptimizationDialog::ProcessOptimizationDialog(QWidget* parent)
        : QDialog(parent)
        , ui(new Ui::ProcessOptimizationDialogClass())
        , trajOptData_(nullptr)
        , generalData_(nullptr)
    {
        ui->setupUi(this);
    }

    ProcessOptimizationDialog::ProcessOptimizationDialog(TrajOptimizationData* trajOptData, GeneralData* generalData, QWidget* parent)
        : QDialog(parent)
        , ui(new Ui::ProcessOptimizationDialogClass())
        , trajOptData_(trajOptData)
        , generalData_(generalData)
    {
        ui->setupUi(this);
        if (trajOptData_) {
            initializeSystem();
        }
    }

    ProcessOptimizationDialog::~ProcessOptimizationDialog()
    {
        delete ui;
    }

    void ProcessOptimizationDialog::setTrajOptData(TrajOptimizationData* trajOptData)
    {
        trajOptData_ = trajOptData;
        if (trajOptData_) {
            initializeSystem();
        }
    }

    void ProcessOptimizationDialog::setGeneralData(GeneralData* generalData)
    {
        generalData_ = generalData;
    }

    void ProcessOptimizationDialog::initializeSystem()
    {
        // Initialize transform matrix UI to identity matrix
        updateTransformMatrixDisplay();

        qDebug() << "[ProcessOptDialog] System initialized.";
    }

    void ProcessOptimizationDialog::updateTransformMatrixDisplay()
    {
        if (!trajOptData_) return;

        Eigen::Matrix4d matrix = trajOptData_->transformMatrix_;

        // Update matrix display in UI
        ui->m00SpinBox->setValue(matrix(0, 0));
        ui->m01SpinBox->setValue(matrix(0, 1));
        ui->m02SpinBox->setValue(matrix(0, 2));
        ui->m03SpinBox->setValue(matrix(0, 3));

        ui->m10SpinBox->setValue(matrix(1, 0));
        ui->m11SpinBox->setValue(matrix(1, 1));
        ui->m12SpinBox->setValue(matrix(1, 2));
        ui->m13SpinBox->setValue(matrix(1, 3));

        ui->m20SpinBox->setValue(matrix(2, 0));
        ui->m21SpinBox->setValue(matrix(2, 1));
        ui->m22SpinBox->setValue(matrix(2, 2));
        ui->m23SpinBox->setValue(matrix(2, 3));

        ui->m30SpinBox->setValue(matrix(3, 0));
        ui->m31SpinBox->setValue(matrix(3, 1));
        ui->m32SpinBox->setValue(matrix(3, 2));
        ui->m33SpinBox->setValue(matrix(3, 3));
    }

    // Original model related slot functions
    void ProcessOptimizationDialog::onBrowseOriginalModelClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/model");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Original Model File"),
            defaultPath,
            tr("Model Files (*.stl *.obj *.xml)"));

        if (!fileName.isEmpty()) {
            ui->originalModelFileLineEdit->setText(fileName);
            ui->loadOriginalModelButton->setEnabled(true);
        }
    }

    void ProcessOptimizationDialog::onLoadOriginalModelClicked()
    {
        QString fileName = ui->originalModelFileLineEdit->text();
        if (!fileName.isEmpty()) {
            // TODO: Implement loading original model logic
            QMessageBox::information(this, tr("Information"), tr("Original model loading feature to be implemented"));
        }
    }

    // Actual model related slot functions
    void ProcessOptimizationDialog::onBrowseActualModelClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/model");

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Select Actual Model File"),
            defaultPath,
            tr("Model Files (*.stl *.obj *.pcd)"));

        if (!fileName.isEmpty()) {
            ui->actualModelFileLineEdit->setText(fileName);
            ui->loadActualModelButton->setEnabled(true);
        }
    }

    void ProcessOptimizationDialog::onLoadActualModelClicked()
    {
        QString fileName = ui->actualModelFileLineEdit->text();
        if (!fileName.isEmpty()) {
            // TODO: Implement loading actual model logic
            QMessageBox::information(this, tr("Information"), tr("Actual model loading feature to be implemented"));
        }
    }

    // Transform matrix related slot functions
    void ProcessOptimizationDialog::onAutoCalculateToggled(bool checked)
    {
        ui->manualTransformGroup->setEnabled(!checked);
        ui->calculateTransformButton->setEnabled(checked);
    }

    void ProcessOptimizationDialog::onManualInputToggled(bool checked)
    {
        ui->manualTransformGroup->setEnabled(checked);
        ui->calculateTransformButton->setEnabled(!checked);
    }

    void ProcessOptimizationDialog::onCalculateTransformClicked()
    {
        // TODO: Implement automatic transform matrix calculation logic
        QMessageBox::information(this, tr("Information"), tr("Auto-calculate transform matrix feature to be implemented"));
    }

    void ProcessOptimizationDialog::onResetTransformClicked()
    {
        // Reset to identity matrix
        ui->m00SpinBox->setValue(1.0);
        ui->m01SpinBox->setValue(0.0);
        ui->m02SpinBox->setValue(0.0);
        ui->m03SpinBox->setValue(0.0);

        ui->m10SpinBox->setValue(0.0);
        ui->m11SpinBox->setValue(1.0);
        ui->m12SpinBox->setValue(0.0);
        ui->m13SpinBox->setValue(0.0);

        ui->m20SpinBox->setValue(0.0);
        ui->m21SpinBox->setValue(0.0);
        ui->m22SpinBox->setValue(1.0);
        ui->m23SpinBox->setValue(0.0);

        ui->m30SpinBox->setValue(0.0);
        ui->m31SpinBox->setValue(0.0);
        ui->m32SpinBox->setValue(0.0);
        ui->m33SpinBox->setValue(1.0);

        ui->transformMatrixTextEdit->setText(tr("Identity matrix set"));
    }

    void ProcessOptimizationDialog::onLoadTransformFromFileClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path);

        QString fileName = QFileDialog::getOpenFileName(this,
            tr("Load Transform Matrix"),
            defaultPath,
            tr("Matrix Files (*.txt *.csv)"));

        if (!fileName.isEmpty()) {
            // TODO: Implement loading transform matrix from file logic
            QMessageBox::information(this, tr("Information"), tr("Load transform matrix from file feature to be implemented"));
        }
    }

    void ProcessOptimizationDialog::onSaveTransformToFileClicked()
    {
        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path);

        QString fileName = QFileDialog::getSaveFileName(this,
            tr("Save Transform Matrix"),
            defaultPath,
            tr("Matrix Files (*.txt *.csv)"));

        if (!fileName.isEmpty()) {
            // TODO: Implement saving transform matrix to file logic
            QMessageBox::information(this, tr("Information"), tr("Save transform matrix to file feature to be implemented"));
        }
    }

    void ProcessOptimizationDialog::onApplyManualTransformClicked()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData not initialized!"));
            return;
        }

        // Read transform matrix from UI
        Eigen::Matrix4d matrix;
        matrix(0, 0) = ui->m00SpinBox->value(); matrix(0, 1) = ui->m01SpinBox->value();
        matrix(0, 2) = ui->m02SpinBox->value(); matrix(0, 3) = ui->m03SpinBox->value();
        matrix(1, 0) = ui->m10SpinBox->value(); matrix(1, 1) = ui->m11SpinBox->value();
        matrix(1, 2) = ui->m12SpinBox->value(); matrix(1, 3) = ui->m13SpinBox->value();
        matrix(2, 0) = ui->m20SpinBox->value(); matrix(2, 1) = ui->m21SpinBox->value();
        matrix(2, 2) = ui->m22SpinBox->value(); matrix(2, 3) = ui->m23SpinBox->value();
        matrix(3, 0) = ui->m30SpinBox->value(); matrix(3, 1) = ui->m31SpinBox->value();
        matrix(3, 2) = ui->m32SpinBox->value(); matrix(3, 3) = ui->m33SpinBox->value();

        TrajOptimizationAlgorithm::setTransformMatrix(*trajOptData_, matrix);

        QMessageBox::information(this, tr("Success"), tr("Transform matrix applied successfully!"));
    }

    // Trajectory transformation and generation slot functions
    void ProcessOptimizationDialog::onTransformTrajectoryClicked()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData not initialized!"));
            return;
        }

        // Read transform matrix from UI
        Eigen::Matrix4d matrix;
        matrix(0, 0) = ui->m00SpinBox->value(); matrix(0, 1) = ui->m01SpinBox->value();
        matrix(0, 2) = ui->m02SpinBox->value(); matrix(0, 3) = ui->m03SpinBox->value();
        matrix(1, 0) = ui->m10SpinBox->value(); matrix(1, 1) = ui->m11SpinBox->value();
        matrix(1, 2) = ui->m12SpinBox->value(); matrix(1, 3) = ui->m13SpinBox->value();
        matrix(2, 0) = ui->m20SpinBox->value(); matrix(2, 1) = ui->m21SpinBox->value();
        matrix(2, 2) = ui->m22SpinBox->value(); matrix(2, 3) = ui->m23SpinBox->value();
        matrix(3, 0) = ui->m30SpinBox->value(); matrix(3, 1) = ui->m31SpinBox->value();
        matrix(3, 2) = ui->m32SpinBox->value(); matrix(3, 3) = ui->m33SpinBox->value();

        TrajOptimizationAlgorithm::setTransformMatrix(*trajOptData_, matrix);

        if (TrajOptimizationAlgorithm::transformTrajectory(*trajOptData_)) {
            QMessageBox::information(this, tr("Success"), tr("Trajectory transformed successfully!"));
        }
        else {
            QMessageBox::warning(this, tr("Error"), tr("Failed to transform trajectory!"));
        }
    }

    void ProcessOptimizationDialog::onPreviewTransformClicked()
    {
        // TODO: Implement preview transform effect logic
        QMessageBox::information(this, tr("Information"), tr("Preview transform effect feature to be implemented"));
    }

    void ProcessOptimizationDialog::onGenerateTrajectoryClicked()
    {
        // TODO: Implement generating transformed trajectory logic
        QMessageBox::information(this, tr("Information"), tr("Generate transformed trajectory feature to be implemented"));
    }

    void ProcessOptimizationDialog::onExportGeneratedTrajectoryClicked()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData not initialized!"));
            return;
        }

        std::string data_path = DATA_PATH;
        QString defaultPath = QString::fromStdString(data_path + "/traj");

        QString fileName = QFileDialog::getSaveFileName(this,
            tr("Export Generated Trajectory"),
            defaultPath,
            tr("Trajectory Files (*.txt *.csv)"));

        if (!fileName.isEmpty()) {
            if (TrajOptimizationAlgorithm::exportTolerancedTrajectory(*trajOptData_, fileName.toStdString())) {
                QMessageBox::information(this, tr("Success"), tr("Trajectory exported successfully!"));
            }
            else {
                QMessageBox::warning(this, tr("Error"), tr("Failed to export trajectory!"));
            }
        }
    }

    void ProcessOptimizationDialog::onVisualizeTrajectoryClicked()
    {
        // TODO: Implement visualizing trajectory logic
        QMessageBox::information(this, tr("Information"), tr("Visualize trajectory feature to be implemented"));
    }

    // Process optimization slot functions
    void ProcessOptimizationDialog::onStartProcessOptimizationClicked()
    {
        if (!trajOptData_) {
            QMessageBox::warning(this, tr("Error"), tr("TrajOptimizationData not initialized!"));
            return;
        }

        // TODO: Implement process optimization logic
        QMessageBox::information(this, tr("Information"), tr("Process optimization feature to be implemented"));
    }

    void ProcessOptimizationDialog::onStopProcessOptimizationClicked()
    {
        // TODO: Implement stopping process optimization logic
        QMessageBox::information(this, tr("Information"), tr("Stop process optimization feature to be implemented"));
    }

}