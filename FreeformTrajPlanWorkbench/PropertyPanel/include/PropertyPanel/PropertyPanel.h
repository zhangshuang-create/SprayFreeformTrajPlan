#pragma once

#include <QtWidgets/QWidget>
#include <QStackedWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QVector>
#include "PropertyPanel_global.h"

QT_BEGIN_NAMESPACE
namespace Ui { class PropertyPanel; };
QT_END_NAMESPACE

class PropertyPanel_SHAREDLIB PropertyPanel : public QWidget
{
    Q_OBJECT

public:
    PropertyPanel(QWidget *parent = nullptr);
    ~PropertyPanel();

    // Switch to a specific page by index
    void switchToPage(int index);

    // Page indices (correspond to Ribbon tabs)
    enum PageIndex {
        TrajectoryGenerationPage = 0,
        CollisionOptimizationPage = 1,
        PoseTransformationPage = 2,
        SimulationPage = 3,
        ThicknessPredictionPage = 4,
        ObjectPropertiesPage = 5
    };

    // Set the SlicingWidgets instance for trajectory generation page
    // Using QWidget* to avoid circular dependency
    void setSlicingWidget(QWidget* widget);
    double getTrajectoryOptimalSpeed() const;
    void showSlicingGenerationPanel();
    void showUvMappingGenerationPanel();
    void showMeshDecompositionGenerationPanel();
    double uvTargetSpacing() const;
    double uvTrajectorySpeed() const;
    double uvSprayDistance() const;
    double uvMaxSeamSprayDistance() const;
    double uvMaximumAcceleration() const;
    double uvMaximumJerk() const;
    double uvArcLockGuardD() const;
    double uvArcSpeedLossPct() const;
    void setUvControlsEnabled(bool enabled);
    void setUvTrajectoryGenerationEnabled(bool enabled);
    bool uvSplitViewEnabled() const;

    // Get current page index
    int getCurrentPage() const;

    // Set workspace active state
    void setWorkspaceActive(bool active);

    // Collision Optimization - Getters/Setters
    void setOptimizationAlgorithms(const QStringList& algorithms, int currentIndex = 0);
    int optimizationAlgorithmIndex() const;
    QString optimizationAlgorithmText() const;
    double safetyDistanceMm() const;
    int maxIterations() const;
    bool showCollisionPoints() const;
    bool showOriginalTrajectory() const;
    bool showOptimizedTrajectory() const;
    bool showOptimizationProcess() const;
    // Pose Transformation - Getters/Setters
    QVector<double> transformTranslationMm() const; // size 3: X, Y, Z
    QVector<double> transformRotationDeg() const;   // size 3: X, Y, Z
    QVector<double> transformMatrixValues() const;  // row-major size 16
    void setTransformTranslationMm(const QVector<double>& values);
    void setTransformRotationDeg(const QVector<double>& values);
    void setTransformMatrixValues(const QVector<double>& values);
    void resetTransformInputs();
    bool transformImportedModelChecked() const;
    bool transformImportedTrajectoryChecked() const;
    bool transformExtraModelChecked() const;

    // Matrix utility methods (returns 16-element matrix in row-major order)
    QVector<double> getTransformMatrixFlattened() const;
    void setTransformMatrixFlattened(const QVector<double>& matrix16);

    // Thickness Prediction - Getters
    int getThicknessDepositionFunctionIndex() const;
    double getThicknessSprayWidth() const;
    double getThicknessDepositionEfficiency() const;
    double getThicknessTargetThickness() const;
    double getThicknessQualityFactor() const;
    double getThicknessDefaultVelocity() const;
    bool showModel() const;
    bool showDepositionResult() const;
    bool showSprayGunPose() const;
    bool showTrajectoryPoints() const;
    bool showTrajectoryLine() const;
    bool pickThicknessOnHover() const;
    void setShowModel(bool visible);

signals:
    // Emitted when page changes
    void pageChanged(int index);

    // Trajectory Generation page signals
    void openSlicingControlDialogRequested();
    void uvSplitViewChanged(bool enabled);
    void uvTrajectoryGenerationRequested();

    void optimizationAlgorithmChanged(int index);

    // Pose Transformation page signals
    void transformTranslationRotationChanged();
    void transformMatrixEdited();

    // Simulation page signals
    void browseTrajectoryRequested();
    void frameSliderChanged(int frame);
    void playbackSpeedChanged(double speed);
    void modelVisibilityChanged(bool visible);
    void depositionVisibilityChanged(bool visible);
    void sprayGunPoseVisibilityChanged(bool visible);
    void trajectoryPointsVisibilityChanged(bool visible);
    void trajectoryLineVisibilityChanged(bool visible);
    void thicknessPickingChanged(bool enabled);

    // View/output signals
    void viewUpdateRequested();
    void projectDirtyRequested();
    void statusMessageRequested(const QString& message, int timeout = 3000);
    void outputRequested(const QString& message);
    void logRequested(const QString& message);

private:
    Ui::PropertyPanel *ui;
    QWidget* m_slicingWidget = nullptr;
    QGroupBox* m_controlGroupBox = nullptr;  // Reference to SlicingWidgets' controlGroupBox
    QStackedWidget* m_generationMethodStack = nullptr;
    QWidget* m_slicingGenerationPage = nullptr;
    QWidget* m_uvMappingGenerationPage = nullptr;
    QWidget* m_meshDecompositionGenerationPage = nullptr;
    QPushButton* m_openSlicingDialogButton = nullptr;  // Open Slicing Control Dialog button
    QGroupBox* m_uvParameterGroup = nullptr;
    QCheckBox* m_uvSplitViewCheckBox = nullptr;
    QPushButton* m_uvGenerateTrajectoryButton = nullptr;
    QVector<QDoubleSpinBox*> m_uvSpinBoxes;
    bool m_hasActiveWorkspace = false;  // Track workspace state
    bool m_syncingTransformInputs = false;  // Prevent circular updates in pose transform

    void setupConnections();
    void ensureGenerationMethodStack();
    void createUvMappingParameterPanel();
    void syncPoseMatrixFromTranslationRotation();
    void syncPoseTranslationRotationFromMatrix();

    // Matrix conversion utilities (internal use)
    static QVector<double> matrixFromTranslationRotation(
        double tx, double ty, double tz,
        double rxDeg, double ryDeg, double rzDeg);
    static void extractTranslationRotation(
        const QVector<double>& matrix16,
        double& tx, double& ty, double& tz,
        double& rxDeg, double& ryDeg, double& rzDeg);
};
