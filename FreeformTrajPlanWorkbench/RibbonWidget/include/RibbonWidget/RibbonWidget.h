#pragma once

#include <QtWidgets/QWidget>
#include <QToolButton>
#include <QGroupBox>
#include <QTabWidget>
#include <QOpenGLWidget>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <vector>
#include <QPointer>
#include "RibbonWidget_global.h"
#include <BurTrajPlan/IBurTrajPlan.h>
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>
#include <PublicDataRender/PublicDataRender.h>
#include <ThicknessData/ThicknessData.h>
#include <ThicknessAnalyzer/ThicknessAnalyzer.h>
#include <TrajOptimization/TrajectoryPlaybackController.h>
#include <TrajModule/CommonDataTypes.h>
namespace SprayTrajPlan { struct TrajectoryParams; class TrajPlanPipeline; class UvPlanningSession; }
class QProgressDialog;
QT_BEGIN_NAMESPACE
namespace Ui { class RibbonWidget; };
QT_END_NAMESPACE

class RibbonWidget_SHAREDLIB RibbonWidget : public QWidget
{
    Q_OBJECT

public:
    RibbonWidget(QWidget *parent = nullptr);
    ~RibbonWidget();

    // Get current active tab index
    int getCurrentTabIndex() const;
    void setThicknessAnalyzer(ModelTrajPlan::ThicknessAnalyzer_spointer analyzer);
    void setThicknessData(ModelTrajPlan::ThicknessData* thicknessData);
    // Set current tab
    void setCurrentTab(int index);

    // Set data pointers for trajectory operations
    void setTrajPlanModel(ModelTrajPlan::IBurTrajPlan_spointer model);
    void setGeneralData(ModelTrajPlan::GeneralData* generalData);
    void setRenderData(ModelTrajPlan::RenderData* renderData);
    void setTrajOptimizationData(ModelTrajPlan::TrajOptimizationData* trajOptimizationData);
    void setPublicModelData(PublicData::PublicModelData* publicModelData);
    void setPublicTrajData(PublicData::TrajData* publicTrajData);
    void setPublicDataRender(PublicData::PublicDataRender* publicDataRender);
    void setOpenGLView(QOpenGLWidget* openGLView);
    void setForceRefreshHook(const std::function<void()>& cb);

    // Set PropertyPanel reference (for reading thickness parameters)
    void setPropertyPanel(QWidget* propertyPanel);

    // Set workspace active state (for import model validation)
    void setWorkspaceActive(bool active);
    void generateUvTrajectory();
    const SprayTrajPlan::ObjUVData* uvMapData() const;
    const std::vector<SprayTrajPlan::TrajectorySegment>& uvTrajectory() const;

    // Public command entry points used by external UI triggers such as menus and PropertyPanel.
    void importModel();
    void importTrajectory();
    void importRobot();
    void importTool();
    void clearModel();
    void clearTrajectory();
    void clearConnection();
    void importCollisionWorkpiece();
    void importCollisionModel();
    void runCollisionOptimization();
    void clearCollisionWorkpiece();
    void clearCollisionRobot();
    void clearCollisionModel();
    void clearOptimizationResult();
    void importTransformJson();
    void exportTransformJson();
    void loadExtraTransformModel();
    void applyTransform();
    void resetTransform();
    void importDepositionModel();
    void runThicknessSimulation();
    void clearThicknessResult();
    void clearDepositionModel();
    void clearThicknessInputs();

    // Trajectory generation Export operations
    void exportTrajectory();
    void exportABB();
    void exportFANUC();
    void exportExcel();

    // Collision optimization operations
    void importCollisionTrajectory();
    void exportOptimizedTrajectory();
    void exportCollisionReport();

    // Pose transformation operations
    void importTransformTrajectory();
    void exportTransformedModel();
    void exportTransformedTrajectory();
    void clearExtraModel();
    void clearTransformResult();
    void clearMatrix();

    // Thickness prediction operations
    void importThicknessWorkpiece();
    void importPointCloud();
    void importThicknessTrajectory();
    void exportThicknessResult();
    void exportHeatmap();
    void exportThicknessReport();

signals:
    // Emitted when tab changes
    void tabChanged(int index);

    // Trajectory generation tab signals
    void trajectoryMethodClicked();
    void slicingMethodClicked();
    void generateTrajectoryClicked();

    // Trajectory generation method selection signals
    void slicingGenerationSelected();
    void uvMappingGenerationSelected();
    void meshDecompositionGenerationSelected();

    // Slicing method selection signals
    void aaSlicingSelected();
    void parallelSlicingSelected();
    void adaptiveSlicingSelected();
    void contourSlicingSelected();

    // Slicing operation signals (Confirm, Generate, Connect, Cancel)
    void confirmSlicingClicked();
    void connectTrajectoryClicked();
    void cancelSlicingClicked();

    // Trajectory generation clear signals
    void clearModelClicked();
    void clearTrajectoryClicked();
    void clearConnectionClicked();

    // Collision optimization tab signals
    void importWorkpieceClicked();
    void importRobotClicked();
    void importCollisionModelClicked();
    void selectOptimizationAlgorithmClicked();
    void runOptimizationClicked();
    void clearOptimizationResultClicked();

    // Collision optimization clear signals
    void clearCollisionWorkpieceClicked();
    void clearCollisionRobotClicked();
    void clearCollisionModelClicked();

    // Pose transformation tab signals
    void inputMatrixClicked();
    void importMatrixJsonClicked();
    void loadExtraModelClicked();
    void transformModelClicked();
    void transformTrajectoryClicked();
    void applyTransformClicked();
    void exportTransformSettingsClicked();
    void resetTransformClicked();

    // Simulation tab signals
    void importTrajectoryFileClicked();
    void useExistingTrajectoryClicked();
    void playClicked();
    void pauseClicked();
    void stopClicked();
    void resetClicked();
    void playbackSpeedClicked();

    // Thickness prediction tab signals
    void importDepositionModelClicked();
    void startThicknessSimulationClicked();
    void clearThicknessResultClicked();

    // Thickness prediction clear signals
    void clearDepositionModelClicked();
    void clearThicknessInputsClicked();

    // Trajectory generation Export signals
    void exportTrajectoryClicked();
    void exportABBClicked();
    void exportFANUCClicked();
    void exportExcelClicked();
    void uvStartMapRequested();
    void uvStartTrajectoryRequested();
    void uvStartOptimizeRequested();
    void uvOptimizationProgress(int percent);
    void uvOptimizationTelemetry(const QString& message);
    void uvExportRequested();

    // Collision optimization Import/Export signals
    void importCollisionTrajectoryClicked();
    void exportOptimizedTrajectoryClicked();
    void exportCollisionReportClicked();

    // Collision optimization Method signals
    void collisionMethodClicked();
    void collisionOptimizationMethodSelected();

    // Pose transformation Import/Export signals
    void importTransformTrajectoryClicked();
    void exportTransformedModelClicked();
    void exportTransformedTrajectoryClicked();
    void clearExtraModelClicked();
    void clearTransformResultClicked();
    void clearMatrixClicked();

    // Thickness prediction Import/Export signals
    void importThicknessWorkpieceClicked();
    void importPointCloudClicked();
    void importThicknessTrajectoryClicked();
    void exportThicknessResultClicked();
    void exportHeatmapClicked();
    void exportThicknessReportClicked();

    // Thickness prediction Method signals
    void thicknessMethodClicked();
    void gaussianModelSelected();
    void betaModelSelected();
    void customModelSelected();
    void zigongModelSelected();
    void model420Selected();

private slots:
    // Actual implementation slots for trajectory operations
    void onGenerateTrajectoryClicked();
    void onConnectTrajectoryClicked();
    void onConfirmSlicingClicked();
    void onCancelSlicingClicked();

    // Thickness prediction operation implementations
    void onStartThicknessSimulationClicked();
    void onClearThicknessResultClicked();

    // Import model operation implementation
    void onImportModelClicked();

    // Import/Export operations implementations
    void onImportTrajectoryClicked();
    void onImportRobotClicked();
    void onClearModelClicked();
    void onClearTrajectoryClicked();
    void onClearConnectionClicked();

    // Collision optimization operations
    void onImportWorkpieceClicked();
    void onImportCollisionModelClicked();
    void onRunCollisionOptimizationClicked();
    void onClearCollisionModelClicked();

    // Transform operations
    void onImportTransformJsonClicked();
    void onExportTransformJsonClicked();
    void onApplyTransformClicked();
    void onResetTransformClicked();

    // Trajectory generation Export implementations
    void onExportTrajectoryClicked();

    // Collision optimization implementations
    void onImportCollisionTrajectoryClicked();
    void onExportOptimizedTrajectoryClicked();
    void onExportCollisionReportClicked();

    // Pose transformation implementations
    void onImportTransformTrajectoryClicked();
    void onExportTransformedModelClicked();
    void onExportTransformedTrajectoryClicked();
    void onClearExtraModelClicked();
    void onClearTransformResultClicked();
    void onClearMatrixClicked();

    // Thickness prediction implementations
    void onImportThicknessWorkpieceClicked();
    void onImportPointCloudClicked();
    void onImportThicknessTrajectoryClicked();
    void onExportThicknessResultClicked();
    void onExportHeatmapClicked();
    void onExportThicknessReportClicked();

private:
    Ui::RibbonWidget *ui;
    ModelTrajPlan::ThicknessAnalyzer_spointer thicknessAnalyzer_;
    ModelTrajPlan::ThicknessData* thicknessData_ = nullptr;
    void setupConnections();
    void setupMenus();
    void setupClearMenus();
    void configureRibbonSizing();
    void setGenerationToolPanel(const QString& title, bool showSlicingControls);
    void createUvToolPanel();
    void updateUvCommandState();
    SprayTrajPlan::TrajectoryParams uvParameters() const;
    bool displayUvSourceModel();
    bool renderUvTrajectory();
    bool renderUvTrajectory(const std::vector<SprayTrajPlan::TrajectorySegment>& trajectory);
    void onStartUvMapClicked();
    void onStartUvTrajectoryClicked();
    void onStartUvOptimizeClicked();
    void onExportUvTrajectoryClicked();

    // Helper method to refresh display
    void refreshDataDisplay();

    // Current method tracking
    QString m_currentGenerationMethod;
    QString m_currentSlicingMethod;
    QString m_currentCollisionMethod;
    QString m_currentThicknessMethod;
    QString m_currentTransformMethod;
    QGroupBox* m_uvToolGroup = nullptr;
    QToolButton* m_startUvMapButton = nullptr;
    QToolButton* m_startUvTrajectoryButton = nullptr;
    QToolButton* m_startUvOptimizeButton = nullptr;
    QToolButton* m_exportUvButton = nullptr;
    std::unique_ptr<SprayTrajPlan::TrajPlanPipeline> m_uvPipeline;
    std::unique_ptr<SprayTrajPlan::UvPlanningSession> m_uvSession;
    std::vector<SprayTrajPlan::TrajectorySegment> m_uvPreviewTrajectory;
    std::thread m_uvOptimizationThread;
    std::atomic_bool m_uvOptimizationRunning{false};
    QPointer<QProgressDialog> m_uvOptimizationProgress;

    // Data pointers for trajectory planning operations
    ModelTrajPlan::IBurTrajPlan_spointer trajPlanModel_;
    ModelTrajPlan::GeneralData* generalData_ = nullptr;
    ModelTrajPlan::RenderData* renderData_ = nullptr;
    ModelTrajPlan::TrajOptimizationData* trajOptimizationData_ = nullptr;
    PublicData::PublicModelData* publicModelData_ = nullptr;
    PublicData::TrajData* publicTrajData_ = nullptr;
    PublicData::PublicDataRender* publicDataRender_ = nullptr;
    QOpenGLWidget* openGLView_ = nullptr;
    QWidget* thicknessOverlay_ = nullptr;
    std::function<void()> forceRefreshHook_;
    PublicData::TrajData transformedTrajectoryData_;
    PublicData::PublicModel originalTransformModel_;
    PublicData::TrajectoryPoints originalTransformTrajectory_;
    bool hasOriginalTransformModel_ = false;
    bool hasOriginalTransformTrajectory_ = false;

    void restoreTransformSources();
    void redrawPublicTransformData(bool redrawModel, bool redrawTrajectory);
    void setPrivateTransformSourcesVisible(bool modelVisible, bool trajectoryVisible);

    // PropertyPanel reference (for reading parameters)
    QWidget* propertyPanel_ = nullptr;

    // Workspace active state
    bool m_hasActiveWorkspace = false;

    // Collision optimization state
    QString m_collisionModelPath;
    QString m_collisionWorkpiecePath;
    std::vector<std::string> collisionRobotPaths_;
    ModelTrajPlan::TrajectoryPlaybackController collisionPlaybackController_;
};
