#include "FreeformTrajPlanWorkbenchContribution.h"
#include "RobotQtViewerDocumentViewRegistry.h"

#include "FreeformTrajPlanWorkbenchController.h"
#include "FreeformTrajPlanWorkbenchPanel.h"

#include <QScrollArea>

namespace robot_qt_viewer
{
    bool registerFreeformTrajPlanWorkbenchContribution(RobotQtViewerWorkbenchPackageRegistry& catalog)
    {
        const QString packageId = QStringLiteral("smrobot.workbench.freeform-trajectory-planning");
        RobotQtViewerWorkbenchDescriptor descriptor;
        descriptor.kind = RobotQtViewerWorkbenchKind::Browse;
        descriptor.domain = RobotQtViewerWorkbenchDomain::SprayProcess;
        descriptor.rightPanel = RobotQtViewerRightPanelKind::Status;
        descriptor.defaultViewportMode = RobotQtViewerViewportInteractionMode::Browse;
        descriptor.id = QString::fromLatin1(kFreeformTrajPlanWorkbenchId);
        descriptor.displayName = QStringLiteral("曲面轨迹规划");
        descriptor.rightPanelTitle = QStringLiteral("曲面轨迹规划");
        descriptor.projectAssemblyTreeProjection = false;
        descriptor.projectAssemblyActions = false;
        return catalog.registerPackage(makeRobotQtViewerWorkbenchPackage(packageId,
                   QStringLiteral("Freeform Trajectory Planning"))) &&
            catalog.registerWorkbench(makeRobotQtViewerWorkbench(packageId, descriptor,
                QStringLiteral("freeformTrajPlanWorkbench"), 67,
                { QStringLiteral("smrobot.feature.freeform-trajectory-planning") },
                { robotQtViewerWorkbenchId(RobotQtViewerWorkbenchKind::Browse) }));
    }
    RobotQtViewerWorkbenchRuntimeContributionFactoryDesc
    makeOwnedFreeformTrajPlanWorkbenchRuntimeContributionFactory(
        FreeformTrajPlanWorkbenchComposition&& composition)
    {
        const QString workbenchId = QString::fromLatin1(kFreeformTrajPlanWorkbenchId);
        return {
            workbenchId,
            [workbenchId, composition = std::move(composition)](QWidget* panelParent) mutable {
                if(panelParent == nullptr || composition.documentContext == nullptr ||
                    !composition.installLeftPanel) {
                    return std::unique_ptr<RobotQtViewerWorkbenchRuntimeContribution>();
                }
                auto* workflowScrollArea = new QScrollArea();
                workflowScrollArea->setWidgetResizable(true);
                workflowScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
                workflowScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

                auto* workflowPanel = new smrobot::workbench::freeform::FreeformTrajPlanWorkflowPanel();
                workflowScrollArea->setWidget(workflowPanel);
                auto* parameterScrollArea = new QScrollArea(panelParent);
                parameterScrollArea->setWidgetResizable(true);
                parameterScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
                parameterScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

                auto* parameterPanel = new smrobot::workbench::freeform::FreeformTrajPlanParameterPanel();
                parameterScrollArea->setWidget(parameterPanel);

                auto* controller = new smrobot::workbench::freeform::FreeformTrajPlanWorkbenchController(
                    *workflowPanel,
                    *parameterPanel,
                    *composition.documentContext,
                    composition.dataSource,
                    {},
                    {},
                    parameterPanel);

                QObject::connect(
                    controller,
                    &smrobot::workbench::freeform::FreeformTrajPlanWorkbenchController::statusMessageRequested,
                    parameterPanel,
                    [showStatus = composition.showStatus](const QString& message, int timeoutMs) {
                        if(showStatus) {
                            showStatus(message, timeoutMs);
                        }
                    });

                if(composition.documentViewRegistry) {
                    composition.documentViewRegistry->registerModule(QStringLiteral("freeformPlanning"),controller,
                        [controller](const RobotQtViewerEvent& event){controller->handleEvent(event);},
                        {
                            RobotQtViewerEventKind::ProjectOpened,
                            RobotQtViewerEventKind::ProjectDocumentChanged,
                            RobotQtViewerEventKind::ViewportReloaded,
                            RobotQtViewerEventKind::SelectionChanged
                        });
                }
                composition.installLeftPanel(workflowScrollArea);

                return std::unique_ptr<RobotQtViewerWorkbenchRuntimeContribution>(
                    std::make_unique<RobotQtViewerBasicWorkbenchRuntimeContribution>(
                        workbenchId,
                        [parameterScrollArea]() { return parameterScrollArea; },
                        std::make_unique<RobotQtViewerNoOpWorkbenchLifecycle>(),
                        RobotQtViewerWorkbenchLifecyclePolicy{
                            RobotQtViewerWorkbenchExecutionPolicy::NoOwnedExecution,
                            RobotQtViewerWorkbenchReactivationPolicy::PackageDefined
                        },
                        std::make_unique<RobotQtViewerNoOpLanguageParticipant>()));
            }
        };
    }
}
