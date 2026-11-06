#include "FreeformTrajPlanWorkbenchContribution.h"
#include "RobotQtViewerDocumentViewRegistry.h"

#include "FreeformTrajPlanWorkbenchController.h"
#include "FreeformTrajPlanWorkbenchPanel.h"

#include <QScrollArea>
#include <QScrollBar>

namespace robot_qt_viewer
{
    namespace {
        class FreeformPanelScrollArea final : public QScrollArea {
        public:
            using QScrollArea::QScrollArea;
            QSize sizeHint() const override {
                if(!widget()) return QScrollArea::sizeHint();
                return QSize(widget()->minimumSizeHint().width() +
                    verticalScrollBar()->sizeHint().width() + 2 * frameWidth(),
                    widget()->sizeHint().height());
            }
        };
        class FreeformWorkbenchLifecycle final : public IRobotQtViewerWorkbenchLifecycle {
        public:
            explicit FreeformWorkbenchLifecycle(smrobot::workbench::freeform::FreeformTrajPlanWorkbenchController& controller)
                : m_controller(controller) {}
            RobotQtViewerWorkbenchTransitionResult prepareDeactivate(const RobotQtViewerWorkbenchTransitionContext&) override {
                return workbenchTransitionSucceeded();
            }
            RobotQtViewerWorkbenchTransitionResult deactivate(const RobotQtViewerWorkbenchTransitionContext& context) override {
                QString error;
                const bool restore = context.cause != RobotQtViewerWorkbenchTransitionCause::ProjectReplacing &&
                    context.cause != RobotQtViewerWorkbenchTransitionCause::ApplicationShutdown;
                return m_controller.deactivateWorkbench(restore, error) ? workbenchTransitionSucceeded() : workbenchTransitionFailed(error);
            }
            RobotQtViewerWorkbenchTransitionResult activate(const RobotQtViewerWorkbenchActivationContext&) override {
                QString error;
                return m_controller.activateWorkbench(error) ? workbenchTransitionSucceeded() : workbenchTransitionFailed(error);
            }
            void releaseProject(const RobotQtViewerWorkbenchProjectReleaseContext&) noexcept override { cleanup(); }
            void shutdown(const RobotQtViewerWorkbenchShutdownContext&) noexcept override { cleanup(); }
        private:
            void cleanup() noexcept { try { QString error; m_controller.deactivateWorkbench(false, error); } catch(...) {} }
            smrobot::workbench::freeform::FreeformTrajPlanWorkbenchController& m_controller;
        };
    }
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
                auto* workflowScrollArea = new FreeformPanelScrollArea();
                workflowScrollArea->setWidgetResizable(true);
                workflowScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
                workflowScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

                auto* workflowPanel = new smrobot::workbench::freeform::FreeformTrajPlanWorkflowPanel();
                workflowScrollArea->setWidget(workflowPanel);
                auto* parameterScrollArea = new FreeformPanelScrollArea(panelParent);
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
                        std::make_unique<FreeformWorkbenchLifecycle>(*controller),
                        RobotQtViewerWorkbenchLifecyclePolicy{
                            RobotQtViewerWorkbenchExecutionPolicy::NoOwnedExecution,
                            RobotQtViewerWorkbenchReactivationPolicy::PackageDefined
                        },
                        std::make_unique<RobotQtViewerNoOpLanguageParticipant>()));
            }
        };
    }
}
