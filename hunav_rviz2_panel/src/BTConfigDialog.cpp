#include "headers/BTConfigDialog.h"
#include "headers/actor_panel.hpp"
#include <QWizardPage>
#include <QLabel>
#include <QTimer>
#include <QApplication>
#include <tinyxml2.h>
#include <behaviortree_cpp/bt_factory.h>
#include <QMessageBox>
#include <QScrollArea>
#include <QFile>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QSplitter>
#include <QHeaderView>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QPalette>
#include <QEvent>
#include <QMouseEvent>
#include <QDrag>
#include <QMimeData>
#include <QDropEvent>
#include <sstream>
#include <algorithm>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QPainter>
#include <functional>

// Helper class for clickable labels
class ClickableLabel : public QObject
{
    Q_OBJECT

public:
    ClickableLabel(int agentIndex, std::function<void(int)> callback, QObject *parent = nullptr)
        : QObject(parent), agentIndex_(agentIndex), callback_(callback) {}

private:
    int agentIndex_;
    std::function<void(int)> callback_;
};

BTConfigDialog::BTConfigDialog(QWidget *parent)
    : QWizard(parent), xmlDoc_(nullptr), isUpdatingConfiguration_(false), 
      actorPanel_(qobject_cast<hunav_rviz2_panel::ActorPanel*>(parent))
{
    // setPage(Page_Welcome, new QWizardPage(this));
    setPage(Page_Configuration, new QWizardPage(this));
    setPage(Page_AgentConfig, new QWizardPage(this));
    setPage(Page_Preview, new QWizardPage(this));

    setWindowTitle("🤖 HuNavSim - Behavior Configuration Wizard");
    setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
    setMinimumSize(1200, 900);

    setStyleSheet(
        "QWizard {"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
        "    stop:0 #f8f9fa, stop:1 #e9ecef);"
        "  border-radius: 12px;"
        "}"
        "QWizard QFrame {"
        "  background-color: transparent;"
        "}"
        "QWizard > QWidget {"
        "  background-color: transparent;"
        "}"
        /* Enhanced Title Styling */
        "QWizard .QLabel {"
        "  font-family: 'Segoe UI', 'Helvetica', 'Arial', sans-serif;"
        "}"
        /* Wizard Page Title Styling */
        "QWizard QWizardPage::title {"
        "  color: #2c3e50;"
        "  font-size: 24px;"
        "  font-weight: bold;"
        "  font-family: 'Segoe UI', 'Helvetica', 'Arial', sans-serif;"
        "  padding: 12px 0px 8px 0px;"
        "  margin-bottom: 5px;"
        "  background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
        "    stop:0 rgba(52, 152, 219, 0.1), stop:1 rgba(26, 188, 156, 0.1));"
        "  border-left: 4px solid #3498db;"
        "  border-radius: 4px;"
        "}"
        /* Wizard Page Subtitle Styling */
        "QWizard QWizardPage::subtitle {"
        "  color: #5d6d7e;"
        "  font-size: 14px;"
        "  font-style: italic;"
        "  font-family: 'Segoe UI', 'Helvetica', 'Arial', sans-serif;"
        "  padding: 8px 0px 12px 0px;"
        "  margin-bottom: 15px;"
        "  line-height: 1.4;"
        "  background: rgba(108, 117, 125, 0.05);"
        "  border-radius: 4px;"
        "  border-left: 3px solid #6c757d;"
        "}");

    buildPages();
    setupPageTransitions();
}

// Destructor
BTConfigDialog::~BTConfigDialog()
{
    delete xmlDoc_;
}

void BTConfigDialog::buildPages()
{
    // Page 0: Comprehensive configuration page (select + assign)
    buildCombinedConfigurationPage();
    // Page 1: Per-agent configuration page (ordering + parameters)
    buildAgentConfigurationPage();
    // Page 2: Preview page
    buildPreviewPage();
}

void BTConfigDialog::setupPageTransitions()
{
    connect(this, &QWizard::currentIdChanged, this, [this](int id)
            {
        qDebug() << "Page changed to:" << id;
        
        switch (id) {
            case Page_Configuration:  // Block selection and assignment page (ID 0)
                // Initialize blocks when the page is first loaded
                if (blocks_.isEmpty()) {
                    initializeBlocks();
                }
                // Clear connection tracking when going back to first page to allow fresh setup
                clearParameterConnectionTracking();
                button(QWizard::NextButton)->setEnabled(true);
                button(QWizard::NextButton)->setText("Next: Agent Configuration");
                setButtonText(QWizard::NextButton, "Next: Agent Configuration");
                break;
                
            case Page_AgentConfig:   // Per-agent configuration page (ID 1)
                // Clear connection tracking FIRST to remove stale widget references
                qDebug() << "Page_AgentConfig: Starting setup...";
                clearParameterConnectionTracking();
                qDebug() << "Page_AgentConfig: clearParameterConnectionTracking() done";
                // Then refresh agent dropdowns and configuration panel
                updateAgentDropdowns();
                qDebug() << "Page_AgentConfig: updateAgentDropdowns() done";
                // Parameter values are automatically restored during connection setup
                setupAllParameterConnections();
                qDebug() << "Page_AgentConfig: setupAllParameterConnections() done";
                button(QWizard::NextButton)->setEnabled(true);
                button(QWizard::NextButton)->setText("Next: Preview");
                setButtonText(QWizard::NextButton, "Next: Preview");
                button(QWizard::NextButton)->setVisible(true);
                button(QWizard::FinishButton)->setVisible(false);
                break;
                
            case Page_Preview:       // Preview page (ID 2)
                generateFinalPreview();
                // Hide the wizard's finish button and use our custom apply button
                button(QWizard::NextButton)->setVisible(false);
                button(QWizard::FinishButton)->setVisible(false);
                // Ensure back button is available
                button(QWizard::BackButton)->setEnabled(true);
                button(QWizard::BackButton)->setText("← Back to Configuration");
                break;
        } });
}

void BTConfigDialog::setAgentList(const QStringList &agentNames, const QStringList &behaviorTypes)
{
    agentNames_ = agentNames;
    behaviorTypes_ = behaviorTypes;

    // Initialize agent assignments
    agents_.clear();
    for (int i = 0; i < agentNames.size(); ++i)
    {
        AgentAssignment assignment;
        assignment.agentIndex = i;
        assignment.agentName = agentNames[i];
        assignment.behaviorType = i < behaviorTypes.size() ? behaviorTypes[i] : "Regular";
        assignment.assignedBlocks.clear();
        agents_.append(assignment);
    }

    // Update agent dropdowns if they exist
    updateAgentDropdowns();

    // Update assignment matrix headers with loaded agent names
    if (assignmentMatrix_)
    {
        updateAssignmentMatrix();
    }
}

void BTConfigDialog::setAvailableGoals(const QList<int> &goalIds)
{
    availableGoals_ = goalIds;
}

void BTConfigDialog::onBlockConfigChanged()
{

    // Update validation feedback to highlight any issues
    updateValidationFeedback();

    // Update XML preview if available
    if (xmlPreviewEdit_)
    {
        updateXMLPreview();
    }
}

void BTConfigDialog::buildPreviewPage()
{
    auto previewPage = page(Page_Preview);
    previewPage->setTitle("📋 Configuration Preview");
    previewPage->setSubTitle("Review and validate your agent behavior configuration before applying the changes to the behavior tree files. Click on any agent to examine its specific XML configuration.");

    auto layout = new QVBoxLayout(previewPage);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(10);

    // Main content with vertical layout
    auto splitter = new QSplitter(Qt::Vertical);

    // Top: Compact assignment summary
    auto summaryWidget = createAssignmentSummary();

    // Bottom: Generated XML preview
    auto xmlWidget = createXMLPreview();

    splitter->addWidget(summaryWidget);
    splitter->addWidget(xmlWidget);

    // Set summary widget size policy to size to content
    summaryWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    xmlWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    layout->addWidget(splitter, 1);

    // Action buttons
    auto buttonLayout = new QHBoxLayout;
    auto applyBtn = new QPushButton("Apply to Agents");

    applyBtn->setStyleSheet(
        "QPushButton { background: #28a745; color: white; border: none; padding: 12px 30px; "
        "border-radius: 6px; font-weight: bold; font-size: 14px; }"
        "QPushButton:hover { background: #218838; }"
        "QPushButton:pressed { background: #1e7e34; }");

    buttonLayout->addStretch();
    buttonLayout->addWidget(applyBtn);

    layout->addLayout(buttonLayout);

    // Store reference for later use
    applyBtn_ = applyBtn;

    // Connect button
    connect(applyBtn, &QPushButton::clicked, this, &BTConfigDialog::applyConfiguration);
}

QWidget *BTConfigDialog::createAssignmentSummary()
{
    auto widget = new QWidget;
    widget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    auto layout = new QVBoxLayout(widget);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    auto titleLabel = new QLabel("📊 Summary");
    titleLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: #495057; margin-bottom: 8px;");
    layout->addWidget(titleLabel);

    // Scroll area for assignment summary
    assignmentSummaryArea_ = new QScrollArea;
    assignmentSummaryArea_->setWidgetResizable(true);
    assignmentSummaryArea_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    assignmentSummaryArea_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    assignmentSummaryArea_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    assignmentSummaryArea_->setStyleSheet("QScrollArea { border: 1px solid #ddd; border-radius: 4px; }");

    assignmentSummaryWidget_ = new QWidget;
    assignmentSummaryWidget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    assignmentSummaryLayout_ = new QVBoxLayout(assignmentSummaryWidget_);

    auto noDataLabel = new QLabel("No configuration data available.\nGo back to configure agents.");
    noDataLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
    noDataLabel->setAlignment(Qt::AlignCenter);
    assignmentSummaryLayout_->addWidget(noDataLabel);

    assignmentSummaryArea_->setWidget(assignmentSummaryWidget_);
    layout->addWidget(assignmentSummaryArea_);

    return widget;
}

QWidget *BTConfigDialog::createXMLPreview()
{
    auto widget = new QWidget;
    auto layout = new QVBoxLayout(widget);
    layout->setContentsMargins(10, 10, 10, 10);

    auto titleLabel = new QLabel("📝 Generated XML");
    titleLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: #495057; margin-bottom: 8px;");
    layout->addWidget(titleLabel);

    // Agent selection for XML preview
    auto agentSelectionLayout = new QHBoxLayout;
    auto agentLabel = new QLabel("View XML patch for:");
    agentLabel->setStyleSheet("font-weight: bold; color: #555; font-size: 14px;");

    agentPreviewCombo_ = new QComboBox;
    agentPreviewCombo_->setMinimumWidth(200);
    agentPreviewCombo_->setStyleSheet(
        "QComboBox { padding: 4px 8px; border: 1px solid #ccc; border-radius: 4px; font-size: 13px; }");

    // Populate agent preview combo - use actual loaded agents
    int agentCount = getAssignmentMatrixAgentCount();

    if (agentCount == 0)
    {
        // No agents loaded yet - add placeholder
        agentPreviewCombo_->addItem("No agents loaded", -1);
        agentPreviewCombo_->setEnabled(false);
    }
    else
    {
        // Populate with actual loaded agents
        for (int i = 0; i < agentCount; ++i)
        {
            QString agentName;
            if (!agentNames_.isEmpty() && i < agentNames_.size())
            {
                agentName = QString("Agent %1").arg(i + 1);
            }
            agentPreviewCombo_->addItem(agentName, i);
        }
    }

    agentSelectionLayout->addWidget(agentLabel);
    agentSelectionLayout->addWidget(agentPreviewCombo_);
    agentSelectionLayout->addStretch();
    layout->addLayout(agentSelectionLayout);

    // XML preview text area
    xmlPreviewEdit_ = new QTextEdit;
    xmlPreviewEdit_->setStyleSheet(
        "QTextEdit { font-family: 'Courier New', monospace; font-size: 12px; "
        "border: 1px solid #ddd; border-radius: 4px; background: #f8f9fa; }");
    xmlPreviewEdit_->setReadOnly(true);

    // Set initial placeholder content
    if (agentCount == 0)
    {
        xmlPreviewEdit_->setText("<!-- No agents loaded yet -->\n"
                                 "<!-- Load agents and configure blocks to see XML preview -->\n"
                                 "<!-- Go back to previous pages to set up your configuration -->");
    }

    layout->addWidget(xmlPreviewEdit_);

    // Connect agent selection to update XML preview
    connect(agentPreviewCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this](int /*index*/)
            {
                int agentIndex = agentPreviewCombo_->currentData().toInt();
                if (agentIndex >= 0 && xmlPreviewEdit_)
                {
                    updateXMLPreviewForAgent(agentIndex);
                }
            });

    return widget;
}

void BTConfigDialog::buildCombinedConfigurationPage()
{
    auto configPage = page(Page_Configuration); // Use the new Page_Configuration (ID 0)
    configPage->setTitle("📚 Block Selection & Assignment");
    configPage->setSubTitle("Discover the available behavior blocks below, then use the interactive assignment matrix to specify which behaviors each agent should execute in their behavior tree.");

    auto layout = new QVBoxLayout(configPage);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(10);

    initializeBlocks();

    auto assignmentWidget = createAssignmentWidget();
    layout->addWidget(assignmentWidget, 1);
}

void BTConfigDialog::buildAgentConfigurationPage()
{
    auto configPage = page(Page_AgentConfig);
    configPage->setTitle("⚙️ Agent Configuration");
    configPage->setSubTitle("Fine-tune individual agent behaviors: select an agent, organize their execution order using the up/down buttons, enable randomization for dynamic behavior, and click on specific blocks to customize their parameters.");

    auto layout = new QVBoxLayout(configPage);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(15);

    // Create the per-agent configuration widget
    auto agentConfigWidget = createPerAgentConfigWidget();
    layout->addWidget(agentConfigWidget);
}

void BTConfigDialog::initializeBlocks()
{
    blocks_ = {
        {"EngageRobot", "Robot Interaction", "Detect, look at, and approach robot", {"IsRobotVisible", "LookAtRobot", "ApproachRobot"}},
        {"TalkInteract", "Social Interaction", "Detect, approach, and engage in conversation with agents", {"IsAgentVisible", "IsAgentClose", "ConversationFormation"}},
        {"RobotAvoidance", "Robot Awareness", "Detect robot, stop, wait, and resume when clear", {"IsRobotVisible", "IsRobotClose", "StopMovement", "Inverter", "IsRobotClose", "ResumeMovement"}},
        {"FollowAgent", "Follow Agent", "Detect, approach, and follow another agent", {"IsAgentVisible", "IsAgentClose", "FollowAgent"}},
        {"BlockingBehavior", "Robot Blocking", "Detect robot, look at it, then block its path", {"IsRobotVisible", "LookAtRobot", "BlockRobot"}},
        {"GroupFormation", "Group Coordination", "Find nearest agent and coordinate group walking", {"FindNearestAgent", "SetGroupWalk"}},
        {"SpeechDetection", "Audio-Triggered Social", "Listen for speech, approach speaker, and join conversation", {"IsAnyoneSpeaking", "ApproachAgent", "ConversationFormation"}},
        {"AttentionSeeking", "Attention Response", "Detect when being watched, look back, and speak up", {"IsAnyoneLookingAtMe", "LookAtAgent", "SaySomething"}},
        {"GreetingInitiator", "Friendly Greeter", "See nearby agents and greet them with a message", {"IsAgentVisible", "SaySomething"}},
        {"ProtectiveGuardian", "Guardian Protector", "Detect robot threat to nearby agents and intervene protectively", {"IsRobotVisible", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"}}};
}

QWidget *BTConfigDialog::createPerAgentConfigWidget()
{
    auto widget = new QWidget;
    auto layout = new QVBoxLayout(widget);
    layout->setContentsMargins(10, 10, 10, 10);

    // Agent selection in a compact styled frame
    auto agentSelectionFrame = new QFrame;
    agentSelectionFrame->setStyleSheet(
        "QFrame { background: white; border: 1px solid #007bff; border-radius: 6px; "
        "padding: 8px; margin: 2px; }");
    agentSelectionFrame->setMaximumHeight(70);

    auto selectionLayout = new QHBoxLayout(agentSelectionFrame);
    selectionLayout->setContentsMargins(8, 6, 8, 6);
    selectionLayout->setSpacing(10);

    auto titleLabel = new QLabel("👤 Choose agent to configure:");
    titleLabel->setStyleSheet(
        "QLabel { font-size: 13px; font-weight: bold; color: #007bff; }");
    titleLabel->setMinimumWidth(60);
    selectionLayout->addWidget(titleLabel);

    // Agent selection combo
    agentConfigCombo_ = new QComboBox;

    // Determine actual agent count from assignment matrix or agent names
    int agentCount = getAssignmentMatrixAgentCount();

    // Always set up the combo box styling for compact layout
    agentConfigCombo_->setStyleSheet(
        "QComboBox { padding: 6px 12px; border: 2px solid #007bff; border-radius: 4px; "
        "font-size: 14px; min-width: 150px; }");
    agentConfigCombo_->setMaximumHeight(35);

    if (agentCount == 0)
    {
        // No agents configured yet
        agentConfigCombo_->addItem("No agents loaded", -1);
        agentConfigCombo_->setEnabled(false);
    }
    else
    {
        // Populate with actual loaded agents
        for (int i = 0; i < agentCount; ++i)
        {
            QString agentDisplayName;
            if (!agentNames_.isEmpty() && i < agentNames_.size())
            {
                // Use actual agent names and behavior types
                QString behaviorType = i < behaviorTypes_.size() ? behaviorTypes_[i] : "Regular";
                agentDisplayName = QString("Agent %1 (%2)")
                                       .arg(i + 1)
                                       .arg(behaviorType);
            }
            else
            {
                // Use default names for loaded agents without specific names
                agentDisplayName = QString("Agent %1").arg(i + 1);
            }
            agentConfigCombo_->addItem(agentDisplayName, i);
        }
    }

    selectionLayout->addWidget(agentConfigCombo_);
    selectionLayout->addStretch();
    layout->addWidget(agentSelectionFrame);

    // Configuration content without overall scroll area
    configContentWidget_ = new QWidget;
    configContentLayout_ = new QVBoxLayout(configContentWidget_);

    // Add initial content based on agent count
    if (agentCount == 0)
    {
        auto noConfigLabel = new QLabel("Load agents first to configure their behavior parameters.");
        noConfigLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
        noConfigLabel->setAlignment(Qt::AlignCenter);
        configContentLayout_->addWidget(noConfigLabel);
    }
    else
    {
        auto noConfigLabel = new QLabel("Select blocks and assign them to agents to configure parameters.");
        noConfigLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
        noConfigLabel->setAlignment(Qt::AlignCenter);
        configContentLayout_->addWidget(noConfigLabel);
    }

    // Add the content widget directly to layout
    layout->addWidget(configContentWidget_, 1);

    connect(agentConfigCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this](int)
            {
                // Only update if we have valid content layout
                if (configContentLayout_)
                {
                    updateConfigurationPanel();
                }
            });

    // Initialize the configuration panel to show default state
    if (agentCount > 0)
    {
        QTimer::singleShot(0, this, [this]()
                           { 
            if (configContentLayout_) {
                updateConfigurationPanel(); 
            } });
    }

    return widget;
}

QWidget *BTConfigDialog::createAssignmentWidget()
{
    auto widget = new QWidget;
    widget->setStyleSheet("QWidget { background: #f8f9fa; }");

    auto layout = new QVBoxLayout(widget);
    layout->setContentsMargins(15, 15, 15, 15);
    layout->setSpacing(20);

    // Create main horizontal layout: Block Info on left, Assignment Matrix on right (primary)
    auto mainLayout = new QHBoxLayout;
    mainLayout->setSpacing(20);

    // LEFT SECTION: Block Information Sidebar
    auto blocksFrame = new QFrame;
    blocksFrame->setStyleSheet("QFrame { background: white; border: 2px solid #007bff; border-radius: 12px; }");
    blocksFrame->setMinimumWidth(400);
    blocksFrame->setMaximumWidth(480);

    auto blocksLayout = new QVBoxLayout(blocksFrame);
    blocksLayout->setContentsMargins(15, 12, 15, 12);
    blocksLayout->setSpacing(8);

    // Sidebar title
    auto titleLabel = new QLabel("📚 Available Behavior Blocks");
    titleLabel->setStyleSheet(
        "QLabel { font-size: 14px; font-weight: bold; color: #007bff; "
        "padding: 8px; background: rgba(0, 123, 255, 0.1); border-radius: 6px; }");
    titleLabel->setAlignment(Qt::AlignCenter);
    blocksLayout->addWidget(titleLabel);

    // Create vertical layout for block info in sidebar
    auto blocksInfoLayout = new QVBoxLayout;
    blocksInfoLayout->setContentsMargins(0, 0, 0, 0);
    blocksInfoLayout->setSpacing(10);

    // Display blocks vertically in sidebar
    for (int i = 0; i < blocks_.size(); ++i)
    {
        const auto &block = blocks_[i];

        auto blockInfo = new QLabel(QString("%1 <b>%2</b><br><small>%3</small>")
                                        .arg(getBlockIcon(block.blockId))
                                        .arg(block.displayName)
                                        .arg(block.description));
        blockInfo->setStyleSheet(
            "QLabel { font-size: 12px; color: #495057; padding: 8px; "
            "background: rgba(0, 123, 255, 0.03); border-radius: 4px; "
            "border-left: 3px solid #007bff; margin: 2px 0; }");
        blockInfo->setWordWrap(true);
        blockInfo->setTextFormat(Qt::RichText);

        blocksInfoLayout->addWidget(blockInfo);
    }

    blocksLayout->addLayout(blocksInfoLayout);
    blocksLayout->addStretch();

    // RIGHT SECTION: Agent Assignment Matrix
    auto agentsFrame = new QFrame;
    agentsFrame->setStyleSheet("QFrame { background: white; border: 2px solid #28a745; border-radius: 12px; }");

    auto agentsLayout = new QVBoxLayout(agentsFrame);
    agentsLayout->setContentsMargins(20, 15, 20, 15);
    agentsLayout->setSpacing(12);

    // Title
    auto agentsTitle = new QLabel("👥 Agent Assignment Matrix");
    agentsTitle->setStyleSheet(
        "QLabel { font-size: 16px; font-weight: bold; color: #28a745; "
        "padding: 8px; background: rgba(40, 167, 69, 0.1); border-radius: 6px; }");
    agentsTitle->setAlignment(Qt::AlignCenter);
    agentsLayout->addWidget(agentsTitle);

    // Quick assignment buttons
    auto buttonLayout = new QHBoxLayout;

    auto assignAllBtn = new QPushButton("✅ Assign All");
    assignAllBtn->setStyleSheet(
        "QPushButton { background: #28a745; color: white; border: none;  "
        "border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 13px; }"
        "QPushButton:hover { background: #218838; }");
    assignAllBtn->setToolTip("Assign all blocks to all agents");

    auto clearAllBtn = new QPushButton("🗑️ Clear All");
    clearAllBtn->setStyleSheet(
        "QPushButton { background: #dc3545; color: white; border: none; "
        "border-radius: 4px; padding: 6px 12px; font-weight: bold; font-size: 13px; }"
        "QPushButton:hover { background: #c82333; }");
    clearAllBtn->setToolTip("Clear all assignments");

    buttonLayout->addWidget(assignAllBtn);
    buttonLayout->addWidget(clearAllBtn);
    buttonLayout->addStretch();
    agentsLayout->addLayout(buttonLayout);

    // Assignment matrix
    assignmentMatrix_ = new QTableWidget;
    assignmentMatrix_->setStyleSheet(
        "QTableWidget { border: 1px solid #ddd; gridline-color: #ddd; background: white; font-size: 14px; }"
        "QHeaderView::section { background-color: #f8f9fa; color: #333; font-weight: bold; "
        "border: 1px solid #ddd; padding: 8px; font-size: 14px; }"
        "QTableWidget::item { padding: 6px; font-size: 14px; }");

    agentsLayout->addWidget(assignmentMatrix_, 1);

    // Add both sections to main horizontal layout
    mainLayout->addWidget(blocksFrame);
    mainLayout->addWidget(agentsFrame, 1);

    layout->addLayout(mainLayout, 1);

    // Connect buttons
    connect(assignAllBtn, &QPushButton::clicked, this, &BTConfigDialog::assignSelectedBlocksToAllAgents);
    connect(clearAllBtn, &QPushButton::clicked, this, &BTConfigDialog::clearAllAssignments);

    // Initialize the assignment matrix with all blocks visible
    initializeFullAssignmentMatrix();

    return widget;
}

QString BTConfigDialog::getBlockIcon(const QString &blockId) const
{
    if (blockId == "EngageRobot")
        return "🤖";
    if (blockId == "TalkInteract")
        return "💬";
    if (blockId == "RobotAvoidance")
        return "⚠️";
    if (blockId == "FollowAgent")
        return "👣";
    if (blockId == "BlockingBehavior")
        return "🚧";
    if (blockId == "GroupFormation")
        return "👥";
    if (blockId == "SpeechDetection")
        return "👂";
    if (blockId == "AttentionSeeking")
        return "👁️";
    if (blockId == "GreetingInitiator")
        return "👋";
    if (blockId == "ProtectiveGuardian")
        return "🛡️";
    return "📦";
}

void BTConfigDialog::initializeFullAssignmentMatrix()
{
    if (!assignmentMatrix_)
        return;

    QStringList allBlocks;
    for (const auto &block : blocks_)
    {
        allBlocks << block.displayName;
    }

    // Set up matrix dimensions
    int agentCount = getAgentCount();
    assignmentMatrix_->setRowCount(allBlocks.size());
    assignmentMatrix_->setColumnCount(agentCount);

    // Set headers
    assignmentMatrix_->setVerticalHeaderLabels(allBlocks);

    // Create agent column headers using actual loaded agent names
    QStringList agentHeaders;
    for (int i = 0; i < agentCount; ++i)
    {
        QString agentName;
        if (!agentNames_.isEmpty() && i < agentNames_.size())
        {
            // Use the actual loaded agent name
            agentName = QString("Agent %1").arg(i + 1);
        }
        agentHeaders << agentName;
    }
    assignmentMatrix_->setHorizontalHeaderLabels(agentHeaders);

    // Add checkboxes to matrix for all blocks
    for (int row = 0; row < allBlocks.size(); ++row)
    {
        for (int col = 0; col < agentCount; ++col)
        {
            auto checkbox = new QCheckBox;
            checkbox->setStyleSheet(
                "QCheckBox { margin: 6px; }"
                "QCheckBox::indicator { width: 18px; height: 18px; border: 2px solid #6c757d; "
                "border-radius: 3px; background: white; }"
                "QCheckBox::indicator:checked { background: #28a745; border-color: #1e7e34; }"
                "QCheckBox::indicator:hover { border-color: #007bff; }");

            // Ensure agents_ list is properly sized for all agents in the matrix
            while (agents_.size() <= col)
            {
                AgentAssignment assignment;
                assignment.agentIndex = agents_.size();
                assignment.agentName = (assignment.agentIndex < agentNames_.size()) ? agentNames_[assignment.agentIndex] : QString("Agent %1").arg(assignment.agentIndex + 1);
                assignment.behaviorType = "Regular";
                assignment.assignedBlocks.clear();
                agents_.append(assignment);
            }

            // Check if this block is already assigned to this agent
            if (row < blocks_.size())
            {
                const QString &blockId = blocks_[row].blockId;
                bool isAssigned = agents_[col].assignedBlocks.contains(blockId);
                checkbox->setChecked(isAssigned);
            }

            // Create a widget to center the checkbox
            auto cellWidget = new QWidget;
            auto cellLayout = new QHBoxLayout(cellWidget);
            cellLayout->addStretch();
            cellLayout->addWidget(checkbox);
            cellLayout->addStretch();
            cellLayout->setContentsMargins(0, 0, 0, 0);

            // Connect checkbox to update agent assignments
            connect(checkbox, &QCheckBox::toggled, [this, row, col](bool checked)
                    { updateAgentAssignmentFromMatrix(row, col, checked); });

            assignmentMatrix_->setCellWidget(row, col, cellWidget);
        }
    }

    // Resize to content
    assignmentMatrix_->resizeColumnsToContents();
    assignmentMatrix_->resizeRowsToContents();
}

void BTConfigDialog::updateAssignmentMatrix()
{
    if (!assignmentMatrix_)
        return;

    qDebug() << "updateAssignmentMatrix: Updating matrix with" << agents_.size() << "agents";

    // Update agent column headers with current agent names
    int agentCount = getAgentCount();

    // Update column count if needed
    int oldColumnCount = assignmentMatrix_->columnCount();
    if (oldColumnCount != agentCount)
    {
        assignmentMatrix_->setColumnCount(agentCount);

        // Add checkboxes for new columns if we increased the count
        if (agentCount > oldColumnCount)
        {
            for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
            {
                for (int col = oldColumnCount; col < agentCount; ++col)
                {
                    auto checkbox = new QCheckBox;
                    checkbox->setStyleSheet(
                        "QCheckBox { margin: 6px; }"
                        "QCheckBox::indicator { width: 18px; height: 18px; border: 2px solid #6c757d; "
                        "border-radius: 3px; background: white; }"
                        "QCheckBox::indicator:checked { background: #28a745; border-color: #1e7e34; }"
                        "QCheckBox::indicator:hover { border-color: #007bff; }");

                    // Ensure agents_ list is properly sized for this checkbox
                    while (agents_.size() <= col)
                    {
                        AgentAssignment assignment;
                        assignment.agentIndex = agents_.size();
                        assignment.agentName = (assignment.agentIndex < agentNames_.size()) ? agentNames_[assignment.agentIndex] : QString("Agent %1").arg(assignment.agentIndex + 1);
                        assignment.behaviorType = "Regular";
                        assignment.assignedBlocks.clear();
                        agents_.append(assignment);
                    }

                    // Check if this block is already assigned to this agent
                    if (row < blocks_.size())
                    {
                        const QString &blockId = blocks_[row].blockId;
                        bool isAssigned = agents_[col].assignedBlocks.contains(blockId);
                        checkbox->setChecked(isAssigned);
                    }

                    // Create a widget to center the checkbox
                    auto cellWidget = new QWidget;
                    auto cellLayout = new QHBoxLayout(cellWidget);
                    cellLayout->addStretch();
                    cellLayout->addWidget(checkbox);
                    cellLayout->addStretch();
                    cellLayout->setContentsMargins(0, 0, 0, 0);

                    // Connect checkbox to update agent assignments
                    connect(checkbox, &QCheckBox::toggled, [this, row, col](bool checked)
                            { updateAgentAssignmentFromMatrix(row, col, checked); });

                    assignmentMatrix_->setCellWidget(row, col, cellWidget);
                }
            }
        }
    }

    // Sync checkbox state with loaded agent assignments
    for (int row = 0; row < assignmentMatrix_->rowCount() && row < blocks_.size(); ++row)
    {
        const QString &blockId = blocks_[row].blockId;

        for (int col = 0; col < assignmentMatrix_->columnCount() && col < agents_.size(); ++col)
        {
            // Get the checkbox from the cell
            QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, col);
            if (checkbox)
            {
                // Check if this block is assigned to this agent
                bool isAssigned = agents_[col].assignedBlocks.contains(blockId);

                // Update checkbox state without triggering signal
                checkbox->blockSignals(true);
                checkbox->setChecked(isAssigned);
                checkbox->blockSignals(false);

                if (isAssigned)
                {
                    qDebug() << "Marked block" << blockId << "as assigned to agent" << col;
                }
            }
        }
    }

    qDebug() << "updateAssignmentMatrix: Completed update";

    // Update agent headers with current loaded names
    QStringList agentHeaders;
    for (int i = 0; i < agentCount; ++i)
    {
        QString agentName;
        agentName = QString("Agent %1").arg(i + 1);
        agentHeaders << agentName;
    }
    assignmentMatrix_->setHorizontalHeaderLabels(agentHeaders);

    // Resize to content
    assignmentMatrix_->resizeColumnsToContents();
    assignmentMatrix_->resizeRowsToContents();
}

QCheckBox *BTConfigDialog::getCheckBoxFromMatrixCell(int row, int col) const
{
    if (!assignmentMatrix_ || row >= assignmentMatrix_->rowCount() || col >= assignmentMatrix_->columnCount())
    {
        return nullptr;
    }

    auto cellWidget = assignmentMatrix_->cellWidget(row, col);
    if (!cellWidget)
    {
        return nullptr;
    }

    // Try direct cast first
    QCheckBox *checkbox = qobject_cast<QCheckBox *>(cellWidget);

    // If that fails, look for checkbox inside container widget
    if (!checkbox)
    {
        checkbox = cellWidget->findChild<QCheckBox *>();
    }

    return checkbox;
}

void BTConfigDialog::updateAgentAssignmentFromMatrix(int row, int col, bool checked)
{
    // Ensure we have valid agents data structure
    while (agents_.size() <= col)
    {
        AgentAssignment assignment;
        assignment.agentIndex = agents_.size();
        assignment.agentName = (assignment.agentIndex < agentNames_.size()) ? agentNames_[assignment.agentIndex] : QString("Agent %1").arg(assignment.agentIndex + 1);
        assignment.behaviorType = "Regular";
        assignment.assignedBlocks.clear();
        agents_.append(assignment);
    }

    // Get the block ID from the row
    if (row >= blocks_.size())
    {
        qWarning() << "Invalid row index:" << row << "blocks size:" << blocks_.size();
        return;
    }

    const QString &blockId = blocks_[row].blockId;

    // Update the agent's assigned blocks
    if (checked)
    {
        // Add the block if not already present
        if (!agents_[col].assignedBlocks.contains(blockId))
        {
            agents_[col].assignedBlocks.append(blockId);
        }
    }
    else
    {
        // Remove the block
        agents_[col].assignedBlocks.removeAll(blockId);
    }

    // Update the configuration panel to reflect changes
    updateConfigurationPanel();

    // Update assignment summary widget if it exists
    if (assignmentSummaryWidget_)
    {
        updateAssignmentSummaryWidget();
    }
}

void BTConfigDialog::assignSelectedBlocksToAllAgents()
{
    if (!assignmentMatrix_)
        return;

    // First, update all checkboxes in the assignment matrix
    for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
    {
        for (int col = 0; col < assignmentMatrix_->columnCount(); ++col)
        {
            auto cellWidget = assignmentMatrix_->cellWidget(row, col);
            if (cellWidget)
            {
                // Find the checkbox inside the cell widget
                auto checkbox = cellWidget->findChild<QCheckBox *>();
                if (checkbox)
                {
                    checkbox->setChecked(true);
                }
            }
        }
    }

    updateConfigurationPanel();
}

void BTConfigDialog::updateAgentDropdowns()
{
    // Update the per-agent configuration dropdown
    if (agentConfigCombo_)
    {
        // Store current selection
        int currentSelection = agentConfigCombo_->currentData().toInt();

        // Clear and repopulate
        agentConfigCombo_->clear();

        if (agentNames_.isEmpty())
        {
            agentConfigCombo_->addItem("No agents loaded", -1);
            agentConfigCombo_->setEnabled(false);
        }
        else
        {
            agentConfigCombo_->setEnabled(true);
            for (int i = 0; i < agentNames_.size(); ++i)
            {
                QString behaviorType = i < behaviorTypes_.size() ? behaviorTypes_[i] : "Regular";
                QString displayName = QString("Agent %1 (%2)")
                                          .arg(i + 1)
                                          .arg(behaviorType);
                agentConfigCombo_->addItem(displayName, i);
            }

            // Restore selection if still valid
            if (currentSelection >= 0 && currentSelection < agentNames_.size())
            {
                for (int i = 0; i < agentConfigCombo_->count(); ++i)
                {
                    if (agentConfigCombo_->itemData(i).toInt() == currentSelection)
                    {
                        agentConfigCombo_->setCurrentIndex(i);
                        break;
                    }
                }
            }
        }
    }

    // Update the preview dropdown if it exists
    if (agentPreviewCombo_)
    {
        // Store current selection
        int currentPreviewSelection = agentPreviewCombo_->currentData().toInt();

        // Clear and repopulate
        agentPreviewCombo_->clear();

        if (agentNames_.isEmpty())
        {
            agentPreviewCombo_->addItem("No agents available", -1);
            agentPreviewCombo_->setEnabled(false);
        }
        else
        {
            agentPreviewCombo_->setEnabled(true);
            for (int i = 0; i < agentNames_.size(); ++i)
            {
                QString behaviorType = i < behaviorTypes_.size() ? behaviorTypes_[i] : "Regular";
                QString displayName = QString("Agent %1 (%2)")
                                          .arg(i + 1)
                                          .arg(behaviorType);
                agentPreviewCombo_->addItem(displayName, i);
            }

            // Restore selection if still valid
            if (currentPreviewSelection >= 0 && currentPreviewSelection < agentNames_.size())
            {
                for (int i = 0; i < agentPreviewCombo_->count(); ++i)
                {
                    if (agentPreviewCombo_->itemData(i).toInt() == currentPreviewSelection)
                    {
                        agentPreviewCombo_->setCurrentIndex(i);
                        break;
                    }
                }
            }
        }
    }

    // Trigger update of configuration panel if agents were loaded
    if (!agentNames_.isEmpty() && configContentLayout_)
    {
        updateConfigurationPanel();
    }
}

void BTConfigDialog::updateConfigurationPanel()
{
    if (!agentConfigCombo_ || !configContentLayout_)
        return;

    // Guard against concurrent updates
    if (isUpdatingConfiguration_)
    {
        return;
    }
    isUpdatingConfiguration_ = true;

    // Complete content widget reset
    configContentWidget_->hide();
    configContentWidget_->setUpdatesEnabled(false);

    // We'll add null checks where needed instead
    QSet<QWidget *> widgetsToDelete;

    // Delete all child widgets immediately - including nested children
    auto children = configContentWidget_->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly);
    for (auto child : children)
    {
        widgetsToDelete.insert(child);
        // Collect all descendants too for removal from blockConfigWidgets_
        auto allDescendants = child->findChildren<QWidget *>();
        for (auto descendant : allDescendants)
        {
            widgetsToDelete.insert(descendant);
            descendant->disconnect();
        }
        child->setParent(nullptr);
        // Disconnect all signals before deletion to prevent crashes
        child->disconnect();
        child->deleteLater();
    }

    // Remove references from blockConfigWidgets_ map BEFORE any signals fire
    for (auto it = blockConfigWidgets_.begin(); it != blockConfigWidgets_.end();)
    {
        if (widgetsToDelete.contains(it.value()))
        {
            it = blockConfigWidgets_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    
    // Clear connection tracking to prevent stale references
    connectedWidgets_.clear();

    // Clear layout completely
    if (configContentLayout_)
    {
        QLayoutItem *item;
        while ((item = configContentLayout_->takeAt(0)) != nullptr)
        {
            if (item->widget())
            {
                item->widget()->setParent(nullptr);
                // Disconnect all signals before deletion
                item->widget()->disconnect();
                item->widget()->deleteLater();
            }
            else if (item->layout())
            {
                delete item->layout();
            }
            delete item;
        }
    }

    // Force immediate processing
    QApplication::processEvents();

    // Slight delay to ensure UI stability before repopulating
    QTimer::singleShot(10, [this]()
                       {
        this->populateConfigurationContent();
        this->isUpdatingConfiguration_ = false; });
}

void BTConfigDialog::populateConfigurationContent()
{
    if (!agentConfigCombo_ || !configContentLayout_)
        return;

    int selectedAgent = agentConfigCombo_->currentData().toInt();

    // Determine actual agent count from assignment matrix or agent names
    int agentCount = getAssignmentMatrixAgentCount();
    if (agentCount == 0)
    {
        // No agents loaded
        auto noAgentsLabel = new QLabel("No agents loaded. Please load agents first to configure their behavior parameters.");
        noAgentsLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
        noAgentsLabel->setAlignment(Qt::AlignCenter);
        configContentLayout_->addWidget(noAgentsLabel);
        configContentWidget_->setUpdatesEnabled(true);
        configContentWidget_->show();
        return;
    }

    // Check if selected agent is valid
    if (selectedAgent < 0 || selectedAgent >= agentCount)
    {
        auto invalidAgentLabel = new QLabel("Invalid agent selection. Please select a valid agent from the dropdown.");
        invalidAgentLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
        invalidAgentLabel->setAlignment(Qt::AlignCenter);
        configContentLayout_->addWidget(invalidAgentLabel);
        configContentWidget_->setUpdatesEnabled(true);
        configContentWidget_->show();
        return;
    }

    // Ensure agents_ list is properly sized for the actual loaded agents
    while (agents_.size() < agentCount)
    {
        AgentAssignment assignment;
        assignment.agentIndex = agents_.size();
        if (!agentNames_.isEmpty() && agents_.size() < agentNames_.size())
        {
            assignment.agentName = agentNames_[agents_.size()];
            assignment.behaviorType = agents_.size() < behaviorTypes_.size() ? behaviorTypes_[agents_.size()] : "Regular";
        }
        else
        {
            assignment.agentName = QString("Agent %1").arg(agents_.size() + 1);
            assignment.behaviorType = "Regular";
        }
        assignment.assignedBlocks.clear();
        agents_.append(assignment);
    }

    // Get assigned blocks for this agent
    QStringList assignedBlocks;
    for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
    {
        QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, selectedAgent);
        if (checkbox && checkbox->isChecked())
        {
            QString blockName = assignmentMatrix_->verticalHeaderItem(row)->text();
            assignedBlocks << blockName;
        }
    }

    if (assignedBlocks.isEmpty())
    {
        auto noBlocksLabel = new QLabel("No blocks assigned to this agent.\nSelect blocks and assign them in the matrix.");
        noBlocksLabel->setStyleSheet("color: #666; font-style: italic; padding: 20px;");
        noBlocksLabel->setAlignment(Qt::AlignCenter);
        configContentLayout_->addWidget(noBlocksLabel);
        configContentWidget_->setUpdatesEnabled(true);
        configContentWidget_->show();
        return;
    }

    // Convert display names back to block IDs and update agent assignment
    QStringList blockIds;
    for (const QString &displayName : assignedBlocks)
    {
        for (const auto &block : blocks_)
        {
            if (block.displayName == displayName)
            {
                blockIds << block.blockId;
                break;
            }
        }
    }

    // Update the agent assignment with current blocks
    if (selectedAgent < agents_.size())
    {
        agents_[selectedAgent].assignedBlocks = blockIds;
    }

    // Create Block Execution Order Group
    auto orderingGroup = new QGroupBox("🎯 Block Execution Order");
    orderingGroup->setStyleSheet(
        "QGroupBox { font-weight: bold; color: #333; border: 2px solid #007bff; "
        "border-radius: 8px; margin: 10px 5px; padding-top: 15px; background: rgba(0, 123, 255, 0.05); }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 8px; }");

    auto orderingLayout = new QVBoxLayout(orderingGroup);

    // Random order checkbox
    auto randomOrderCheck = new QCheckBox("🎲 Randomize block execution order");
    randomOrderCheck->setStyleSheet(
        "QCheckBox { font-size: 14px; font-weight: 600; color: #495057; padding: 8px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; }"
        "QCheckBox::indicator:checked { background: #007bff; border: 2px solid #007bff; }"
        "QCheckBox::indicator:unchecked { background: white; border: 2px solid #ced4da; }");

    // Set current randomization state
    if (selectedAgent < agents_.size())
    {
        randomOrderCheck->setChecked(agents_[selectedAgent].randomizeOrder);
    }

    orderingLayout->addWidget(randomOrderCheck);

    // Create centered ordering list with arrows using new method
    QStringList currentOrder = blockIds;
    if (selectedAgent < agents_.size() && !agents_[selectedAgent].assignedBlocks.isEmpty())
    {
        currentOrder = agents_[selectedAgent].assignedBlocks;
    }

    auto orderingContainer = createOrderingList(currentOrder, selectedAgent);

    // Get the hidden list widget for compatibility
    auto orderListWidget = orderingContainer->findChild<QListWidget *>("orderListWidget");

    // Enable/disable ordering based on randomization setting
    if (orderListWidget)
    {
        orderListWidget->setEnabled(!randomOrderCheck->isChecked());
    }

    // Connect randomization checkbox
    connect(randomOrderCheck, &QCheckBox::toggled, [this, selectedAgent, orderListWidget, orderingContainer](bool checked)
            {
        if (selectedAgent < agents_.size()) {
            agents_[selectedAgent].randomizeOrder = checked;
            if (orderListWidget) {
                orderListWidget->setEnabled(!checked);
            }
            
            // Update visual style for randomization mode
            auto scrollArea = orderingContainer->property("scrollArea").value<QScrollArea*>();
            if (scrollArea) {
                if (checked) {
                    scrollArea->setStyleSheet(
                        "QScrollArea { border: 2px solid #ffc107; border-radius: 6px; background: rgba(255, 193, 7, 0.1); }"
                        "QScrollBar:vertical { width: 8px; background: #f0f0f0; border-radius: 4px; }"
                        "QScrollBar::handle:vertical { background: #ffc107; border-radius: 4px; min-height: 20px; }");
                } else {
                    scrollArea->setStyleSheet(
                        "QScrollArea { border: 2px solid #ced4da; border-radius: 6px; background: white; }"
                        "QScrollBar:vertical { width: 8px; background: #f0f0f0; border-radius: 4px; }"
                        "QScrollBar::handle:vertical { background: #007bff; border-radius: 4px; min-height: 20px; }");
                }
            }
        } });

    // Connect list reordering to update agent assignment
    if (orderListWidget)
    {
        connect(orderListWidget, &QListWidget::itemChanged, [this, selectedAgent, orderListWidget]()
                {
            if (selectedAgent >= agents_.size()) return;
            
            QStringList newOrder;
            for (int i = 0; i < orderListWidget->count(); ++i) {
                auto item = orderListWidget->item(i);
                if (item) {
                    newOrder << item->data(Qt::UserRole).toString();
                }
            }
            agents_[selectedAgent].assignedBlocks = newOrder; });
    }

    orderingLayout->addWidget(orderingContainer);

    // Add buttons for ordering
    auto buttonLayout = new QHBoxLayout;
    buttonLayout->setContentsMargins(0, 8, 0, 4);
    buttonLayout->setSpacing(8);

    auto moveUpBtn = new QPushButton("⬆️ Up");
    moveUpBtn->setStyleSheet(
        "QPushButton { padding: 6px 12px; border: 1px solid #007bff; border-radius: 4px; "
        "background: #007bff; color: white; font-weight: 600; font-size: 12px; }"
        "QPushButton:hover { background: #0056b3; }");

    auto moveDownBtn = new QPushButton("⬇️ Down");
    moveDownBtn->setStyleSheet(moveUpBtn->styleSheet());

    auto shuffleBtn = new QPushButton("🔀 Shuffle");
    shuffleBtn->setStyleSheet(
        "QPushButton { padding: 6px 12px; border: 1px solid #ffc107; border-radius: 4px; "
        "background: #ffc107; color: #212529; font-weight: 600; font-size: 12px; }"
        "QPushButton:hover { background: #e0a800; }");

    moveUpBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    moveDownBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    shuffleBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

    // Connect button functionality with visual block synchronization
    connect(moveUpBtn, &QPushButton::clicked, [orderListWidget, orderingContainer, this, selectedAgent]()
            {
        int currentRow = orderListWidget->currentRow();
        if (currentRow > 0) {
            // Remember the selected block ID
            QString selectedBlockId;
            auto selectedItem = orderListWidget->item(currentRow);
            if (selectedItem) {
                selectedBlockId = selectedItem->data(Qt::UserRole).toString();
            }
            
            auto item = orderListWidget->takeItem(currentRow);
            orderListWidget->insertItem(currentRow - 1, item);
            orderListWidget->setCurrentRow(currentRow - 1);
            
            // Update agent assignment
            if (selectedAgent < agents_.size()) {
                QStringList newOrder;
                for (int i = 0; i < orderListWidget->count(); ++i) {
                    auto listItem = orderListWidget->item(i);
                    if (listItem) {
                        newOrder << listItem->data(Qt::UserRole).toString();
                    }
                }
                agents_[selectedAgent].assignedBlocks = newOrder;
                
                // Recreate visual ordering display
                auto contentWidget = orderingContainer->property("contentWidget").value<QWidget*>();
                auto contentLayout = orderingContainer->property("contentLayout").value<QVBoxLayout*>();
                if (contentWidget && contentLayout) {
                    // Clear existing visual blocks 
                    while (contentLayout->count() > 0) {
                        QLayoutItem *child = contentLayout->takeAt(0);
                        if (child->widget()) {
                            child->widget()->deleteLater();
                        }
                        delete child;
                    }
                    
                    // Recreate visual blocks with new order and restore selection
                    recreateVisualBlocks(orderingContainer, newOrder, selectedAgent, selectedBlockId);
                }
            }
        } });

    connect(moveDownBtn, &QPushButton::clicked, [orderListWidget, orderingContainer, this, selectedAgent]()
            {
        int currentRow = orderListWidget->currentRow();
        if (currentRow >= 0 && currentRow < orderListWidget->count() - 1) {
            // Remember the selected block ID
            QString selectedBlockId;
            auto selectedItem = orderListWidget->item(currentRow);
            if (selectedItem) {
                selectedBlockId = selectedItem->data(Qt::UserRole).toString();
            }
            
            auto item = orderListWidget->takeItem(currentRow);
            orderListWidget->insertItem(currentRow + 1, item);
            orderListWidget->setCurrentRow(currentRow + 1);
            
            // Update agent assignment
            if (selectedAgent < agents_.size()) {
                QStringList newOrder;
                for (int i = 0; i < orderListWidget->count(); ++i) {
                    auto listItem = orderListWidget->item(i);
                    if (listItem) {
                        newOrder << listItem->data(Qt::UserRole).toString();
                    }
                }
                agents_[selectedAgent].assignedBlocks = newOrder;
                
                // Recreate visual ordering display
                auto contentWidget = orderingContainer->property("contentWidget").value<QWidget*>();
                auto contentLayout = orderingContainer->property("contentLayout").value<QVBoxLayout*>();
                if (contentWidget && contentLayout) {
                    // Clear existing visual blocks
                    while (contentLayout->count() > 0) {
                        QLayoutItem *child = contentLayout->takeAt(0);
                        if (child->widget()) {
                            child->widget()->deleteLater();
                        }
                        delete child;
                    }
                    
                    // Recreate visual blocks with new order and restore selection
                    recreateVisualBlocks(orderingContainer, newOrder, selectedAgent, selectedBlockId);
                }
            }
        } });

    connect(shuffleBtn, &QPushButton::clicked, [orderListWidget, orderingContainer, this, selectedAgent]()
            {
        if (orderListWidget->count() <= 1) return;
        
        // Remember the selected block ID
        QString selectedBlockId;
        auto selectedItems = orderListWidget->selectedItems();
        if (!selectedItems.isEmpty()) {
            selectedBlockId = selectedItems.first()->data(Qt::UserRole).toString();
        }
        
        // Get all items
        QList<QListWidgetItem*> items;
        for (int i = orderListWidget->count() - 1; i >= 0; --i) {
            items.append(orderListWidget->takeItem(i));
        }
        
        // Shuffle items
        for (int i = items.size() - 1; i > 0; --i) {
            int j = QRandomGenerator::global()->bounded(i + 1);
            items.swapItemsAt(i, j);
        }
        
        // Add back to list
        for (auto item : items) {
            orderListWidget->addItem(item);
        }
        
        // Restore selection if we had one
        if (!selectedBlockId.isEmpty()) {
            for (int i = 0; i < orderListWidget->count(); ++i) {
                auto item = orderListWidget->item(i);
                if (item && item->data(Qt::UserRole).toString() == selectedBlockId) {
                    orderListWidget->setCurrentRow(i);
                    break;
                }
            }
        }
        
        // Update agent assignment
        if (selectedAgent < agents_.size()) {
            QStringList newOrder;
            for (int i = 0; i < orderListWidget->count(); ++i) {
                auto listItem = orderListWidget->item(i);
                if (listItem) {
                    newOrder << listItem->data(Qt::UserRole).toString();
                }
            }
            agents_[selectedAgent].assignedBlocks = newOrder;
            
            // Recreate visual ordering display
            auto contentWidget = orderingContainer->property("contentWidget").value<QWidget*>();
            auto contentLayout = orderingContainer->property("contentLayout").value<QVBoxLayout*>();
            if (contentWidget && contentLayout) {
                // Clear existing visual blocks
                while (contentLayout->count() > 0) {
                    QLayoutItem *child = contentLayout->takeAt(0);
                    if (child->widget()) {
                        child->widget()->deleteLater();
                    }
                    delete child;
                }
                
                // Recreate visual blocks with new order and restore selection
                recreateVisualBlocks(orderingContainer, newOrder, selectedAgent, selectedBlockId);
            }
        } });

    buttonLayout->addWidget(moveUpBtn);
    buttonLayout->addWidget(moveDownBtn);
    buttonLayout->addWidget(shuffleBtn);
    buttonLayout->addStretch();

    // Add button layout only once
    orderingLayout->addLayout(buttonLayout);

    // Create Block Parameters Section with its own scroll area
    auto parametersGroup = new QGroupBox("⚙️ Block Parameters");
    parametersGroup->setStyleSheet(
        "QGroupBox { font-weight: bold; color: #333; border: 2px solid #28a745; "
        "border-radius: 8px; margin: 10px 5px; padding-top: 15px; background: rgba(40, 167, 69, 0.05); }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 8px; }");

    auto parametersMainLayout = new QVBoxLayout(parametersGroup);

    // Create scroll area for parameters content only
    auto parametersScrollArea = new QScrollArea;
    parametersScrollArea->setWidgetResizable(true);
    parametersScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    parametersScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    parametersScrollArea->setStyleSheet(
        "QScrollArea { border: none; background: transparent; }"
        "QScrollBar:vertical { width: 12px; background: #f0f0f0; border-radius: 6px; }"
        "QScrollBar::handle:vertical { background: #28a745; border-radius: 6px; min-height: 20px; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }");

    // Parameters content widget
    auto parametersContentWidget = new QWidget;
    auto parametersLayout = new QVBoxLayout(parametersContentWidget);

    // Add initial instruction message
    auto instructionLabel = new QLabel("👆 Click on a block from the execution order list to configure its parameters");
    instructionLabel->setStyleSheet(
        "QLabel { color: #6c757d; font-style: italic; font-size: 14px; padding: 30px; "
        "background: rgba(40, 167, 69, 0.05); border: 1px dashed #28a745; border-radius: 8px; "
        "margin: 20px; }");
    instructionLabel->setAlignment(Qt::AlignCenter);
    instructionLabel->setWordWrap(true);
    parametersLayout->addWidget(instructionLabel);
    parametersLayout->addStretch();

    // Complete the scroll area setup for parameters
    parametersScrollArea->setWidget(parametersContentWidget);
    parametersMainLayout->addWidget(parametersScrollArea);

    // Create horizontal splitter for ordering and parameters
    auto splitter = new QSplitter(Qt::Horizontal);
    splitter->setChildrenCollapsible(false);
    orderingGroup->setMinimumWidth(350);
    // Both sections should expand to use available vertical space
    orderingGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    parametersGroup->setMinimumWidth(350);
    // Parameters group should also expand to use available vertical space
    parametersGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Set the scroll area to use available space properly
    parametersScrollArea->setMinimumHeight(150);
    parametersScrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    splitter->addWidget(orderingGroup);
    splitter->addWidget(parametersGroup);

    // Set initial splitter proportions
    splitter->setStretchFactor(0, 45);
    splitter->setStretchFactor(1, 55);

    // Connect order list item selection to update parameters display
    // Use parametersContentWidget as context to ensure connection is cleaned up properly
    connect(orderListWidget, &QListWidget::itemClicked, parametersContentWidget, 
            [this, selectedAgent, parametersLayout, parametersContentWidget](QListWidgetItem *item)
            {
        if (!item) return;
        
        QString selectedBlockId = item->data(Qt::UserRole).toString();
        updateParametersForSelectedBlock(selectedBlockId, selectedAgent, parametersLayout, parametersContentWidget); });

    // Also connect to selection change
    connect(orderListWidget, &QListWidget::itemSelectionChanged, parametersContentWidget,
            [this, selectedAgent, parametersLayout, parametersContentWidget, orderListWidget]()
            {
        auto selectedItems = orderListWidget->selectedItems();
        if (selectedItems.isEmpty()) {
            // Show instruction message when nothing is selected
            showParametersInstruction(parametersLayout, parametersContentWidget);
        } else {
            QString selectedBlockId = selectedItems.first()->data(Qt::UserRole).toString();
            updateParametersForSelectedBlock(selectedBlockId, selectedAgent, parametersLayout, parametersContentWidget);
        } });

    configContentLayout_->addWidget(splitter, 1);

    // Re-enable updates and show the widget
    configContentWidget_->setUpdatesEnabled(true);
    configContentWidget_->show();
}

void BTConfigDialog::updateParametersForSelectedBlock(const QString &blockId, int agentIndex, QVBoxLayout *parametersLayout, QWidget *parametersContentWidget)
{
    // Clear existing parameters content
    QLayoutItem *item;
    while ((item = parametersLayout->takeAt(0)) != nullptr)
    {
        if (item->widget())
        {
            // Disconnect all signals to prevent dangling connections
            item->widget()->disconnect();
            item->widget()->deleteLater();
        }
        delete item;
    }

    // Find the block configuration
    const BlockConfig *blockConfig = nullptr;
    for (const auto &block : blocks_)
    {
        if (block.blockId == blockId)
        {
            blockConfig = &block;
            break;
        }
    }

    if (!blockConfig)
    {
        showParametersInstruction(parametersLayout, parametersContentWidget);
        return;
    }

    // Create parameters group for the selected block
    auto blockParamGroup = new QGroupBox(QString("⚙️ %1 Parameters").arg(blockConfig->displayName));
    blockParamGroup->setStyleSheet(
        "QGroupBox { font-weight: bold; color: #495057; border: 2px solid #28a745; "
        "border-radius: 8px; margin: 10px; padding-top: 15px; background: rgba(40, 167, 69, 0.05); }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 8px; }");

    auto blockParamLayout = new QFormLayout(blockParamGroup);

    // Add description
    auto descLabel = new QLabel(blockConfig->description);
    descLabel->setStyleSheet("color: #6c757d; font-style: italic; font-size: 12px; margin-bottom: 10px;");
    descLabel->setWordWrap(true);
    blockParamLayout->addRow(descLabel);

    // Add execution mode group
    auto executionModeGroup = new QGroupBox("Execution Mode");
    executionModeGroup->setStyleSheet(
        "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 10px; "
        // "border: 1px solid #dee2e6; border-radius: 4px; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
    
    auto executionLayout = new QVBoxLayout;
    
    // Radio buttons for execution modes
    auto alwaysRadio = new QRadioButton("🔄 Always Execute");
    alwaysRadio->setToolTip("Execute this block every time conditions are met");
    
    auto onceRadio = new QRadioButton("1️⃣  Execute Only Once");
    onceRadio->setToolTip("Execute this block only once per agent");
    
    auto randomRadio = new QRadioButton("🎲 Random Execution");
    randomRadio->setToolTip("Execute this block probabilistically based on a chance value");
    
    // Probability spinbox (only enabled when random mode is selected)
    auto probabilityWidget = new QWidget;
    auto probabilityLayout = new QHBoxLayout(probabilityWidget);
    probabilityLayout->setContentsMargins(20, 0, 0, 0);
    
    auto probabilityLabel = new QLabel("Probability:");
    auto probabilitySpin = new QDoubleSpinBox;
    probabilitySpin->setRange(0.0, 1.0);
    probabilitySpin->setValue(0.5);
    probabilitySpin->setSingleStep(0.1);
    probabilitySpin->setDecimals(2);
    probabilitySpin->setEnabled(false);
    probabilitySpin->setToolTip("Probability of executing this block (0.0 = never, 1.0 = always)");
    
    probabilityLayout->addWidget(probabilityLabel);
    probabilityLayout->addWidget(probabilitySpin);
    probabilityLayout->addStretch();
    
    executionLayout->addWidget(alwaysRadio);
    executionLayout->addWidget(onceRadio);
    executionLayout->addWidget(randomRadio);
    executionLayout->addWidget(probabilityWidget);
    
    executionModeGroup->setLayout(executionLayout);
    
    // Set current state from agent assignment
    bool isRunOnce = false;
    bool isRandom = false;
    double probability = 0.5;
    
    if (agentIndex < agents_.size())
    {
        isRunOnce = agents_[agentIndex].runOnceBlocks.contains(blockId);
        if (agents_[agentIndex].randomExecutionBlocks.contains(blockId))
        {
            isRandom = true;
            probability = agents_[agentIndex].randomExecutionBlocks[blockId];
        }
    }
    
    if (isRandom)
    {
        randomRadio->setChecked(true);
        probabilitySpin->setEnabled(true);
        probabilitySpin->setValue(probability);
    }
    else if (isRunOnce)
    {
        onceRadio->setChecked(true);
    }
    else
    {
        alwaysRadio->setChecked(true);
    }
    
    // Connect radio buttons with proper Qt parent context to avoid crashes
    // Use QPointer to safely handle widget deletion
    connect(alwaysRadio, &QRadioButton::toggled, alwaysRadio, [this, agentIndex, blockId, probabilitySpin](bool checked)
    {
        if (!checked || !probabilitySpin) return; // Only act on checked, not unchecked; check widget validity
        
        probabilitySpin->setEnabled(false);
        while (agents_.size() <= agentIndex) {
            AgentAssignment newAssignment;
            newAssignment.agentIndex = agents_.size();
            agents_.append(newAssignment);
        }
        agents_[agentIndex].runOnceBlocks.removeAll(blockId);
        agents_[agentIndex].randomExecutionBlocks.remove(blockId);
    });
    
    connect(onceRadio, &QRadioButton::toggled, onceRadio, [this, agentIndex, blockId, probabilitySpin](bool checked)
    {
        if (!checked || !probabilitySpin) return; // Only act on checked, not unchecked; check widget validity
        
        probabilitySpin->setEnabled(false);
        while (agents_.size() <= agentIndex) {
            AgentAssignment newAssignment;
            newAssignment.agentIndex = agents_.size();
            agents_.append(newAssignment);
        }
        if (!agents_[agentIndex].runOnceBlocks.contains(blockId)) {
            agents_[agentIndex].runOnceBlocks.append(blockId);
        }
        agents_[agentIndex].randomExecutionBlocks.remove(blockId);
    });
    
    connect(randomRadio, &QRadioButton::toggled, randomRadio, [this, agentIndex, blockId, probabilitySpin](bool checked)
    {
        if (!checked || !probabilitySpin) return; // Only act on checked, not unchecked; check widget validity
        
        probabilitySpin->setEnabled(true);
        while (agents_.size() <= agentIndex) {
            AgentAssignment newAssignment;
            newAssignment.agentIndex = agents_.size();
            agents_.append(newAssignment);
        }
        agents_[agentIndex].runOnceBlocks.removeAll(blockId);
        if (probabilitySpin) { // Double check before accessing
            agents_[agentIndex].randomExecutionBlocks[blockId] = probabilitySpin->value();
        }
    });
    
    connect(probabilitySpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            probabilitySpin, [this, agentIndex, blockId](double value)
    {
        if (agentIndex >= agents_.size()) return; // Safety check
        
        while (agents_.size() <= agentIndex) {
            AgentAssignment newAssignment;
            newAssignment.agentIndex = agents_.size();
            agents_.append(newAssignment);
        }
        // Only update if random mode is actually selected
        if (agents_[agentIndex].randomExecutionBlocks.contains(blockId)) {
            agents_[agentIndex].randomExecutionBlocks[blockId] = value;
        }
    });
    
    blockParamLayout->addRow(executionModeGroup);

    // Add parameters based on block type
    createBlockParametersForAgent(blockConfig->blockId, blockParamLayout, agentIndex);

    // Setup connections to update parameters in agent assignment
    setupParameterUpdateConnections(blockId, agentIndex);

    parametersLayout->addWidget(blockParamGroup);
    parametersLayout->addStretch();

    // Refresh the widget
    parametersContentWidget->update();
}

void BTConfigDialog::showParametersInstruction(QVBoxLayout *parametersLayout, QWidget *parametersContentWidget)
{
    // Clear existing content
    QLayoutItem *item;
    while ((item = parametersLayout->takeAt(0)) != nullptr)
    {
        if (item->widget())
        {
            // Disconnect all signals to prevent dangling connections
            item->widget()->disconnect();
            item->widget()->deleteLater();
        }
        delete item;
    }

    // Add instruction message
    auto instructionLabel = new QLabel("👆 Click on a block from the execution order list to configure its parameters");
    instructionLabel->setStyleSheet(
        "QLabel { color: #6c757d; font-style: italic; font-size: 14px; padding: 30px; "
        "background: rgba(40, 167, 69, 0.05); border: 1px dashed #28a745; border-radius: 8px; "
        "margin: 20px; }");
    instructionLabel->setAlignment(Qt::AlignCenter);
    instructionLabel->setWordWrap(true);
    parametersLayout->addWidget(instructionLabel);
    parametersLayout->addStretch();

    // Refresh the widget
    parametersContentWidget->update();
}

QWidget *BTConfigDialog::createOrderingList(const QStringList &blockOrder, int selectedAgent)
{
    // Create main container for the ordering display
    auto container = new QWidget;
    auto mainLayout = new QVBoxLayout(container);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // Create scroll area for the ordering list
    auto scrollArea = new QScrollArea;
    scrollArea->setWidgetResizable(true);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setStyleSheet(
        "QScrollArea { border: 2px solid #ced4da; border-radius: 6px; background: white; }"
        "QScrollBar:vertical { width: 8px; background: #f0f0f0; border-radius: 4px; }"
        "QScrollBar::handle:vertical { background: #007bff; border-radius: 4px; min-height: 20px; }");

    // Create widget to hold the ordered blocks
    auto contentWidget = new QWidget;
    auto contentLayout = new QVBoxLayout(contentWidget);
    contentLayout->setContentsMargins(10, 10, 10, 10);
    contentLayout->setSpacing(8);
    contentLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

    // Create hidden list widget for compatibility with existing code
    auto orderListWidget = new QListWidget;
    orderListWidget->setObjectName("orderListWidget");
    orderListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    orderListWidget->setDragDropMode(QAbstractItemView::NoDragDrop);
    orderListWidget->setVisible(false);

    // Create visual blocks with arrows
    for (int i = 0; i < blockOrder.size(); ++i)
    {
        const QString &blockId = blockOrder[i];

        // Find block display name
        QString displayName = blockId;
        for (const auto &block : blocks_)
        {
            if (block.blockId == blockId)
            {
                displayName = block.displayName;
                break;
            }
        }

        // Create clickable block item
        auto blockButton = new QPushButton(QString("📋 %1").arg(displayName));
        blockButton->setObjectName(QString("block_%1").arg(i));
        blockButton->setProperty("blockId", blockId);
        blockButton->setProperty("blockIndex", i);

        // Style the block button
        blockButton->setStyleSheet(
            "QPushButton { "
            "  background: white; border: 1px solid #dee2e6; border-radius: 8px; "
            "  padding: 12px 20px; font-size: 14px; font-weight: bold; color: #495057; "
            "  min-width: 200px; text-align: center; "
            "}"
            "QPushButton:hover { "
            "  background: rgba(0, 123, 255, 0.1); border-color: #007bff; color: #007bff; "
            "}"
            "QPushButton:pressed, QPushButton:checked { "
            "  background: #007bff; color: white; border-color: #0056b3; "
            "}");

        blockButton->setCheckable(true);
        blockButton->setMinimumHeight(50);

        // Connect to selection functionality
        connect(blockButton, &QPushButton::clicked, [this, blockButton, orderListWidget, selectedAgent, blockId, i]()
                {
            // Update hidden list widget selection
            if (orderListWidget && i < orderListWidget->count()) {
                orderListWidget->setCurrentRow(i);
                
                // Update visual selection state for all buttons in this container
                updateButtonSelectionState(blockButton);
            } });

        // Add to content layout
        contentLayout->addWidget(blockButton, 0, Qt::AlignHCenter);

        // Add corresponding item to hidden list widget
        auto listItem = new QListWidgetItem(QString("📋 %1").arg(displayName));
        listItem->setData(Qt::UserRole, blockId);
        orderListWidget->addItem(listItem);

        // Add downward arrow (except for the last item)
        if (i < blockOrder.size() - 1)
        {
            auto arrowLabel = new QLabel("↓");
            arrowLabel->setAlignment(Qt::AlignHCenter);
            arrowLabel->setStyleSheet(
                "QLabel { color: #6c757d; font-size: 24px; font-weight: bold; "
                "padding: 5px; margin: 0; }");
            contentLayout->addWidget(arrowLabel, 0, Qt::AlignHCenter);
        }
    }

    contentLayout->addStretch();
    scrollArea->setWidget(contentWidget);
    mainLayout->addWidget(scrollArea);

    // Add hidden list widget to container for compatibility
    container->layout()->addWidget(orderListWidget);

    // Store references for easy access
    container->setProperty("contentLayout", QVariant::fromValue(contentLayout));
    container->setProperty("contentWidget", QVariant::fromValue(contentWidget));
    container->setProperty("scrollArea", QVariant::fromValue(scrollArea));

    return container;
}

void BTConfigDialog::recreateVisualBlocks(QWidget *orderingContainer, const QStringList &blockOrder, int selectedAgent, const QString &selectedBlockId)
{
    auto contentLayout = orderingContainer->property("contentLayout").value<QVBoxLayout *>();
    if (!contentLayout)
        return;

    // Create visual blocks with arrows
    for (int i = 0; i < blockOrder.size(); ++i)
    {
        const QString &blockId = blockOrder[i];

        // Find block display name
        QString displayName = blockId;
        for (const auto &block : blocks_)
        {
            if (block.blockId == blockId)
            {
                displayName = block.displayName;
                break;
            }
        }

        // Create clickable block item
        auto blockButton = new QPushButton(QString("📋 %1").arg(displayName));
        blockButton->setObjectName(QString("block_%1").arg(i));
        blockButton->setProperty("blockId", blockId);
        blockButton->setProperty("blockIndex", i);

        // Style the block button
        blockButton->setStyleSheet(
            "QPushButton { "
            "  background: white; border: 1px solid #dee2e6; border-radius: 8px; "
            "  padding: 12px 20px; font-size: 14px; font-weight: bold; color: #495057; "
            "  min-width: 200px; text-align: center; "
            "}"
            "QPushButton:hover { "
            "  background: rgba(0, 123, 255, 0.1); border-color: #007bff; color: #007bff; "
            "}"
            "QPushButton:pressed, QPushButton:checked { "
            "  background: #007bff; color: white; border-color: #0056b3; "
            "}");

        blockButton->setCheckable(true);
        blockButton->setMinimumHeight(50);

        // Restore selection state if this was the selected block
        if (!selectedBlockId.isEmpty() && blockId == selectedBlockId)
        {
            blockButton->setChecked(true);
        }

        // Get the hidden list widget for synchronization
        auto orderListWidget = orderingContainer->findChild<QListWidget *>("orderListWidget");

        // Connect to selection functionality
        connect(blockButton, &QPushButton::clicked, [this, blockButton, orderListWidget, selectedAgent, blockId, i]()
                {
            // Update hidden list widget selection
            if (orderListWidget && i < orderListWidget->count()) {
                orderListWidget->setCurrentRow(i);
                
                // Update visual selection state for all buttons in this container
                updateButtonSelectionState(blockButton);
            } });

        // Add to content layout
        contentLayout->addWidget(blockButton, 0, Qt::AlignHCenter);

        // Add downward arrow (except for the last item)
        if (i < blockOrder.size() - 1)
        {
            auto arrowLabel = new QLabel("↓");
            arrowLabel->setAlignment(Qt::AlignHCenter);
            arrowLabel->setStyleSheet(
                "QLabel { color: #6c757d; font-size: 24px; font-weight: bold; "
                "padding: 5px; margin: 0; }");
            contentLayout->addWidget(arrowLabel, 0, Qt::AlignHCenter);
        }
    }

    contentLayout->addStretch();
}

void BTConfigDialog::createBlockParametersForAgent(const QString &blockId, QFormLayout *formLayout, int agentIndex)
{
    QString fieldStyle =
        "QSpinBox, QDoubleSpinBox, QComboBox { "
        "  padding: 6px 10px; border: 1px solid #ced4da; border-radius: 4px; "
        "  font-size: 13px; background: white; min-width: 100px; }"
        "QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { "
        "  border-color: #007bff; outline: none; }";

    QString labelStyle =
        "QLabel { font-weight: 500; color: #495057; font-size: 13px; "
        "min-width: 120px; padding: 2px 0; }";

    if (blockId == "EngageRobot")
    {
        // Robot Interaction: IsRobotVisible -> LookAtRobot -> ApproachRobot

        auto detectionDistSpin = new QDoubleSpinBox;
        detectionDistSpin->setRange(1.0, 20.0);
        detectionDistSpin->setValue(5.0);
        detectionDistSpin->setSuffix(" m");
        detectionDistSpin->setDecimals(1);
        detectionDistSpin->setStyleSheet(fieldStyle);
        detectionDistSpin->setToolTip("Maximum distance to detect robots for interaction approach");

        auto detectionLabel = new QLabel("🤖 Robot Detection Distance:");
        detectionLabel->setStyleSheet(labelStyle);
        formLayout->addRow(detectionLabel, detectionDistSpin);

        auto closestDistSpin = new QDoubleSpinBox;
        closestDistSpin->setRange(0.5, 5.0);
        closestDistSpin->setValue(1.5);
        closestDistSpin->setSuffix(" m");
        closestDistSpin->setDecimals(2);
        closestDistSpin->setStyleSheet(fieldStyle);
        closestDistSpin->setToolTip("Desired final distance when approaching the robot");

        auto closestLabel = new QLabel("🎯 Robot Approach Distance:");
        closestLabel->setStyleSheet(labelStyle);
        formLayout->addRow(closestLabel, closestDistSpin);

        auto maxVelSpin = new QDoubleSpinBox;
        maxVelSpin->setRange(0.1, 3.0);
        maxVelSpin->setValue(1.8);
        maxVelSpin->setSuffix(" m/s");
        maxVelSpin->setDecimals(2);
        maxVelSpin->setStyleSheet(fieldStyle);
        maxVelSpin->setToolTip("Maximum speed when moving towards the robot (safety limit)");

        auto maxVelLabel = new QLabel("⚡ Maximum Approach Speed:");
        maxVelLabel->setStyleSheet(labelStyle);
        formLayout->addRow(maxVelLabel, maxVelSpin);

        auto durationSpin = new QDoubleSpinBox;
        durationSpin->setRange(5.0, 120.0);
        durationSpin->setValue(30.0);
        durationSpin->setSuffix(" s");
        durationSpin->setDecimals(1);
        durationSpin->setStyleSheet(fieldStyle);
        durationSpin->setToolTip("Total time to spend in robot interaction behavior");

        auto durLabel = new QLabel("⏱️ Robot Interaction Duration:");
        durLabel->setStyleSheet(labelStyle);
        formLayout->addRow(durLabel, durationSpin);

        // Store widget references for parameter collection
        QString detectionDistKey = QString("%1_%2_detection_distance").arg(blockId).arg(agentIndex);
        QString closestDistKey = QString("%1_%2_closest_dist").arg(blockId).arg(agentIndex);
        QString maxVelKey = QString("%1_%2_max_vel").arg(blockId).arg(agentIndex);
        QString durationKey = QString("%1_%2_duration").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[detectionDistKey] = detectionDistSpin;
        blockConfigWidgets_[closestDistKey] = closestDistSpin;
        blockConfigWidgets_[maxVelKey] = maxVelSpin;
        blockConfigWidgets_[durationKey] = durationSpin;
    }
    else if (blockId == "TalkInteract")
    {
        // Social Interaction: User can choose between checking a specific agent or finding nearest

        // Add interaction mode selection
        auto modeGroup = new QGroupBox("🎭 Interaction Initiation Mode");
        modeGroup->setStyleSheet(
            "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 5px; "
            //"border: 1px solid #dee2e6; border-radius: 4px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
        
        auto modeLayout = new QVBoxLayout;
        
        auto specificTargetRadio = new QRadioButton("🎯 Specific Target - Check for a specific agent");
        specificTargetRadio->setToolTip("Use IsAgentVisible and IsAgentClose to check a specific target agent ID from Conversation Participants");
        
        auto findNearestRadio = new QRadioButton("🔍 Find Nearest - Dynamically find nearest agent");
        findNearestRadio->setToolTip("Use FindNearestAgent to discover nearby agents, then look at them and approach for natural interaction");
        
        // Check if there's a saved mode in parameters, otherwise default to FindNearest
        QString savedMode = "find_nearest";
        QString modeParamKey = QString("%1.interaction_mode").arg(blockId);
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(modeParamKey))
        {
            savedMode = agents_[agentIndex].agentSpecificParams[modeParamKey].toString();
        }
        
        if (savedMode == "specific_target")
        {
            specificTargetRadio->setChecked(true);
        }
        else
        {
            findNearestRadio->setChecked(true);
        }
        
        modeLayout->addWidget(specificTargetRadio);
        modeLayout->addWidget(findNearestRadio);
        modeGroup->setLayout(modeLayout);
        
        formLayout->addRow(modeGroup);
        
        // Store mode selection - hidden QLineEdit with parent so it gets picked up by parameter system
        QString modeKey = QString("%1_%2_interaction_mode").arg(blockId).arg(agentIndex);
        auto modeEdit = new QLineEdit(savedMode, modeGroup);
        modeEdit->setVisible(false);
        blockConfigWidgets_[modeKey] = modeEdit;
        
        // Store radio buttons for proper cleanup
        QString specificRadioKey = QString("%1_%2_specific_radio").arg(blockId).arg(agentIndex);
        QString findNearestRadioKey = QString("%1_%2_findnearest_radio").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[specificRadioKey] = specificTargetRadio;
        blockConfigWidgets_[findNearestRadioKey] = findNearestRadio;

        auto visibilityDistSpin = new QDoubleSpinBox;
        visibilityDistSpin->setRange(2.0, 20.0);
        visibilityDistSpin->setValue(10.0);
        visibilityDistSpin->setSuffix(" m");
        visibilityDistSpin->setDecimals(1);
        visibilityDistSpin->setStyleSheet(fieldStyle);
        visibilityDistSpin->setToolTip("Maximum distance to detect other agents for social interaction");

        auto visLabel = new QLabel("👁️ Social Detection Distance:");
        visLabel->setStyleSheet(labelStyle);
        formLayout->addRow(visLabel, visibilityDistSpin);

        // Proximity threshold parameter for IsAgentClose (only for Specific Target mode)
        auto proximityThresholdSpin = new QDoubleSpinBox;
        proximityThresholdSpin->setRange(0.5, 10.0);
        proximityThresholdSpin->setValue(2.0);
        proximityThresholdSpin->setSuffix(" m");
        proximityThresholdSpin->setDecimals(1);
        proximityThresholdSpin->setStyleSheet(fieldStyle);
        proximityThresholdSpin->setToolTip("Distance considered 'close enough' for social interaction to begin (used in Specific Target mode)");

        auto proximityLabel = new QLabel("📏 Social Interaction Threshold:");
        proximityLabel->setStyleSheet(labelStyle);
        formLayout->addRow(proximityLabel, proximityThresholdSpin);

        // Store widget references for parameter collection
        QString visibilityDistKey = QString("%1_%2_visibility_distance").arg(blockId).arg(agentIndex);
        QString socialDistanceKey = QString("%1_%2_social_distance_threshold").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[visibilityDistKey] = visibilityDistSpin;
        blockConfigWidgets_[socialDistanceKey] = proximityThresholdSpin;

        auto conversationDurationSpin = new QDoubleSpinBox;
        conversationDurationSpin->setRange(10.0, 180.0);
        conversationDurationSpin->setValue(30.0);
        conversationDurationSpin->setSuffix(" s");
        conversationDurationSpin->setDecimals(1);
        conversationDurationSpin->setStyleSheet(fieldStyle);
        conversationDurationSpin->setToolTip("How long the conversation should last once initiated");

        auto convDurLabel = new QLabel("💬 Conversation Duration:");
        convDurLabel->setStyleSheet(labelStyle);
        formLayout->addRow(convDurLabel, conversationDurationSpin);

        // Store conversation duration widget reference
        QString conversationDurationKey = QString("%1_%2_conversation_duration").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[conversationDurationKey] = conversationDurationSpin;

        // Goal ID selection - horizontal layout with clickable buttons
        auto goalIdsWidget = new QWidget;
        auto goalIdsLayout = new QVBoxLayout(goalIdsWidget);
        goalIdsLayout->setContentsMargins(0, 0, 0, 0);
        goalIdsLayout->setSpacing(4);

        // Instructions for goal selection
        auto goalInstrLabel = new QLabel("Select conversation formation goal:");
        goalInstrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 4px;");
        goalIdsLayout->addWidget(goalInstrLabel);

        // Create grid layout for goal buttons (max 3 per row)
        auto goalButtonsLayout = new QGridLayout;
        goalButtonsLayout->setContentsMargins(0, 0, 0, 0);
        goalButtonsLayout->setSpacing(6);

        QButtonGroup *goalButtonGroup = new QButtonGroup(goalIdsWidget);
        QList<QPushButton *> goalButtons;

        // Determine available goals to display
        QList<int> goalsToShow;
        if (availableGoals_.isEmpty())
        {
            // Fallback: create default goals 1-10
            for (int goalId = 1; goalId <= 10; ++goalId)
            {
                goalsToShow.append(goalId);
            }
        }
        else
        {
            // Use actual available goals from actor panel
            goalsToShow = availableGoals_;
        }

        // Check if there's a saved goal_id
        int savedGoalId = -1;
        QString savedGoalKey = QString("%1.goal_id").arg(blockId);
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(savedGoalKey))
        {
            savedGoalId = agents_[agentIndex].agentSpecificParams[savedGoalKey].toInt();
            qDebug() << "Found saved goal_id for" << blockId << "agent" << agentIndex << ":" << savedGoalId;
        }

        // Create clickable goal buttons in grid layout (max 3 per row)
        for (int i = 0; i < goalsToShow.size(); ++i)
        {
            int goalId = goalsToShow[i];
            auto goalButton = new QPushButton(QString("Goal %1").arg(goalId));
            goalButton->setCheckable(true);
            goalButton->setProperty("goalId", goalId);
            goalButton->setStyleSheet(
                "QPushButton { "
                "  padding: 6px 12px; border: 2px solid #ced4da; border-radius: 4px; "
                "  background: white; color: #495057; font-size: 13px; font-weight: 500; "
                "  min-width: 60px; }"
                "QPushButton:hover { "
                "  border-color: #007bff; color: #007bff; }"
                "QPushButton:checked { "
                "  background: #007bff; border-color: #007bff; color: white; font-weight: bold; }"
                "QPushButton:focus { "
                "  outline: none; border-color: #007bff; }");

            // Check if this is the saved goal or default to first goal
            bool shouldBeChecked = false;
            if (savedGoalId > 0)
            {
                shouldBeChecked = (goalId == savedGoalId);
            }
            else
            {
                shouldBeChecked = (i == 0); // Default to first goal
            }
            
            if (shouldBeChecked)
            {
                goalButton->setChecked(true);
            }

            goalButtonGroup->addButton(goalButton, goalId);
            goalButtons.append(goalButton);

            // Add to grid layout (max 3 per row)
            int row = i / 3;
            int col = i % 3;
            goalButtonsLayout->addWidget(goalButton, row, col);
        }

        goalIdsLayout->addLayout(goalButtonsLayout);

        // Hidden line edit to store the selected goal value
        auto goalIdsEdit = new QLineEdit(goalIdsWidget);
        // Use saved goal_id if available
        int initialGoalId;
        if (savedGoalId > 0)
        {
            initialGoalId = savedGoalId;
        }
        else
        {
            initialGoalId = goalsToShow.isEmpty() ? 1 : goalsToShow.first();
        }
        goalIdsEdit->setText(QString::number(initialGoalId));
        goalIdsEdit->setVisible(false);

        QString goalIdsKey = QString("%1_%2_goal_id").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[goalIdsKey] = goalIdsEdit;

        auto goalIdsLabel = new QLabel("🎯 Conversation Formation Goal:");
        goalIdsLabel->setStyleSheet(labelStyle);
        formLayout->addRow(goalIdsLabel, goalIdsWidget);

        // Update function for goal selection using QPointer for safety
        QPointer<QLineEdit> goalIdsEditPtr(goalIdsEdit);
        auto updateGoalIds = [goalIdsEditPtr, goalButtonGroup]()
        {
            if (!goalIdsEditPtr)
                return;
            auto checkedButton = goalButtonGroup->checkedButton();
            if (checkedButton)
            {
                int goalId = checkedButton->property("goalId").toInt();
                goalIdsEditPtr->setText(QString::number(goalId));
            }
        };

        // Connect goal button selection to update function
        connect(goalButtonGroup, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked),
                [updateGoalIds](QAbstractButton *)
                {
                    updateGoalIds();
                });

        // Non-main agent IDs selection - multi-select buttons
        auto agentIdsWidget = new QWidget;
        auto agentIdsLayout = new QVBoxLayout(agentIdsWidget);
        agentIdsLayout->setContentsMargins(0, 0, 0, 0);
        agentIdsLayout->setSpacing(4);

        // Instructions
        auto instrLabel = new QLabel("Choose additional agents to join this conversation (optional):");
        instrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 8px;");
        agentIdsLayout->addWidget(instrLabel);

        // Create grid layout for multi-selection buttons (max 3 per row)
        int totalAgents = getAgentCount();
        auto buttonLayout = new QGridLayout;
        buttonLayout->setContentsMargins(0, 0, 0, 0);
        buttonLayout->setSpacing(6);
        QStringList selectedIds;

        // Store buttons for safe access later
        QList<QPushButton *> participantButtons;

        // Check if there are saved participant selections
        QString savedParticipantsKey = QString("%1.non_main_agent_ids").arg(blockId);
        QStringList savedParticipantIds;
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(savedParticipantsKey))
        {
            QString savedStr = agents_[agentIndex].agentSpecificParams[savedParticipantsKey].toString();
            savedParticipantIds = savedStr.split(",", Qt::SkipEmptyParts);
            // Trim whitespace
            for (QString &id : savedParticipantIds)
            {
                id = id.trimmed();
            }
        }

        int buttonIndex = 0;
        for (int i = 0; i < totalAgents; ++i)
        {
            if (i == agentIndex)
                continue;

            auto button = new QPushButton;
            QString agentLabel = (i < agentNames_.size()) ? QString("Agent %1").arg(i + 1) : QString("Agent %1").arg(i + 1);
            button->setText(agentLabel);
            button->setCheckable(true);

            button->setStyleSheet(
                "QPushButton {"
                "  background-color: #f8f9fa;"
                "  color: #6c757d;"
                "  border: 2px solid #dee2e6;"
                "  border-radius: 6px;"
                "  padding: 6px 12px;"
                "  font-weight: 600;"
                "  font-size: 13px;"
                "  min-width: 70px;"
                "}"
                "QPushButton:hover {"
                "  background-color: #e9ecef;"
                "  border-color: #adb5bd;"
                "  color: #495057;"
                "}"
                "QPushButton:checked {"
                "  background-color: #007bff;"
                "  color: white;"
                "  border-color: #0056b3;"
                "}"
                "QPushButton:checked:hover {"
                "  background-color: #0056b3;"
                "  border-color: #004085;"
                "}"
                "QPushButton:pressed {"
                "  transform: translateY(1px);"
                "}");

            // Check if this agent should be selected based on saved data
            QString agentIdStr = QString::number(i + 1);
            bool shouldBeChecked = false;
            if (!savedParticipantIds.isEmpty())
            {
                // Use saved selection
                shouldBeChecked = savedParticipantIds.contains(agentIdStr);
            }
            else
            {
                // Default: select only the first other agent
                shouldBeChecked = (buttonIndex == 0);
            }
            
            button->setChecked(shouldBeChecked);
            if (button->isChecked())
            {
                selectedIds.append(agentIdStr);
            }

            // Store button for parameter collection
            QString buttonKey = QString("%1_%2_agent_%3_button").arg(blockId).arg(agentIndex).arg(i + 1);
            blockConfigWidgets_[buttonKey] = button;

            // Add to safe list for lambda capture
            participantButtons.append(button);

            // Add to grid layout (max 3 per row)
            int row = buttonIndex / 3;
            int col = buttonIndex % 3;
            buttonLayout->addWidget(button, row, col);
            buttonIndex++;
        }

        agentIdsLayout->addLayout(buttonLayout);

        // Hidden line edit to store the final value
        auto agentIdsEdit = new QLineEdit(agentIdsWidget);
        agentIdsEdit->setText(selectedIds.join(","));
        agentIdsEdit->setVisible(false);

        QString agentIdsKey = QString("%1_%2_non_main_agent_ids").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[agentIdsKey] = agentIdsEdit;

        auto agentIdsLabel = new QLabel("👥 Conversation Participants:");
        agentIdsLabel->setStyleSheet(labelStyle);
        formLayout->addRow(agentIdsLabel, agentIdsWidget);

        // Update function for participant selection
        auto updateAgentIds = [agentIdsEdit, participantButtons, totalAgents, agentIndex]()
        {
            QStringList ids;
            for (int i = 0, buttonIdx = 0; i < totalAgents; ++i)
            {
                if (i == agentIndex)
                    continue; // Skip current agent

                if (buttonIdx < participantButtons.size())
                {
                    QPushButton *btn = participantButtons[buttonIdx];
                    if (btn && btn->isChecked())
                    {
                        ids.append(QString::number(i + 1));
                    }
                    buttonIdx++;
                }
            }
            agentIdsEdit->setText(ids.join(","));
        };

        // Connect all buttons to update function
        for (QPushButton *button : participantButtons)
        {
            if (button)
            {
                connect(button, &QPushButton::toggled, updateAgentIds);
            }
        }
        
        // Connect mode changes to control visibility - Use widget-bound context to auto-disconnect on deletion
        QPointer<QLineEdit> modeEditPtr(modeEdit);
        QPointer<QDoubleSpinBox> proximityThresholdPtr(proximityThresholdSpin);
        QPointer<QLabel> proximityLabelPtr(proximityLabel);
        QPointer<QWidget> agentIdsWidgetPtr(agentIdsWidget);
        QPointer<QLabel> agentIdsLabelPtr(agentIdsLabel);
        
        // Lambda to update visibility based on mode
        auto updateModeVisibility = [agentIdsWidgetPtr, agentIdsLabelPtr](const QString &mode)
        {
            bool isSpecificTarget = (mode == "specific_target");
            
            // Conversation participants only visible in specific target mode
            // (in find_nearest mode, the target is determined by FindNearestAgent)
            if (agentIdsWidgetPtr)
                agentIdsWidgetPtr->setVisible(isSpecificTarget);
            if (agentIdsLabelPtr)
                agentIdsLabelPtr->setVisible(isSpecificTarget);
        };
        
        connect(specificTargetRadio, &QRadioButton::toggled, modeEdit, [modeEditPtr, updateModeVisibility](bool checked)
        {
            if (checked && modeEditPtr)
            {
                modeEditPtr->setText("specific_target");
                updateModeVisibility("specific_target");
            }
        });
        
        connect(findNearestRadio, &QRadioButton::toggled, modeEdit, [modeEditPtr, updateModeVisibility](bool checked)
        {
            if (checked && modeEditPtr)
            {
                modeEditPtr->setText("find_nearest");
                updateModeVisibility("find_nearest");
            }
        });
        
        // Set initial visibility based on the saved/default mode
        updateModeVisibility(savedMode);
    }
    else if (blockId == "RobotAvoidance")
    {
        // Robot Awareness: IsRobotVisible -> IsRobotClose -> StopMovement -> Inverter(IsRobotClose) -> ResumeMovement

        auto detectionDistSpin = new QDoubleSpinBox;
        detectionDistSpin->setRange(1.0, 15.0);
        detectionDistSpin->setValue(5.0);
        detectionDistSpin->setSuffix(" m");
        detectionDistSpin->setDecimals(1);
        detectionDistSpin->setStyleSheet(fieldStyle);
        detectionDistSpin->setToolTip("Distance at which robots are detected for avoidance behavior");

        auto detectionLabel = new QLabel("🤖 Robot Avoidance Detection:");
        detectionLabel->setStyleSheet(labelStyle);
        formLayout->addRow(detectionLabel, detectionDistSpin);

        auto closeThresholdSpin = new QDoubleSpinBox;
        closeThresholdSpin->setRange(0.5, 5.0);
        closeThresholdSpin->setValue(2.0);
        closeThresholdSpin->setSuffix(" m");
        closeThresholdSpin->setDecimals(1);
        closeThresholdSpin->setStyleSheet(fieldStyle);
        closeThresholdSpin->setToolTip("Critical distance - agent will stop if robot gets this close");

        auto closeLabel = new QLabel("⚠️ Emergency Stop Distance:");
        closeLabel->setStyleSheet(labelStyle);
        formLayout->addRow(closeLabel, closeThresholdSpin);

        // Store widget references for parameter collection
        QString detectionDistKey = QString("%1_%2_detection_distance").arg(blockId).arg(agentIndex);
        QString closeThresholdKey = QString("%1_%2_close_threshold").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[detectionDistKey] = detectionDistSpin;
        blockConfigWidgets_[closeThresholdKey] = closeThresholdSpin;
    }
    else if (blockId == "FollowAgent")
    {
        // Follow Agent: Two modes - Find Nearest or Specific Target
        
        // Mode selection using radio buttons
        auto modeGroup = new QGroupBox("Interaction Mode Selection");
        modeGroup->setStyleSheet(
            "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 5px; "
            //"border: 1px solid #dee2e6; border-radius: 4px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
        
        auto modeLayout = new QVBoxLayout;
        
        auto specificTargetRadio = new QRadioButton("🎯 Specific Target - Follow a specific agent");
        specificTargetRadio->setToolTip("Use IsAgentVisible and IsAgentClose to track and follow a specific target agent");
        
        auto findNearestRadio = new QRadioButton("🔍 Find Nearest - Dynamically find nearest agent");
        findNearestRadio->setToolTip("Use FindNearestAgent to discover nearby agents, then follow the nearest one");
        
        // Check if there's a saved mode in parameters, otherwise default to FindNearest
        QString savedMode = "find_nearest";
        QString modeParamKey = QString("%1.interaction_mode").arg(blockId);
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(modeParamKey))
        {
            savedMode = agents_[agentIndex].agentSpecificParams[modeParamKey].toString();
        }
        
        if (savedMode == "specific_target")
        {
            specificTargetRadio->setChecked(true);
        }
        else
        {
            findNearestRadio->setChecked(true);
        }
        
        modeLayout->addWidget(specificTargetRadio);
        modeLayout->addWidget(findNearestRadio);
        modeGroup->setLayout(modeLayout);
        
        formLayout->addRow(modeGroup);
        
        // Store mode selection - hidden QLineEdit with parent so it gets picked up by parameter system
        QString modeKey = QString("%1_%2_interaction_mode").arg(blockId).arg(agentIndex);
        auto modeEdit = new QLineEdit(savedMode, modeGroup);
        modeEdit->setVisible(false);
        blockConfigWidgets_[modeKey] = modeEdit;
        
        // Store radio buttons for proper cleanup
        QString specificRadioKey = QString("%1_%2_specific_radio").arg(blockId).arg(agentIndex);
        QString findNearestRadioKey = QString("%1_%2_findnearest_radio").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[specificRadioKey] = specificTargetRadio;
        blockConfigWidgets_[findNearestRadioKey] = findNearestRadio;

        // Follow Agent: IsAgentVisible -> IsAgentClose -> FollowAgent

        auto visibilityDistSpin = new QDoubleSpinBox;
        visibilityDistSpin->setRange(2.0, 20.0);
        visibilityDistSpin->setValue(10.0);
        visibilityDistSpin->setSuffix(" m");
        visibilityDistSpin->setDecimals(1);
        visibilityDistSpin->setStyleSheet(fieldStyle);
        visibilityDistSpin->setToolTip("Maximum distance to detect the target agent for following");

        auto visLabel = new QLabel("👁️ Target Agent Detection:");
        visLabel->setStyleSheet(labelStyle);
        formLayout->addRow(visLabel, visibilityDistSpin);

        // IsAgentClose threshold parameter
        auto isCloseThresholdSpin = new QDoubleSpinBox;
        isCloseThresholdSpin->setRange(0.5, 8.0);
        isCloseThresholdSpin->setValue(3.0);
        isCloseThresholdSpin->setSuffix(" m");
        isCloseThresholdSpin->setDecimals(1);
        isCloseThresholdSpin->setStyleSheet(fieldStyle);
        isCloseThresholdSpin->setToolTip("Distance threshold to determine when target agent is considered 'close enough' for following behavior to activate");

        auto isCloseLabel = new QLabel("🎯 Agent Close Threshold:");
        isCloseLabel->setStyleSheet(labelStyle);
        formLayout->addRow(isCloseLabel, isCloseThresholdSpin);

        // FollowAgent closest_dist parameter
        auto followingDistSpin = new QDoubleSpinBox;
        followingDistSpin->setRange(0.5, 5.0);
        followingDistSpin->setValue(1.5);
        followingDistSpin->setSuffix(" m");
        followingDistSpin->setDecimals(1);
        followingDistSpin->setStyleSheet(fieldStyle);
        followingDistSpin->setToolTip("Preferred distance to maintain when actively following the target agent");

        auto followingLabel = new QLabel("📏 Following Distance:");
        followingLabel->setStyleSheet(labelStyle);
        formLayout->addRow(followingLabel, followingDistSpin);

        // Maximum velocity parameter for FollowAgent
        auto maxVelSpin = new QDoubleSpinBox;
        maxVelSpin->setRange(0.1, 3.0);
        maxVelSpin->setValue(1.5);
        maxVelSpin->setSuffix(" m/s");
        maxVelSpin->setDecimals(2);
        maxVelSpin->setStyleSheet(fieldStyle);
        maxVelSpin->setToolTip("Maximum speed when following the target agent");

        auto maxVelLabel = new QLabel("⚡ Maximum Follow Speed:");
        maxVelLabel->setStyleSheet(labelStyle);
        formLayout->addRow(maxVelLabel, maxVelSpin);

        // Store widget references for parameter collection
        QString visibilityDistKey = QString("%1_%2_visibility_distance").arg(blockId).arg(agentIndex);
        QString isCloseThresholdKey = QString("%1_%2_is_close_threshold").arg(blockId).arg(agentIndex);
        QString followingDistKey = QString("%1_%2_following_distance").arg(blockId).arg(agentIndex);
        QString maxVelKey = QString("%1_%2_max_vel").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[visibilityDistKey] = visibilityDistSpin;
        blockConfigWidgets_[isCloseThresholdKey] = isCloseThresholdSpin;
        blockConfigWidgets_[followingDistKey] = followingDistSpin;
        blockConfigWidgets_[maxVelKey] = maxVelSpin;

        auto durationSpin = new QDoubleSpinBox;
        durationSpin->setRange(10.0, 300.0);
        durationSpin->setValue(60.0);
        durationSpin->setSuffix(" s");
        durationSpin->setDecimals(1);
        durationSpin->setStyleSheet(fieldStyle);
        durationSpin->setToolTip("Maximum time to spend following the target agent");

        auto durLabel = new QLabel("⏱️ Maximum Follow Time:");
        durLabel->setStyleSheet(labelStyle);
        formLayout->addRow(durLabel, durationSpin);

        // Store duration widget reference
        QString followDurationKey = QString("%1_%2_duration").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[followDurationKey] = durationSpin;

        // Target agent selection - vertical layout with clickable buttons
        auto targetAgentWidget = new QWidget;
        auto targetAgentLayout = new QVBoxLayout(targetAgentWidget);
        targetAgentLayout->setContentsMargins(0, 0, 0, 0);
        targetAgentLayout->setSpacing(4);

        // Instructions for target agent selection
        auto targetInstrLabel = new QLabel("Select target agent to follow:");
        targetInstrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 4px;");
        targetAgentLayout->addWidget(targetInstrLabel);

        // Create grid layout for target agent buttons (max 3 per row)
        int totalAgents = getAgentCount();
        auto targetButtonsLayout = new QGridLayout;
        targetButtonsLayout->setContentsMargins(0, 0, 0, 0);
        targetButtonsLayout->setSpacing(6);

        QButtonGroup *targetButtonGroup = new QButtonGroup(targetAgentWidget);
        QList<QPushButton *> targetButtons;

        int buttonIndex = 0;
        for (int i = 0; i < totalAgents; ++i)
        {
            if (i == agentIndex)
                continue; // Skip current agent

            auto targetButton = new QPushButton(QString("Agent %1").arg(i + 1));
            targetButton->setCheckable(true);
            targetButton->setProperty("agentId", i + 1);
            targetButton->setStyleSheet(
                "QPushButton { "
                "  padding: 6px 12px; border: 2px solid #ced4da; border-radius: 4px; "
                "  background: white; color: #495057; font-size: 13px; font-weight: 500; "
                "  min-width: 70px; }"
                "QPushButton:hover { "
                "  border-color: #007bff; color: #007bff; }"
                "QPushButton:checked { "
                "  background: #007bff; border-color: #007bff; color: white; font-weight: bold; }"
                "QPushButton:focus { "
                "  outline: none; border-color: #007bff; }");

            // Select default agent (first available that's not current agent)
            if (buttonIndex == 0)
            {
                targetButton->setChecked(true);
            }

            targetButtonGroup->addButton(targetButton, i + 1);
            targetButtons.append(targetButton);

            // Add to grid layout (max 3 per row)
            int row = buttonIndex / 3;
            int col = buttonIndex % 3;
            targetButtonsLayout->addWidget(targetButton, row, col);
            buttonIndex++;
        }

        targetAgentLayout->addLayout(targetButtonsLayout);

        // Hidden spin box to store the selected target agent value - needs parent for parameter system
        auto targetAgentSpin = new QSpinBox(targetAgentWidget);
        targetAgentSpin->setRange(1, 100);
        targetAgentSpin->setValue(agentIndex == 0 ? 2 : 1); // Default to different agent
        targetAgentSpin->setStyleSheet(fieldStyle);
        targetAgentSpin->setToolTip("Choose which agent to follow (by agent ID number)");
        targetAgentSpin->setVisible(false);

        // Store widget for parameter collection
        QString targetWidgetKey = QString("%1_%2_target_agent_id").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[targetWidgetKey] = targetAgentSpin;

        auto targetLabel = new QLabel("🎯 Agent to Follow:");
        targetLabel->setStyleSheet(labelStyle);
        formLayout->addRow(targetLabel, targetAgentWidget);

        // Update function for target agent selection
        auto updateTargetAgent = [targetAgentSpin, targetButtonGroup]()
        {
            auto checkedButton = targetButtonGroup->checkedButton();
            if (checkedButton)
            {
                int agentId = checkedButton->property("agentId").toInt();
                targetAgentSpin->setValue(agentId);
            }
        };

        // Connect all target buttons to update function
        for (QPushButton *button : targetButtons)
        {
            if (button)
            {
                connect(button, &QPushButton::toggled, updateTargetAgent);
            }
        }
        
        // Connect mode changes to control visibility
        QPointer<QLineEdit> modeEditPtr(modeEdit);
        QPointer<QWidget> targetAgentWidgetPtr(targetAgentWidget);
        QPointer<QLabel> targetLabelPtr(targetLabel);
        
        // Lambda to update visibility based on mode
        auto updateModeVisibility = [targetAgentWidgetPtr, targetLabelPtr](const QString &mode)
        {
            bool isSpecificTarget = (mode == "specific_target");
            
            // Target agent selection only visible in specific target mode
            // (in find_nearest mode, the target is determined by FindNearestAgent)
            if (targetAgentWidgetPtr)
                targetAgentWidgetPtr->setVisible(isSpecificTarget);
            if (targetLabelPtr)
                targetLabelPtr->setVisible(isSpecificTarget);
        };
        
        connect(specificTargetRadio, &QRadioButton::toggled, modeEdit, [modeEditPtr, updateModeVisibility](bool checked)
        {
            if (checked && modeEditPtr)
            {
                modeEditPtr->setText("specific_target");
                updateModeVisibility("specific_target");
            }
        });
        
        connect(findNearestRadio, &QRadioButton::toggled, modeEdit, [modeEditPtr, updateModeVisibility](bool checked)
        {
            if (checked && modeEditPtr)
            {
                modeEditPtr->setText("find_nearest");
                updateModeVisibility("find_nearest");
            }
        });
        
        // Set initial visibility based on the saved/default mode
        updateModeVisibility(savedMode);

        // Store duration spin box reference
        QString durationKey = QString("%1_%2_duration").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[durationKey] = durationSpin;
    }
    else if (blockId == "BlockingBehavior")
    {
        // Robot Blocking: IsRobotVisible -> LookAtRobot -> BlockRobot

        auto detectionDistSpin = new QDoubleSpinBox;
        detectionDistSpin->setRange(2.0, 20.0);
        detectionDistSpin->setValue(8.0);
        detectionDistSpin->setSuffix(" m");
        detectionDistSpin->setDecimals(1);
        detectionDistSpin->setStyleSheet(fieldStyle);
        detectionDistSpin->setToolTip("Maximum distance to detect the robot for blocking behavior");

        auto detectionLabel = new QLabel("👁️ Robot Detection Distance:");
        detectionLabel->setStyleSheet(labelStyle);
        formLayout->addRow(detectionLabel, detectionDistSpin);

        // Front distance parameter for BlockRobot
        auto frontDistSpin = new QDoubleSpinBox;
        frontDistSpin->setRange(0.5, 5.0);
        frontDistSpin->setValue(2.0);
        frontDistSpin->setSuffix(" m");
        frontDistSpin->setDecimals(1);
        frontDistSpin->setStyleSheet(fieldStyle);
        frontDistSpin->setToolTip("Distance in front of agent used for blocking the robot's path");

        auto frontDistLabel = new QLabel("🚧 Blocking Distance:");
        frontDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(frontDistLabel, frontDistSpin);

        // Duration parameter for BlockRobot
        auto blockDurationSpin = new QDoubleSpinBox;
        blockDurationSpin->setRange(5.0, 120.0);
        blockDurationSpin->setValue(30.0);
        blockDurationSpin->setSuffix(" s");
        blockDurationSpin->setDecimals(1);
        blockDurationSpin->setStyleSheet(fieldStyle);
        blockDurationSpin->setToolTip("Duration for which the agent will block the robot's path");

        auto blockDurLabel = new QLabel("⏱️ Blocking Duration:");
        blockDurLabel->setStyleSheet(labelStyle);
        formLayout->addRow(blockDurLabel, blockDurationSpin);

        // Store widget references for parameter collection
        QString detectionDistKey = QString("%1_%2_detection_distance").arg(blockId).arg(agentIndex);
        QString frontDistKey = QString("%1_%2_front_dist").arg(blockId).arg(agentIndex);
        QString blockDurationKey = QString("%1_%2_duration").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[detectionDistKey] = detectionDistSpin;
        blockConfigWidgets_[frontDistKey] = frontDistSpin;
        blockConfigWidgets_[blockDurationKey] = blockDurationSpin;
    }
    else if (blockId == "GroupFormation")
    {
        // Group Coordination: FindNearestAgent -> SetGroupWalk

        // Walk duration parameter for SetGroupWalk
        auto walkDurationSpin = new QDoubleSpinBox;
        walkDurationSpin->setRange(10.0, 600.0);
        walkDurationSpin->setValue(30.0);
        walkDurationSpin->setSuffix(" s");
        walkDurationSpin->setDecimals(1);
        walkDurationSpin->setStyleSheet(fieldStyle);
        walkDurationSpin->setToolTip("Duration for group walking behavior (0 = indefinite)");

        auto walkDurLabel = new QLabel("⏱️ Group Walk Duration:");
        walkDurLabel->setStyleSheet(labelStyle);
        formLayout->addRow(walkDurLabel, walkDurationSpin);

        // Store widget references for parameter collection
        QString walkDurationKey = QString("%1_%2_duration").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[walkDurationKey] = walkDurationSpin;
    }
    else if (blockId == "SpeechDetection")
    {
        // Audio-Triggered Social: IsAnyoneSpeaking -> ApproachAgent -> ConversationFormation

        auto speechDistSpin = new QDoubleSpinBox;
        speechDistSpin->setRange(2.0, 20.0);
        speechDistSpin->setValue(8.0);
        speechDistSpin->setSuffix(" m");
        speechDistSpin->setDecimals(1);
        speechDistSpin->setStyleSheet(fieldStyle);
        speechDistSpin->setToolTip("Maximum distance to detect speech for social interaction");

        auto speechDistLabel = new QLabel("👂 Speech Detection Distance:");
        speechDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(speechDistLabel, speechDistSpin);

        // Approach distance parameter for ApproachAgent
        auto approachDistSpin = new QDoubleSpinBox;
        approachDistSpin->setRange(0.5, 5.0);
        approachDistSpin->setValue(1.5);
        approachDistSpin->setSuffix(" m");
        approachDistSpin->setDecimals(1);
        approachDistSpin->setStyleSheet(fieldStyle);
        approachDistSpin->setToolTip("Distance to maintain when approaching the speaking agent");

        auto approachDistLabel = new QLabel("🚶 Approach Distance:");
        approachDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(approachDistLabel, approachDistSpin);

        // Maximum velocity parameter for ApproachAgent
        auto approachVelSpin = new QDoubleSpinBox;
        approachVelSpin->setRange(0.1, 3.0);
        approachVelSpin->setValue(1.2);
        approachVelSpin->setSuffix(" m/s");
        approachVelSpin->setDecimals(2);
        approachVelSpin->setStyleSheet(fieldStyle);
        approachVelSpin->setToolTip("Maximum speed when approaching the speaking agent");

        auto approachVelLabel = new QLabel("⚡ Approach Speed:");
        approachVelLabel->setStyleSheet(labelStyle);
        formLayout->addRow(approachVelLabel, approachVelSpin);

        // Conversation duration parameter for ConversationFormation
        auto convDurationSpin = new QDoubleSpinBox;
        convDurationSpin->setRange(10.0, 180.0);
        convDurationSpin->setValue(45.0);
        convDurationSpin->setSuffix(" s");
        convDurationSpin->setDecimals(1);
        convDurationSpin->setStyleSheet(fieldStyle);
        convDurationSpin->setToolTip("Duration of conversation after joining the speaking agent");

        auto convDurationLabel = new QLabel("💬 Conversation Duration:");
        convDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(convDurationLabel, convDurationSpin);

        // Duration for IsAnyoneSpeaking node
        auto speakingDurationSpin = new QDoubleSpinBox;
        speakingDurationSpin->setRange(1.0, 30.0);
        speakingDurationSpin->setValue(5.0);
        speakingDurationSpin->setSuffix(" s");
        speakingDurationSpin->setDecimals(1);
        speakingDurationSpin->setStyleSheet(fieldStyle);
        speakingDurationSpin->setToolTip("Duration for speech detection check");

        auto speakingDurationLabel = new QLabel("⏱️ Speech Detection Duration:");
        speakingDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(speakingDurationLabel, speakingDurationSpin);

        // Duration for ApproachAgent node
        auto approachDurationSpin = new QDoubleSpinBox;
        approachDurationSpin->setRange(5.0, 60.0);
        approachDurationSpin->setValue(15.0);
        approachDurationSpin->setSuffix(" s");
        approachDurationSpin->setDecimals(1);
        approachDurationSpin->setStyleSheet(fieldStyle);
        approachDurationSpin->setToolTip("Maximum duration for approaching the speaking agent");

        auto approachDurationLabel = new QLabel("⏱️ Approach Duration:");
        approachDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(approachDurationLabel, approachDurationSpin);

        // Store widget references for parameter collection
        QString speechDistKey = QString("%1_%2_speech_distance").arg(blockId).arg(agentIndex);
        QString approachDistKey = QString("%1_%2_approach_distance").arg(blockId).arg(agentIndex);
        QString approachVelKey = QString("%1_%2_approach_velocity").arg(blockId).arg(agentIndex);
        QString convDurationKey = QString("%1_%2_conversation_duration").arg(blockId).arg(agentIndex);
        QString speakingDurationKey = QString("%1_%2_speaking_duration").arg(blockId).arg(agentIndex);
        QString approachDurationKey = QString("%1_%2_approach_duration").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[speechDistKey] = speechDistSpin;
        blockConfigWidgets_[approachDistKey] = approachDistSpin;
        blockConfigWidgets_[approachVelKey] = approachVelSpin;
        blockConfigWidgets_[convDurationKey] = convDurationSpin;
        blockConfigWidgets_[speakingDurationKey] = speakingDurationSpin;
        blockConfigWidgets_[approachDurationKey] = approachDurationSpin;
        
        // Goal ID selection for conversation formation
        auto goalIdsWidget = new QWidget;
        auto goalIdsLayout = new QVBoxLayout(goalIdsWidget);
        goalIdsLayout->setContentsMargins(0, 0, 0, 0);
        goalIdsLayout->setSpacing(4);

        // Instructions for goal selection
        auto goalInstrLabel = new QLabel("Select conversation formation goal:");
        goalInstrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 4px;");
        goalIdsLayout->addWidget(goalInstrLabel);

        // Default goals
        QList<int> goalsToShow = {1, 2, 3, 4, 5, 6};

        // Create grid layout for goal buttons (max 3 per row)
        auto goalButtonsLayout = new QGridLayout;
        goalButtonsLayout->setContentsMargins(0, 0, 0, 0);
        goalButtonsLayout->setSpacing(6);

        QButtonGroup *goalButtonGroup = new QButtonGroup(goalIdsWidget);
        QList<QPushButton *> goalButtons;

        int buttonIndex = 0;
        for (int goalId : goalsToShow)
        {
            auto goalButton = new QPushButton(QString("Goal %1").arg(goalId));
            goalButton->setCheckable(true);
            goalButton->setProperty("goalId", goalId);
            goalButton->setStyleSheet(
                "QPushButton {"
                "  background-color: #f8f9fa;"
                "  color: #6c757d;"
                "  border: 2px solid #dee2e6;"
                "  border-radius: 6px;"
                "  padding: 6px 12px;"
                "  font-weight: 600;"
                "  font-size: 13px;"
                "  min-width: 70px;"
                "}"
                "QPushButton:hover {"
                "  background-color: #e9ecef;"
                "  border-color: #adb5bd;"
                "  color: #495057;"
                "}"
                "QPushButton:checked {"
                "  background-color: #28a745;"
                "  color: white;"
                "  border-color: #1e7e34;"
                "}"
                "QPushButton:checked:hover {"
                "  background-color: #1e7e34;"
                "  border-color: #145523;"
                "}"
                "QPushButton:pressed {"
                "  transform: translateY(1px);"
                "}");

            // Select first goal by default
            if (buttonIndex == 0)
            {
                goalButton->setChecked(true);
            }

            goalButtonGroup->addButton(goalButton, goalId);
            goalButtons.append(goalButton);

            // Add to grid layout (max 3 per row)
            int row = buttonIndex / 3;
            int col = buttonIndex % 3;
            goalButtonsLayout->addWidget(goalButton, row, col);
            buttonIndex++;
        }

        goalIdsLayout->addLayout(goalButtonsLayout);

        // Hidden line edit to store the selected goal value
        auto goalIdsEdit = new QLineEdit(goalIdsWidget);
        int defaultGoalId = goalsToShow.isEmpty() ? 1 : goalsToShow.first();
        goalIdsEdit->setText(QString::number(defaultGoalId));
        goalIdsEdit->setVisible(false);

        QString goalIdsKey = QString("%1_%2_goal_id").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[goalIdsKey] = goalIdsEdit;

        auto goalIdsLabel = new QLabel("🎯 Conversation Formation Goal:");
        goalIdsLabel->setStyleSheet(labelStyle);
        formLayout->addRow(goalIdsLabel, goalIdsWidget);

        // Update function for goal selection using QPointer for safety
        QPointer<QLineEdit> goalIdsEditPtr(goalIdsEdit);
        auto updateGoalIds = [goalIdsEditPtr, goalButtonGroup]()
        {
            if (!goalIdsEditPtr)
                return;
            auto checkedButton = goalButtonGroup->checkedButton();
            if (checkedButton)
            {
                int goalId = checkedButton->property("goalId").toInt();
                goalIdsEditPtr->setText(QString::number(goalId));
            }
        };

        // Connect goal button selection to update function
        connect(goalButtonGroup, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked),
                [updateGoalIds](QAbstractButton *)
                {
                    updateGoalIds();
                });
    }
    else if (blockId == "AttentionSeeking")
    {
        // Attention Response: Two modes - Agent Attention or Robot Attention
        
        // Mode selection using radio buttons
        auto modeGroup = new QGroupBox("Attention Mode Selection");
        modeGroup->setStyleSheet(
            "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 5px; "
            //"border: 1px solid #dee2e6; border-radius: 4px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
        
        auto modeLayout = new QVBoxLayout;
        
        auto agentAttentionRadio = new QRadioButton("👥 Agent Attention - Respond to other agents looking");
        agentAttentionRadio->setToolTip("Use IsAnyoneLookingAtMe to detect when other agents are looking, then look back at them");
        
        auto robotAttentionRadio = new QRadioButton("🤖 Robot Attention - Respond to robot looking");
        robotAttentionRadio->setToolTip("Use IsRobotFacingAgent to detect when robot is looking, then look at the robot");
        
        // Check if there's a saved mode in parameters, otherwise default to agent_attention
        QString savedMode = "agent_attention";
        QString modeParamKey = QString("%1.attention_mode").arg(blockId);
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(modeParamKey))
        {
            savedMode = agents_[agentIndex].agentSpecificParams[modeParamKey].toString();
        }
        
        if (savedMode == "robot_attention")
        {
            robotAttentionRadio->setChecked(true);
        }
        else
        {
            agentAttentionRadio->setChecked(true);
        }
        
        modeLayout->addWidget(agentAttentionRadio);
        modeLayout->addWidget(robotAttentionRadio);
        modeGroup->setLayout(modeLayout);
        
        formLayout->addRow(modeGroup);
        
        // Store mode selection - hidden QLineEdit with parent so it gets picked up by parameter system
        QString modeKey = QString("%1_%2_attention_mode").arg(blockId).arg(agentIndex);
        auto modeEdit = new QLineEdit(savedMode, modeGroup);
        modeEdit->setVisible(false);
        blockConfigWidgets_[modeKey] = modeEdit;
        
        // Store radio buttons for proper cleanup
        QString agentRadioKey = QString("%1_%2_agent_radio").arg(blockId).arg(agentIndex);
        QString robotRadioKey = QString("%1_%2_robot_radio").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[agentRadioKey] = agentAttentionRadio;
        blockConfigWidgets_[robotRadioKey] = robotAttentionRadio;

        // Attention Response: IsAnyoneLookingAtMe/IsRobotFacingAgent -> LookAtAgent/LookAtRobot -> SaySomething

        auto attentionDistSpin = new QDoubleSpinBox;
        attentionDistSpin->setRange(2.0, 15.0);
        attentionDistSpin->setValue(6.0);
        attentionDistSpin->setSuffix(" m");
        attentionDistSpin->setDecimals(1);
        attentionDistSpin->setStyleSheet(fieldStyle);
        attentionDistSpin->setToolTip("Maximum distance to detect when being looked at");

        auto attentionDistLabel = new QLabel("👁️ Attention Detection Distance:");
        attentionDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(attentionDistLabel, attentionDistSpin);

        // Duration for IsAnyoneLookingAtMe
        auto lookingDurationSpin = new QDoubleSpinBox;
        lookingDurationSpin->setRange(1.0, 30.0);
        lookingDurationSpin->setValue(5.0);
        lookingDurationSpin->setSuffix(" s");
        lookingDurationSpin->setDecimals(1);
        lookingDurationSpin->setStyleSheet(fieldStyle);
        lookingDurationSpin->setToolTip("Duration for attention detection check (only for agent attention mode)");

        auto lookingDurationLabel = new QLabel("⏱️ Attention Detection Duration:");
        lookingDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(lookingDurationLabel, lookingDurationSpin);

        // Response message parameter for SaySomething
        auto responseMessageEdit = new QLineEdit;
        responseMessageEdit->setText("I see you looking at me!");
        responseMessageEdit->setStyleSheet(fieldStyle);
        responseMessageEdit->setToolTip("Message to say when detected being looked at");

        auto responseMessageLabel = new QLabel("💭 Response Message:");
        responseMessageLabel->setStyleSheet(labelStyle);
        formLayout->addRow(responseMessageLabel, responseMessageEdit);

        // Store widget references for parameter collection
        QString attentionDistKey = QString("%1_%2_attention_distance").arg(blockId).arg(agentIndex);
        QString lookingDurationKey = QString("%1_%2_looking_duration").arg(blockId).arg(agentIndex);
        QString responseMessageKey = QString("%1_%2_response_message").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[attentionDistKey] = attentionDistSpin;
        blockConfigWidgets_[lookingDurationKey] = lookingDurationSpin;
        blockConfigWidgets_[responseMessageKey] = responseMessageEdit;
        
        // Connect mode selection to update the hidden field
        QPointer<QLineEdit> modeEditPtr(modeEdit);
        QPointer<QRadioButton> agentAttentionPtr(agentAttentionRadio);
        QPointer<QRadioButton> robotAttentionPtr(robotAttentionRadio);
        
        auto updateMode = [modeEditPtr, agentAttentionPtr, robotAttentionPtr]()
        {
            if (!modeEditPtr || !agentAttentionPtr || !robotAttentionPtr)
                return;
            if (agentAttentionPtr->isChecked())
            {
                modeEditPtr->setText("agent_attention");
            }
            else if (robotAttentionPtr->isChecked())
            {
                modeEditPtr->setText("robot_attention");
            }
        };
        
        connect(agentAttentionRadio, &QRadioButton::toggled, agentAttentionRadio, updateMode);
        connect(robotAttentionRadio, &QRadioButton::toggled, robotAttentionRadio, updateMode);
    }
    else if (blockId == "GreetingInitiator")
    {
        // Ensure agents_ array is properly sized
        while (agents_.size() <= agentIndex)
        {
            AgentAssignment newAgent;
            newAgent.agentIndex = agents_.size();
            newAgent.agentName = QString("Agent_%1").arg(agents_.size() + 1);
            newAgent.behaviorType = "Regular";
            newAgent.randomizeOrder = false;
            agents_.append(newAgent);
        }
        
        // Friendly Greeter: Three modes - Greet Robot, Greet Specific Agent, or Greet Nearest Agent
        
        // Mode selection using radio buttons
        auto modeGroup = new QGroupBox("Greeting Target Selection");
        modeGroup->setStyleSheet(
            "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 5px; "
            // "border: 1px solid #dee2e6; border-radius: 4px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
        
        auto modeLayout = new QVBoxLayout;
        
        auto greetRobotRadio = new QRadioButton("🤖 Greet Robot - Detect and greet the robot");
        greetRobotRadio->setToolTip("Use IsRobotVisible to detect robot and greet it with a message");
        
        auto specificAgentRadio = new QRadioButton("🎯 Greet Specific Agent - Greet a specific agent");
        specificAgentRadio->setToolTip("Greet a specific agent when they come into view");
        
        auto nearestAgentRadio = new QRadioButton("🔍 Greet Nearest Agent - Greet whoever is closest");
        nearestAgentRadio->setToolTip("Use FindNearestAgent to discover and greet whoever is closest");
        
        // Check if there's a saved mode in parameters, otherwise default to specific_agent
        QString savedMode = "specific_agent";
        QString modeParamKey = QString("%1.greeting_mode").arg(blockId);
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(modeParamKey))
        {
            savedMode = agents_[agentIndex].agentSpecificParams[modeParamKey].toString();
        }
        
        // Map old mode names to new ones for backward compatibility
        if (savedMode == "specific_target")
        {
            savedMode = "specific_agent";
        }
        else if (savedMode == "find_nearest")
        {
            savedMode = "nearest_agent";
        }
        
        if (savedMode == "greet_robot")
        {
            greetRobotRadio->setChecked(true);
        }
        else if (savedMode == "nearest_agent")
        {
            nearestAgentRadio->setChecked(true);
        }
        else
        {
            specificAgentRadio->setChecked(true);
        }
        
        modeLayout->addWidget(greetRobotRadio);
        modeLayout->addWidget(specificAgentRadio);
        modeLayout->addWidget(nearestAgentRadio);
        modeGroup->setLayout(modeLayout);
        
        formLayout->addRow(modeGroup);
        
        // Store mode selection - hidden QLineEdit with parent so it gets picked up by parameter system
        QString modeKey = QString("%1_%2_greeting_mode").arg(blockId).arg(agentIndex);
        auto modeEdit = new QLineEdit(savedMode, modeGroup);
        modeEdit->setVisible(false);
        blockConfigWidgets_[modeKey] = modeEdit;
        
        // Store radio buttons for proper cleanup
        QString robotRadioKey = QString("%1_%2_robot_radio").arg(blockId).arg(agentIndex);
        QString specificRadioKey = QString("%1_%2_specific_radio").arg(blockId).arg(agentIndex);
        QString nearestRadioKey = QString("%1_%2_nearest_radio").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[robotRadioKey] = greetRobotRadio;
        blockConfigWidgets_[specificRadioKey] = specificAgentRadio;
        blockConfigWidgets_[nearestRadioKey] = nearestAgentRadio;

        // Friendly Greeter: IsRobotVisible -> SaySomething (Robot mode)
        //                or IsAgentVisible -> SaySomething (Agent mode)
        //                or FindNearestAgent -> IsAgentVisible -> SaySomething (Nearest Agent mode)

        auto detectionDistSpin = new QDoubleSpinBox;
        detectionDistSpin->setRange(2.0, 15.0);
        detectionDistSpin->setValue(6.0);
        detectionDistSpin->setSuffix(" m");
        detectionDistSpin->setDecimals(1);
        detectionDistSpin->setStyleSheet(fieldStyle);
        detectionDistSpin->setToolTip("Maximum distance to detect robot or agents for greeting");

        auto detectionDistLabel = new QLabel("👀 Detection Distance:");
        detectionDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(detectionDistLabel, detectionDistSpin);

        // Greeting message parameter for SaySomething
        auto greetingMessageEdit = new QLineEdit;
        greetingMessageEdit->setText("Hello there!");
        greetingMessageEdit->setStyleSheet(fieldStyle);
        greetingMessageEdit->setToolTip("Message to say when greeting robot or other agents");

        auto greetingMessageLabel = new QLabel("👋 Greeting Message:");
        greetingMessageLabel->setStyleSheet(labelStyle);
        formLayout->addRow(greetingMessageLabel, greetingMessageEdit);

        // Target Agent ID selection for which agent to greet (only visible in specific_agent mode)
        auto greetTargetAgentWidget = new QWidget;
        auto greetTargetAgentLayout = new QVBoxLayout(greetTargetAgentWidget);
        greetTargetAgentLayout->setContentsMargins(0, 0, 0, 0);
        greetTargetAgentLayout->setSpacing(4);

        // Instructions for target agent selection
        auto greetTargetInstrLabel = new QLabel("Select specific agent to greet when they come into view:");
        greetTargetInstrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 4px;");
        greetTargetAgentLayout->addWidget(greetTargetInstrLabel);

        // Create grid layout for target agent buttons (max 3 per row)
        int totalAgents = getAgentCount();
        auto greetTargetButtonsLayout = new QGridLayout;
        greetTargetButtonsLayout->setContentsMargins(0, 0, 0, 0);
        greetTargetButtonsLayout->setSpacing(6);

        auto greetTargetButtonGroup = new QButtonGroup(this);
        QList<QPushButton *> greetTargetButtons;

        // Create buttons for each available other agent
        int buttonIndex = 0;
        for (int i = 0; i < totalAgents; ++i)
        {
            if (i == agentIndex)
                continue;

            auto greetTargetButton = new QPushButton(QString("Agent %1").arg(i + 1));
            greetTargetButton->setCheckable(true);
            greetTargetButton->setProperty("agentId", i + 1);
            greetTargetButton->setStyleSheet(
                "QPushButton { "
                "  padding: 6px 12px; border: 2px solid #ced4da; border-radius: 4px; "
                "  background: white; color: #495057; font-size: 13px; font-weight: 500; "
                "  min-width: 70px; }"
                "QPushButton:hover { "
                "  border-color: #ffc107; color: #ffc107; }"
                "QPushButton:checked { "
                "  background: #ffc107; border-color: #ffc107; color: #212529; font-weight: bold; }"
                "QPushButton:focus { "
                "  outline: none; border-color: #ffc107; }");

            // Select default agent
            if (buttonIndex == 0)
            {
                greetTargetButton->setChecked(true);
            }

            greetTargetButtonGroup->addButton(greetTargetButton, i + 1);
            greetTargetButtons.append(greetTargetButton);

            // Add to grid layout (max 3 per row)
            int row = buttonIndex / 3;
            int col = buttonIndex % 3;
            greetTargetButtonsLayout->addWidget(greetTargetButton, row, col);
            buttonIndex++;
        }

        greetTargetAgentLayout->addLayout(greetTargetButtonsLayout);

        // Hidden spin box to store the selected target agent value
        auto greetTargetAgentSpin = new QSpinBox(greetTargetAgentWidget);
        greetTargetAgentSpin->setRange(1, 100);
        greetTargetAgentSpin->setValue(agentIndex == 0 ? 2 : 1); // Default to different agent
        greetTargetAgentSpin->setStyleSheet(fieldStyle);
        greetTargetAgentSpin->setToolTip("Choose which agent to greet when they come into view");
        greetTargetAgentSpin->setVisible(false);

        auto greetTargetLabel = new QLabel("🎯 Agent to Greet:");
        greetTargetLabel->setStyleSheet(labelStyle);
        formLayout->addRow(greetTargetLabel, greetTargetAgentWidget);

        // Connect button group to update hidden spinbox
        connect(greetTargetButtonGroup, QOverload<int>::of(&QButtonGroup::idClicked),
                [greetTargetAgentSpin](int agentId)
                {
                    greetTargetAgentSpin->setValue(agentId);
                    qDebug() << "GreetingInitiator target agent set to:" << agentId;
                });

        // Store widget references for parameter collection
        QString detectionDistKey = QString("%1_%2_detection_distance").arg(blockId).arg(agentIndex);
        QString greetingMessageKey = QString("%1_%2_greeting_message").arg(blockId).arg(agentIndex);
        QString greetTargetAgentKey = QString("%1_%2_target_agent_id").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[detectionDistKey] = detectionDistSpin;
        blockConfigWidgets_[greetingMessageKey] = greetingMessageEdit;
        blockConfigWidgets_[greetTargetAgentKey] = greetTargetAgentSpin;
        
        // Control visibility based on mode
        // Target agent selection only visible in specific_agent mode
        QPointer<QWidget> greetTargetAgentWidgetPtr(greetTargetAgentWidget);
        QPointer<QLabel> greetTargetLabelPtr(greetTargetLabel);
        QPointer<QRadioButton> specificAgentRadioPtr(specificAgentRadio);
        
        auto updateGreetingVisibility = [greetTargetAgentWidgetPtr, greetTargetLabelPtr, specificAgentRadioPtr]()
        {
            if (!greetTargetAgentWidgetPtr || !greetTargetLabelPtr || !specificAgentRadioPtr)
                return;
            bool shouldShow = specificAgentRadioPtr->isChecked();
            greetTargetAgentWidgetPtr->setVisible(shouldShow);
            greetTargetLabelPtr->setVisible(shouldShow);
        };
        
        // Set initial visibility
        updateGreetingVisibility();
        
        // Connect mode selection to update the hidden field and visibility
        QPointer<QLineEdit> modeEditPtr(modeEdit);
        QPointer<QRadioButton> greetRobotPtr(greetRobotRadio);
        QPointer<QRadioButton> specificAgentPtr(specificAgentRadio);
        QPointer<QRadioButton> nearestAgentPtr(nearestAgentRadio);
        
        auto updateGreetingMode = [this, agentIndex, blockId, modeEditPtr, greetRobotPtr, specificAgentPtr, nearestAgentPtr, updateGreetingVisibility]()
        {
            if (!modeEditPtr || !greetRobotPtr || !specificAgentPtr || !nearestAgentPtr)
                return;
            
            QString newMode;
            if (greetRobotPtr->isChecked())
            {
                newMode = "greet_robot";
            }
            else if (specificAgentPtr->isChecked())
            {
                newMode = "specific_agent";
            }
            else if (nearestAgentPtr->isChecked())
            {
                newMode = "nearest_agent";
            }
            
            modeEditPtr->setText(newMode);
            
            // Also directly update the agents_ array to ensure it's saved
            if (agentIndex >= 0 && agentIndex < agents_.size())
            {
                QString paramKey = QString("%1.greeting_mode").arg(blockId);
                agents_[agentIndex].agentSpecificParams[paramKey] = newMode;
                qDebug() << "GreetingInitiator: Updated mode to" << newMode << "for agent" << agentIndex;
            }
            
            updateGreetingVisibility();
        };
        
        connect(greetRobotRadio, &QRadioButton::toggled, greetRobotRadio, updateGreetingMode);
        connect(specificAgentRadio, &QRadioButton::toggled, specificAgentRadio, updateGreetingMode);
        connect(nearestAgentRadio, &QRadioButton::toggled, nearestAgentRadio, updateGreetingMode);
        
        // Set the initial mode value in agents_ array
        if (agentIndex >= 0 && agentIndex < agents_.size())
        {
            QString paramKey = QString("%1.greeting_mode").arg(blockId);
            agents_[agentIndex].agentSpecificParams[paramKey] = savedMode;
            qDebug() << "GreetingInitiator: Initial mode set to" << savedMode << "for agent" << agentIndex;
        }
    }
    else if (blockId == "ProtectiveGuardian")
    {
        // Guardian Protector: IsRobotVisible -> IsAgentVisible -> IsRobotClose -> LookAtAgent -> ApproachAgent -> BlockRobot

        // Protection Mode Selection (place right after execution mode)
        auto protectionModeGroup = new QGroupBox("Protection Strategy");
        protectionModeGroup->setStyleSheet(""
            "QGroupBox { font-size: 13px; font-weight: 600; color: #495057; padding: 10px; margin-top: 5px; "
            // //"border: 1px solid #dee2e6; border-radius: 4px; }"
            "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }");
        auto protectionModeLayout = new QVBoxLayout(protectionModeGroup);
        
        auto specificProtectedRadio = new QRadioButton("🎯 Protect Specific Agent");
        specificProtectedRadio->setToolTip("Protect a single designated agent from robot threats");
        auto protectNearestRadio = new QRadioButton("🔍 Protect Nearest Threatened Agent");
        protectNearestRadio->setToolTip("Dynamically protect any agent that gets close to the robot");
        
        // Restore saved mode or default to specific_protected
        QString savedProtectionMode = "specific_protected";
        if (agents_.size() > agentIndex)
        {
            QString modeKey = QString("%1.protection_mode").arg(blockId);
            savedProtectionMode = agents_[agentIndex].agentSpecificParams.value(modeKey, "specific_protected").toString();
        }
        
        if (savedProtectionMode == "protect_nearest_threatened")
        {
            protectNearestRadio->setChecked(true);
        }
        else
        {
            specificProtectedRadio->setChecked(true);
        }
        
        protectionModeLayout->addWidget(specificProtectedRadio);
        protectionModeLayout->addWidget(protectNearestRadio);
        
        formLayout->addRow(protectionModeGroup);
        
        // Hidden field to store the selected mode
        auto protectionModeEdit = new QLineEdit(savedProtectionMode, protectionModeGroup);
        protectionModeEdit->setVisible(false);
        
        QString protectionModeKey = QString("%1_%2_protection_mode").arg(blockId).arg(agentIndex);
        blockConfigWidgets_[protectionModeKey] = protectionModeEdit;

        // Robot detection distance for IsRobotVisible
        auto robotDetectionSpin = new QDoubleSpinBox;
        robotDetectionSpin->setRange(5.0, 25.0);
        robotDetectionSpin->setValue(15.0);
        robotDetectionSpin->setSuffix(" m");
        robotDetectionSpin->setDecimals(1);
        robotDetectionSpin->setStyleSheet(fieldStyle);
        robotDetectionSpin->setToolTip("Maximum distance to detect robot as potential threat");

        auto robotDetectionLabel = new QLabel("🤖 Robot Detection Range:");
        robotDetectionLabel->setStyleSheet(labelStyle);
        formLayout->addRow(robotDetectionLabel, robotDetectionSpin);

        // Agent visibility distance for IsAgentVisible
        auto agentVisibilitySpin = new QDoubleSpinBox;
        agentVisibilitySpin->setRange(3.0, 20.0);
        agentVisibilitySpin->setValue(10.0);
        agentVisibilitySpin->setSuffix(" m");
        agentVisibilitySpin->setDecimals(1);
        agentVisibilitySpin->setStyleSheet(fieldStyle);
        agentVisibilitySpin->setToolTip("Maximum distance to detect agents that might need protection");

        auto agentVisibilityLabel = new QLabel("👥 Agent Detection Range:");
        agentVisibilityLabel->setStyleSheet(labelStyle);
        formLayout->addRow(agentVisibilityLabel, agentVisibilitySpin);

        // Robot close threshold for IsRobotClose
        auto robotCloseThresholdSpin = new QDoubleSpinBox;
        robotCloseThresholdSpin->setRange(1.0, 8.0);
        robotCloseThresholdSpin->setValue(3.0);
        robotCloseThresholdSpin->setSuffix(" m");
        robotCloseThresholdSpin->setDecimals(1);
        robotCloseThresholdSpin->setStyleSheet(fieldStyle);
        robotCloseThresholdSpin->setToolTip("Distance threshold to consider robot as immediate threat");

        auto robotCloseThresholdLabel = new QLabel("⚠️ Threat Distance Threshold:");
        robotCloseThresholdLabel->setStyleSheet(labelStyle);
        formLayout->addRow(robotCloseThresholdLabel, robotCloseThresholdSpin);

        // Approach distance for ApproachAgent
        auto protectiveApproachDistSpin = new QDoubleSpinBox;
        protectiveApproachDistSpin->setRange(0.5, 4.0);
        protectiveApproachDistSpin->setValue(1.8);
        protectiveApproachDistSpin->setSuffix(" m");
        protectiveApproachDistSpin->setDecimals(1);
        protectiveApproachDistSpin->setStyleSheet(fieldStyle);
        protectiveApproachDistSpin->setToolTip("Distance to maintain when approaching agent to help");

        auto protectiveApproachDistLabel = new QLabel("🛡️ Protective Approach Distance:");
        protectiveApproachDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(protectiveApproachDistLabel, protectiveApproachDistSpin);

        // Max velocity for ApproachAgent
        auto protectiveApproachVelSpin = new QDoubleSpinBox;
        protectiveApproachVelSpin->setRange(0.5, 2.5);
        protectiveApproachVelSpin->setValue(1.8);
        protectiveApproachVelSpin->setSuffix(" m/s");
        protectiveApproachVelSpin->setDecimals(2);
        protectiveApproachVelSpin->setStyleSheet(fieldStyle);
        protectiveApproachVelSpin->setToolTip("Speed when rushing to help the threatened agent");

        auto protectiveApproachVelLabel = new QLabel("⚡ Protective Response Speed:");
        protectiveApproachVelLabel->setStyleSheet(labelStyle);
        formLayout->addRow(protectiveApproachVelLabel, protectiveApproachVelSpin);

        // Duration for ApproachAgent
        auto protectiveApproachDurationSpin = new QDoubleSpinBox;
        protectiveApproachDurationSpin->setRange(1.0, 30.0);
        protectiveApproachDurationSpin->setValue(5.0);
        protectiveApproachDurationSpin->setSuffix(" s");
        protectiveApproachDurationSpin->setDecimals(1);
        protectiveApproachDurationSpin->setStyleSheet(fieldStyle);
        protectiveApproachDurationSpin->setToolTip("Duration to approach and stay near the threatened agent");

        auto protectiveApproachDurationLabel = new QLabel("⏱️ Approach Duration:");
        protectiveApproachDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(protectiveApproachDurationLabel, protectiveApproachDurationSpin);

        // Blocking duration for BlockRobot
        auto blockingDurationSpin = new QDoubleSpinBox;
        blockingDurationSpin->setRange(5.0, 60.0);
        blockingDurationSpin->setValue(20.0);
        blockingDurationSpin->setSuffix(" s");
        blockingDurationSpin->setDecimals(1);
        blockingDurationSpin->setStyleSheet(fieldStyle);
        blockingDurationSpin->setToolTip("Duration to block the robot protectively");

        auto blockingDurationLabel = new QLabel("🚧 Protective Blocking Duration:");
        blockingDurationLabel->setStyleSheet(labelStyle);
        formLayout->addRow(blockingDurationLabel, blockingDurationSpin);

        // Front distance for BlockRobot
        auto protectiveFrontDistSpin = new QDoubleSpinBox;
        protectiveFrontDistSpin->setRange(0.5, 3.0);
        protectiveFrontDistSpin->setValue(1.2);
        protectiveFrontDistSpin->setSuffix(" m");
        protectiveFrontDistSpin->setDecimals(1);
        protectiveFrontDistSpin->setStyleSheet(fieldStyle);
        protectiveFrontDistSpin->setToolTip("Distance in front when positioning to block robot");

        auto protectiveFrontDistLabel = new QLabel("🛑 Blocking Position Distance:");
        protectiveFrontDistLabel->setStyleSheet(labelStyle);
        formLayout->addRow(protectiveFrontDistLabel, protectiveFrontDistSpin);

        // Target Agent ID selection for which agent to protect (only visible in specific_protected mode)
        auto protectedAgentWidget = new QWidget;
        auto protectedAgentLayout = new QVBoxLayout(protectedAgentWidget);
        protectedAgentLayout->setContentsMargins(0, 0, 0, 0);
        protectedAgentLayout->setSpacing(4);

        // Instructions for protected agent selection
        auto protectedInstrLabel = new QLabel("Select agent to protect from robot threats:");
        protectedInstrLabel->setStyleSheet("font-size: 12px; color: #666; margin-bottom: 4px;");
        protectedAgentLayout->addWidget(protectedInstrLabel);

        // Create grid layout for protected agent buttons (max 3 per row)
        int totalAgents = getAgentCount();
        auto protectedButtonsLayout = new QGridLayout;
        protectedButtonsLayout->setContentsMargins(0, 0, 0, 0);
        protectedButtonsLayout->setSpacing(6);

        auto protectedButtonGroup = new QButtonGroup(this);
        QList<QPushButton *> protectedButtons;

        // Create buttons for each available agent (excluding current agent)
        int buttonIndex = 0;
        for (int i = 0; i < totalAgents; ++i)
        {
            if (i == agentIndex)
                continue; // Skip current agent

            auto protectedButton = new QPushButton(QString("Agent %1").arg(i + 1));
            protectedButton->setCheckable(true);
            protectedButton->setProperty("agentId", i + 1);
            protectedButton->setStyleSheet(
                "QPushButton { "
                "  padding: 6px 12px; border: 2px solid #ced4da; border-radius: 4px; "
                "  background: white; color: #495057; font-size: 13px; font-weight: 500; "
                "  min-width: 70px; }"
                "QPushButton:hover { "
                "  border-color: #28a745; color: #28a745; }"
                "QPushButton:checked { "
                "  background: #28a745; border-color: #28a745; color: white; font-weight: bold; }"
                "QPushButton:focus { "
                "  outline: none; border-color: #28a745; }");

            // Select default agent
            if (buttonIndex == 0)
            {
                protectedButton->setChecked(true);
            }

            protectedButtonGroup->addButton(protectedButton, i + 1);
            protectedButtons.append(protectedButton);

            // Add to grid layout (max 3 per row)
            int row = buttonIndex / 3;
            int col = buttonIndex % 3;
            protectedButtonsLayout->addWidget(protectedButton, row, col);
            buttonIndex++;
        }

        protectedAgentLayout->addLayout(protectedButtonsLayout);

        // Hidden spin box to store the selected protected agent value
        auto protectedAgentSpin = new QSpinBox(protectedAgentWidget);
        protectedAgentSpin->setRange(1, 100);
        protectedAgentSpin->setValue(agentIndex == 0 ? 2 : 1); // Default to different agent
        protectedAgentSpin->setStyleSheet(fieldStyle);
        protectedAgentSpin->setToolTip("Choose which agent to protect from robot threats");
        protectedAgentSpin->setVisible(false);

        auto protectedLabel = new QLabel("🛡️ Agent to Protect:");
        protectedLabel->setStyleSheet(labelStyle);
        formLayout->addRow(protectedLabel, protectedAgentWidget);

        // Connect button group to update hidden spinbox
        connect(protectedButtonGroup, QOverload<int>::of(&QButtonGroup::idClicked),
                [protectedAgentSpin](int agentId)
                {
                    protectedAgentSpin->setValue(agentId);
                });

        // Store widget references for parameter collection
        QString robotDetectionKey = QString("%1_%2_robot_detection_distance").arg(blockId).arg(agentIndex);
        QString agentVisibilityKey = QString("%1_%2_agent_visibility_distance").arg(blockId).arg(agentIndex);
        QString robotCloseThresholdKey = QString("%1_%2_robot_close_threshold").arg(blockId).arg(agentIndex);
        QString protectiveApproachDistKey = QString("%1_%2_protective_approach_distance").arg(blockId).arg(agentIndex);
        QString protectiveApproachVelKey = QString("%1_%2_protective_approach_velocity").arg(blockId).arg(agentIndex);
        QString protectiveApproachDurationKey = QString("%1_%2_protective_approach_duration").arg(blockId).arg(agentIndex);
        QString blockingDurationKey = QString("%1_%2_blocking_duration").arg(blockId).arg(agentIndex);
        QString protectiveFrontDistKey = QString("%1_%2_protective_front_distance").arg(blockId).arg(agentIndex);
        QString protectedAgentKey = QString("%1_%2_target_agent_id").arg(blockId).arg(agentIndex);

        blockConfigWidgets_[robotDetectionKey] = robotDetectionSpin;
        blockConfigWidgets_[agentVisibilityKey] = agentVisibilitySpin;
        blockConfigWidgets_[robotCloseThresholdKey] = robotCloseThresholdSpin;
        blockConfigWidgets_[protectiveApproachDistKey] = protectiveApproachDistSpin;
        blockConfigWidgets_[protectiveApproachVelKey] = protectiveApproachVelSpin;
        blockConfigWidgets_[protectiveApproachDurationKey] = protectiveApproachDurationSpin;
        blockConfigWidgets_[blockingDurationKey] = blockingDurationSpin;
        blockConfigWidgets_[protectiveFrontDistKey] = protectiveFrontDistSpin;
        blockConfigWidgets_[protectedAgentKey] = protectedAgentSpin;
        
        // Control visibility based on protection mode
        // Use QPointer for all widgets to prevent crashes on deletion
        QPointer<QLabel> protectedLabelPtr(protectedLabel);
        QPointer<QWidget> protectedAgentWidgetPtr(protectedAgentWidget);
        QPointer<QRadioButton> specificProtectedRadioPtr(specificProtectedRadio);
        
        auto updateProtectionVisibility = [protectedLabelPtr, protectedAgentWidgetPtr, specificProtectedRadioPtr]()
        {
            if (!protectedLabelPtr || !protectedAgentWidgetPtr || !specificProtectedRadioPtr)
                return;
            bool showSpecific = specificProtectedRadioPtr->isChecked();
            protectedLabelPtr->setVisible(showSpecific);
            protectedAgentWidgetPtr->setVisible(showSpecific);
        };
        
        // Set initial visibility
        updateProtectionVisibility();
        
        // Connect mode selection to update the hidden field and visibility
        QPointer<QLineEdit> protectionModeEditPtr(protectionModeEdit);
        QPointer<QRadioButton> specificProtectedPtr(specificProtectedRadio);
        QPointer<QRadioButton> protectNearestPtr(protectNearestRadio);
        
        auto updateProtectionMode = [protectionModeEditPtr, specificProtectedPtr, protectNearestPtr, updateProtectionVisibility]()
        {
            if (!protectionModeEditPtr || !specificProtectedPtr || !protectNearestPtr)
                return;
            if (specificProtectedPtr->isChecked())
            {
                protectionModeEditPtr->setText("specific_protected");
            }
            else if (protectNearestPtr->isChecked())
            {
                protectionModeEditPtr->setText("protect_nearest_threatened");
            }
            updateProtectionVisibility();
        };
        
        connect(specificProtectedRadio, &QRadioButton::toggled, specificProtectedRadio, updateProtectionMode);
        connect(protectNearestRadio, &QRadioButton::toggled, protectNearestRadio, updateProtectionMode);
    }
}

void BTConfigDialog::generateFinalPreview()
{
    // Ensure assignment matrix is up to date before generating preview
    if (!assignmentMatrix_)
    {
        // Try to find the assignment matrix widget
        auto pages = findChildren<QWizardPage *>();
        for (auto page : pages)
        {
            auto tables = page->findChildren<QTableWidget *>();
            for (auto table : tables)
            {
                if (table->objectName().contains("assignment") || table->rowCount() > 0)
                {
                    assignmentMatrix_ = table;
                    break;
                }
            }
            if (assignmentMatrix_)
                break;
        }
    }

    if (!assignmentMatrix_)
    {
        // Show a default preview indicating no configuration
        if (xmlPreviewEdit_)
        {
            xmlPreviewEdit_->setText("<!-- No configuration available -->\n<!-- Please go back and configure block assignments -->");
        }
        return;
    }

    // Update agent combo boxes with current agent data
    if (agentPreviewCombo_)
    {
        agentPreviewCombo_->clear();

        // Determine actual agent count from assignment matrix or agent names
        int agentCount = getAssignmentMatrixAgentCount();

        if (agentCount == 0)
        {
            // No agents loaded
            agentPreviewCombo_->addItem("No agents loaded", -1);
            agentPreviewCombo_->setEnabled(false);
            return;
        }

        // Populate with actual loaded agents
        for (int i = 0; i < agentCount; ++i)
        {
            QString agentName;

            agentName = QString("Agent %1").arg(i + 1);
            agentPreviewCombo_->addItem(agentName, i);
        }
        agentPreviewCombo_->setEnabled(true);
        // Set to first agent and update preview
        if (agentCount > 0)
        {
            agentPreviewCombo_->setCurrentIndex(0);
            updateXMLPreviewForAgent(0);
        }
    }

    // Update agent configuration combo box
    if (agentConfigCombo_)
    {
        agentConfigCombo_->clear();

        // Determine actual agent count from assignment matrix or agent names
        int agentCount = getAssignmentMatrixAgentCount();

        if (agentCount == 0)
        {
            // No agents loaded
            agentConfigCombo_->addItem("No agents loaded", -1);
            agentConfigCombo_->setEnabled(false);
        }
        else
        {
            // Populate with actual loaded agents
            for (int i = 0; i < agentCount; ++i)
            {
                QString agentDisplayName;
                if (!agentNames_.isEmpty() && i < agentNames_.size())
                {
                    // Use actual agent names and behavior types
                    QString behaviorType = i < behaviorTypes_.size() ? behaviorTypes_[i] : "Regular";
                    agentDisplayName = QString("Agent %1 (%2)")
                                           .arg(i + 1)
                                           .arg(behaviorType);
                }
                else
                {
                    // Use default names for loaded agents without specific names
                    agentDisplayName = QString("Agent %1").arg(i + 1);
                }
                agentConfigCombo_->addItem(agentDisplayName, i);
            }
            agentConfigCombo_->setEnabled(true);
        }
    }

    updateAssignmentSummaryWidget();
    updateXMLPreview();
}

void BTConfigDialog::updateAssignmentSummaryWidget()
{
    if (!assignmentSummaryLayout_)
        return;

    // Clear existing content
    QLayoutItem *item;
    while ((item = assignmentSummaryLayout_->takeAt(0)) != nullptr)
    {
        delete item->widget();
        delete item;
    }

    // Get agent count
    int agentCount = getAgentCount();
    bool hasAssignments = false;

    // Create compact summary table
    auto summaryTable = new QTableWidget;
    summaryTable->setStyleSheet(
        "QTableWidget { border: 1px solid #ddd; gridline-color: #eee; background: white; "
        "font-size: 14px; selection-background-color: #e3f2fd; }"
        "QHeaderView::section { background: #f8f9fa; color: #495057; font-weight: bold; "
        "border: 1px solid #ddd; padding: 6px 8px; font-size: 14px; }"
        "QTableWidget::item { padding: 4px 8px; border-bottom: 1px solid #f0f0f0; text-align: center;}"
        "QTableWidget::item:hover { background: #f8f9fa; }"
        "QTableWidget::item:selected { background: #e3f2fd; color: #1976d2; }");

    summaryTable->setColumnCount(3);
    summaryTable->setHorizontalHeaderLabels(QStringList() << "Agent" << "Type" << "Assigned Blocks");
    summaryTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    summaryTable->setSelectionMode(QAbstractItemView::SingleSelection);
    summaryTable->setShowGrid(true);
    summaryTable->setGridStyle(Qt::SolidLine);
    summaryTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    summaryTable->setSortingEnabled(false);
    summaryTable->setTextElideMode(Qt::ElideRight);
    summaryTable->setWordWrap(true);
    summaryTable->setAlternatingRowColors(true);
    summaryTable->verticalHeader()->setVisible(false);

    // Set column widths for better space usage
    summaryTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents); // Agent
    summaryTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents); // Type
    summaryTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);          // Blocks

    summaryTable->setRowCount(agentCount);

    // Populate table for each agent
    for (int agentIdx = 0; agentIdx < agentCount; ++agentIdx)
    {
        QStringList assignedBlocks;

        // Check which blocks are assigned to this agent
        if (assignmentMatrix_)
        {
            for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
            {
                QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, agentIdx);
                if (checkbox && checkbox->isChecked())
                {
                    QString blockName = assignmentMatrix_->verticalHeaderItem(row)->text();
                    assignedBlocks << blockName;
                    hasAssignments = true;
                }
            }
        }

        // Agent name (clickable)
        QString agentName = QString("%1").arg(agentIdx + 1);
        auto agentItem = new QTableWidgetItem(agentName);
        agentItem->setTextAlignment(Qt::AlignCenter);
        agentItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        agentItem->setData(Qt::UserRole, agentIdx);
        summaryTable->setItem(agentIdx, 0, agentItem);

        // Behavior type
        QString behaviorType = (agentIdx < behaviorTypes_.size()) ? behaviorTypes_[agentIdx] : "Regular";
        auto typeItem = new QTableWidgetItem(behaviorType);
        typeItem->setTextAlignment(Qt::AlignCenter);
        typeItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        summaryTable->setItem(agentIdx, 1, typeItem);

        // Assigned blocks (compact format)
        QString blocksText;
        if (!assignedBlocks.isEmpty())
        {
            QStringList displayBlocks;
            for (const QString &blockName : assignedBlocks)
            {
                QString icon = "📦"; // Default icon
                if (blockName.contains("Robot"))
                    icon = "🤖";
                else if (blockName.contains("Social") || blockName.contains("Talk"))
                    icon = "💬";
                else if (blockName.contains("Follow"))
                    icon = "👥";
                else if (blockName.contains("Avoidance"))
                    icon = "⚠️";

                displayBlocks << QString("%1 %2").arg(icon, blockName);
            }
            blocksText = displayBlocks.join(" → ");
        }
        else
        {
            blocksText = "❌ No blocks assigned";
        }

        auto blocksItem = new QTableWidgetItem(blocksText);
        blocksItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        blocksItem->setTextAlignment(Qt::AlignCenter);
        if (assignedBlocks.isEmpty())
        {
            blocksItem->setForeground(QColor("#999"));
            blocksItem->setFont(QFont("", -1, -1, true));
        }
        summaryTable->setItem(agentIdx, 2, blocksItem);
    }

    // Set optimal height based on content
    int rowHeight = summaryTable->verticalHeader()->defaultSectionSize();
    int headerHeight = summaryTable->horizontalHeader()->height();
    int totalHeight = headerHeight + (rowHeight * agentCount) + 2;

    // Set the table size
    summaryTable->setMaximumHeight(std::min(totalHeight, 300));
    summaryTable->setMinimumHeight(std::min(totalHeight, 120));
    summaryTable->setFixedHeight(std::min(totalHeight, 300));

    // Set the scroll area to size to its content
    assignmentSummaryArea_->setMaximumHeight(std::min(totalHeight + 20, 320));
    assignmentSummaryArea_->setMinimumHeight(std::min(totalHeight + 20, 140));

    // Connect row selection to update XML preview
    connect(summaryTable, &QTableWidget::itemSelectionChanged, [this, summaryTable]()
            {
        auto selectedItems = summaryTable->selectedItems();
        if (!selectedItems.isEmpty()) {
            int agentIndex = selectedItems.first()->data(Qt::UserRole).toInt();
            if (agentPreviewCombo_) {
                agentPreviewCombo_->setCurrentIndex(agentIndex);
                updateXMLPreviewForAgent(agentIndex);
            }
        } });

    assignmentSummaryLayout_->addWidget(summaryTable);

    if (!hasAssignments)
    {
        auto noDataLabel = new QLabel("💡 No blocks assigned. Go back to configure assignments.");
        noDataLabel->setStyleSheet("color: #ff9800; font-weight: bold; font-size: 12px; "
                                   "background: #fff3e0; padding: 8px; border-radius: 4px; "
                                   "border-left: 4px solid #ff9800; margin-top: 10px;");
        noDataLabel->setAlignment(Qt::AlignCenter);
        assignmentSummaryLayout_->addWidget(noDataLabel);
    }

    assignmentSummaryLayout_->addStretch();
}

void BTConfigDialog::updateXMLPreview()
{
    if (!xmlPreviewEdit_ || !agentPreviewCombo_)
        return;

    // Update for the currently selected agent
    int selectedAgentIndex = agentPreviewCombo_->currentData().toInt();
    updateXMLPreviewForAgent(selectedAgentIndex);
}

void BTConfigDialog::updateXMLPreviewForAgent(int agentIndex)
{
    if (!xmlPreviewEdit_)
    {
        return;
    }

    // Generate XML preview showing the XML section that will be patched
    QString xmlContent = "<?xml version=\"1.0\"?>\n";
    xmlContent += "<!-- XML patch that will be inserted into the BT file -->\n";
    xmlContent += QString("<!-- Agent %1 Behavior Tree Patch Preview -->\n").arg(agentIndex + 1);
    xmlContent += "<!-- This shows the XML blocks that will be added after the SetGoals sequence -->\n\n";

    // Get the current configuration with error handling
    Config config;
    try
    {
        config = getEnhancedConfig();
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception in getEnhancedConfig during XML preview:" << e.what();
        xmlContent += "<!-- ERROR: Failed to get configuration data -->\n";
        xmlContent += QString("<!-- Exception: %1 -->\n").arg(e.what());
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }
    catch (...)
    {
        qCritical() << "Unknown exception in getEnhancedConfig during XML preview";
        xmlContent += "<!-- ERROR: Unknown exception while getting configuration data -->\n";
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }

    // Check if we have a valid agent assignment
    if (agentIndex >= config.agentAssignments.size())
    {
        xmlContent += "<!-- NO CONFIGURATION FOUND FOR THIS AGENT -->\n";
        xmlContent += "<!-- Please configure blocks for this agent in the assignment matrix -->\n";
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }

    const auto &assignment = config.agentAssignments[agentIndex];

    if (assignment.assignedBlocks.isEmpty())
    {
        xmlContent += "<!-- NO BLOCKS ASSIGNED TO THIS AGENT -->\n";
        xmlContent += "<!-- This agent will only perform basic goal navigation -->\n";
        xmlContent += "<!-- Go back to the Configuration page to assign behavior blocks -->\n";
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }

    // Create a temporary XML document to generate the actual XML structure
    tinyxml2::XMLDocument tempDoc;
    auto *tempRoot = tempDoc.NewElement("TempRoot");
    tempDoc.InsertFirstChild(tempRoot);

    // Iterate over assigned blocks and generate XML
    try
    {
        for (const QString &blockId : assignment.assignedBlocks)
        {
            // Find the block definition
            auto blockDef = std::find_if(config.availableBlocks.begin(), config.availableBlocks.end(),
                                         [&blockId](const auto &block)
                                         { return block.blockId == blockId; });

            if (blockDef == config.availableBlocks.end())
            {
                xmlContent += QString("<!-- ERROR: Block '%1' not found in available blocks -->\n").arg(blockId);
                continue;
            }

            // Create container for this block
            auto *blockSeq = tempDoc.NewElement("Sequence");
            std::string name_str = ((blockId + "Block")).toStdString();
            blockSeq->SetAttribute("name", name_str.c_str());

            // Add comment for clarity
            std::string comment_str = (QString(" Block: %1 - %2 ").arg(blockId, blockDef->description)).toStdString();
            auto *comment = tempDoc.NewComment(comment_str.c_str());
            blockSeq->InsertEndChild(comment);

            // Get nodes for this block
            QStringList nodes = getEnhancedNodesForBlock(blockId);
            
            // Special handling for TalkInteract based on interaction mode
            if (blockId == "TalkInteract")
            {
                // Use the param key format (with dot) to match how it's stored
                QString modeKey = QString("%1.interaction_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
                
                if (mode == "specific_target")
                {
                    // Option B: Specific Target - Check specific agent(s)
                    // Note: For multiple participants, we'll create multiple IsAgentVisible/IsAgentClose nodes
                    // with different agent_id values during node creation
                    nodes = QStringList({"IsAgentVisible", "IsAgentClose", "ConversationFormation"});
                }
                else // find_nearest
                {
                    // Option C: Find Nearest - Dynamic discovery with looking
                    nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "LookAtAgent", "ConversationFormation"});
                }
            }
            // Special handling for FollowAgent based on interaction mode
            else if (blockId == "FollowAgent")
            {
                QString modeKey = QString("%1.interaction_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
                
                if (mode == "specific_target")
                {
                    // Specific Target - Follow a specific agent
                    nodes = QStringList({"IsAgentVisible", "IsAgentClose", "FollowAgent"});
                }
                else // find_nearest
                {
                    // Find Nearest - Dynamically find and follow nearest agent
                    nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "IsAgentClose", "FollowAgent"});
                }
            }
            // Special handling for AttentionSeeking based on attention mode
            else if (blockId == "AttentionSeeking")
            {
                QString modeKey = QString("%1.attention_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "agent_attention").toString();
                
                if (mode == "robot_attention")
                {
                    // Robot Attention - Detect robot looking and respond
                    nodes = QStringList({"IsRobotFacingAgent", "LookAtRobot", "SaySomething"});
                }
                else // agent_attention
                {
                    // Agent Attention - Detect other agents looking and respond
                    nodes = QStringList({"IsAnyoneLookingAtMe", "LookAtAgent", "SaySomething"});
                }
            }
            // Special handling for GreetingInitiator based on greeting mode
            else if (blockId == "GreetingInitiator")
            {
                QString modeKey = QString("%1.greeting_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_agent").toString();
                
                qDebug() << "GreetingInitiator XML Preview: modeKey=" << modeKey << "mode=" << mode;
                qDebug() << "Available params:" << assignment.agentSpecificParams.keys();
                
                // Map old mode names for backward compatibility
                if (mode == "specific_target")
                    mode = "specific_agent";
                else if (mode == "find_nearest")
                    mode = "nearest_agent";
                
                if (mode == "greet_robot")
                {
                    // Greet Robot - Detect, look at, and greet robot
                    nodes = QStringList({"IsRobotVisible", "LookAtRobot", "SaySomething"});
                }
                else if (mode == "nearest_agent")
                {
                    // Greet Nearest Agent - Dynamically find, look at, and greet nearest agent
                    nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "LookAtAgent", "SaySomething"});
                }
                else // specific_agent
                {
                    // Greet Specific Agent - Detect, look at, and greet specific agent
                    nodes = QStringList({"IsAgentVisible", "LookAtAgent", "SaySomething"});
                }
            }
            // Special handling for ProtectiveGuardian based on protection mode
            else if (blockId == "ProtectiveGuardian")
            {
                QString modeKey = QString("%1.protection_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
                
                if (mode == "protect_nearest_threatened")
                {
                    // Protect Nearest Threatened - Dynamically find any threatened agent
                    // First find nearest agent, then check if robot is threatening them
                    nodes = QStringList({"FindNearestAgent", "IsRobotVisible", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"});
                }
                else // specific_protected
                {
                    // Specific Protected - Protect specific agent from robot
                    nodes = QStringList({"IsRobotVisible", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"});
                }
            }

            for (const QString &nodeId : nodes)
            {
                try
                {
                    // Check if this is an Inverter wrapper
                    if (nodeId.startsWith("Inverter:"))
                    {
                        QString wrappedNodeId = nodeId.section(':', 1);

                        // Create simple Inverter element
                        auto *inverterElem = tempDoc.NewElement("Inverter");

                        // Create the wrapped condition
                        auto *conditionElem = tempDoc.NewElement("Condition");
                        std::string id_str = (wrappedNodeId).toStdString();
                        conditionElem->SetAttribute("ID", id_str.c_str());

                        // Apply parameters to the wrapped condition
                        applyBlockParametersToNode(conditionElem, blockId, wrappedNodeId, config);
                        applyAgentSpecificParametersToNode(conditionElem, agentIndex, blockId, wrappedNodeId, assignment);

                        // Add wrapped condition to inverter
                        inverterElem->InsertEndChild(conditionElem);

                        // Check if this block should be wrapped in RunOnce tags
                        if (assignment.runOnceBlocks.contains(blockId))
                        {
                            // Create RunOnce wrapper for the entire Inverter
                            auto *runOnceElem = tempDoc.NewElement("RunOnce");
                            runOnceElem->InsertEndChild(inverterElem);
                            blockSeq->InsertEndChild(runOnceElem);
                        }
                        else
                        {
                            blockSeq->InsertEndChild(inverterElem);
                        }

                        continue;
                    }

                    // Special handling for TalkInteract with multiple participants
                    // For multi-participant conversations, we need multiple condition check instances
                    if (blockId == "TalkInteract" && (nodeId == "IsAgentVisible" || nodeId == "IsAgentClose"))
                    {
                        QString modeKey = QString("%1.interaction_mode").arg(blockId);
                        QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
                        
                        if (mode == "specific_target")
                        {
                            QString participantsKey = blockId + ".non_main_agent_ids";
                            if (assignment.agentSpecificParams.contains(participantsKey))
                            {
                                QString participantsStr = assignment.agentSpecificParams[participantsKey].toString();
                                QStringList participants = participantsStr.split(",", Qt::SkipEmptyParts);
                                
                                if (participants.size() > 1)
                                {
                                    // Multiple participants - create one node instance per participant
                                    for (const QString &participantId : participants)
                                    {
                                        QString nodeType = "Condition";
                                        std::string nodeType_str = nodeType.toStdString();
                                        auto *nodeElem = tempDoc.NewElement(nodeType_str.c_str());
                                        std::string id_str = (nodeId).toStdString();
                                        nodeElem->SetAttribute("ID", id_str.c_str());
                                        
                                        // Set participant-specific attributes
                                        if (nodeId == "IsAgentVisible")
                                        {
                                            nodeElem->SetAttribute("observer_id", "{id}");
                                            std::string agent_id_str = participantId.trimmed().toStdString();
                                            nodeElem->SetAttribute("agent_id", agent_id_str.c_str());
                                            nodeElem->SetAttribute("field_of_view", "3.14");
                                        }
                                        else if (nodeId == "IsAgentClose")
                                        {
                                            nodeElem->SetAttribute("observer_id", "{id}");
                                            std::string target_agent_id_str = participantId.trimmed().toStdString();
                                            nodeElem->SetAttribute("target_agent_id", target_agent_id_str.c_str());
                                        }
                                        
                                        // Apply parameters (distance/threshold from user config)
                                        applyBlockParametersToNode(nodeElem, blockId, nodeId, config);
                                        applyAgentSpecificParametersToNode(nodeElem, agentIndex, blockId, nodeId, assignment);
                                        
                                        // Add node to sequence
                                        if (assignment.runOnceBlocks.contains(blockId))
                                        {
                                            auto *runOnceElem = tempDoc.NewElement("RunOnce");
                                            runOnceElem->InsertEndChild(nodeElem);
                                            blockSeq->InsertEndChild(runOnceElem);
                                        }
                                        else
                                        {
                                            blockSeq->InsertEndChild(nodeElem);
                                        }
                                    }
                                    continue; // Skip the normal single-node creation below
                                }
                            }
                        }
                    }

                    // Determine correct XML element type based on node ID
                    QString nodeType = "Action"; // Default to Action
                    if (nodeId.contains("Is") || nodeId.contains("Condition") || nodeId == "RandomChanceCondition")
                    {
                        nodeType = "Condition";
                    }

                    std::string nodeType_str = nodeType.toStdString();
                    auto *nodeElem = tempDoc.NewElement(nodeType_str.c_str());
                    std::string id_str = (nodeId).toStdString();
                    nodeElem->SetAttribute("ID", id_str.c_str());

                    // Apply parameters to the node
                    applyBlockParametersToNode(nodeElem, blockId, nodeId, config);
                    applyAgentSpecificParametersToNode(nodeElem, agentIndex, blockId, nodeId, assignment);

                    // Check if this block should be wrapped in RunOnce tags
                    if (assignment.runOnceBlocks.contains(blockId))
                    {
                        // Create RunOnce wrapper
                        auto *runOnceElem = tempDoc.NewElement("RunOnce");
                        runOnceElem->InsertEndChild(nodeElem);
                        blockSeq->InsertEndChild(runOnceElem);
                    }
                    else
                    {
                        blockSeq->InsertEndChild(nodeElem);
                    }
                }
                catch (const std::exception &e)
                {
                    qWarning() << "Exception processing node" << nodeId << "in block" << blockId << ":" << e.what();
                    xmlContent += QString("<!-- ERROR processing node %1: %2 -->\n").arg(nodeId, e.what());
                }
                catch (...)
                {
                    qWarning() << "Unknown exception processing node" << nodeId << "in block" << blockId;
                    xmlContent += QString("<!-- ERROR: Unknown exception processing node %1 -->\n").arg(nodeId);
                }
            }

            // Check if this block should use random execution
            tinyxml2::XMLElement *elementToInsert = blockSeq;
            
            if (assignment.randomExecutionBlocks.contains(blockId))
            {
                // Wrap the entire block sequence in RandomChanceCondition
                double probability = assignment.randomExecutionBlocks[blockId];
                
                auto *randomCondition = tempDoc.NewElement("Condition");
                randomCondition->SetAttribute("ID", "RandomChanceCondition");
                randomCondition->SetAttribute("agent_id", "{id}");
                std::string probability_str = (QString::number(probability, 'f', 2)).toStdString();
                randomCondition->SetAttribute("probability", probability_str.c_str());
                
                // Create a new sequence to contain the random condition + block sequence
                auto *randomWrapper = tempDoc.NewElement("Sequence");
                std::string name_str = ((blockId + "RandomBlock")).toStdString();
                randomWrapper->SetAttribute("name", name_str.c_str());
                
                randomWrapper->InsertEndChild(randomCondition);
                randomWrapper->InsertEndChild(blockSeq);
                
                elementToInsert = randomWrapper;
            }

            tempRoot->InsertEndChild(elementToInsert);
        }
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception during XML generation:" << e.what();
        xmlContent += QString("<!-- FATAL ERROR during XML generation: %1 -->\n").arg(e.what());
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }
    catch (...)
    {
        qCritical() << "Unknown exception during XML generation";
        xmlContent += "<!-- FATAL ERROR: Unknown exception during XML generation -->\n";
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }

    // Convert the XML document to string with error handling
    QString tempXmlString;
    try
    {
        tinyxml2::XMLPrinter printer;
        tempDoc.Print(&printer);
        tempXmlString = QString::fromUtf8(printer.CStr());
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception during XML string generation:" << e.what();
        xmlContent += QString("<!-- ERROR converting XML to string: %1 -->\n").arg(e.what());
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }
    catch (...)
    {
        qCritical() << "Unknown exception during XML string generation";
        xmlContent += "<!-- ERROR: Unknown exception converting XML to string -->\n";
        xmlPreviewEdit_->setText(xmlContent);
        return;
    }

    // Extract just the block sequences
    QStringList lines = tempXmlString.split('\n');
    QStringList relevantLines;
    bool insideSequence = false;
    int sequenceDepth = 0;

    for (const QString &line : lines)
    {
        // Match both regular blocks and random blocks
        if (line.contains("<Sequence name=") && (line.contains("Block\"") || line.contains("RandomBlock\"")))
        {
            insideSequence = true;
            sequenceDepth = 1;
            relevantLines.append(line);
        }
        else if (insideSequence)
        {
            // Track nested sequences
            if (line.contains("<Sequence"))
            {
                sequenceDepth++;
            }
            else if (line.contains("</Sequence>"))
            {
                sequenceDepth--;
                relevantLines.append(line);
                
                if (sequenceDepth == 0)
                {
                    relevantLines.append(""); // Add blank line between blocks
                    insideSequence = false;
                }
            }
            else
            {
                relevantLines.append(line);
            }
        }
    }

    if (!relevantLines.isEmpty())
    {
        xmlContent += relevantLines.join('\n');
    }
    else
    {
        xmlContent += "<!-- ERROR: Failed to generate XML preview -->\n";
    }

    // Add configuration summary at the end
    try
    {
        xmlContent += "\n\n<!-- Configuration Summary -->\n";
        xmlContent += QString("<!-- Agent: Agent %1 -->\n").arg(agentIndex + 1);
        xmlContent += QString("<!-- Behavior Type: %1 -->\n").arg(agentIndex < behaviorTypes_.size() ? behaviorTypes_[agentIndex] : "Regular");
        xmlContent += QString("<!-- Assigned Blocks: %1 -->\n").arg(assignment.assignedBlocks.join(", "));
        xmlContent += QString("<!-- Execution Order: %1 -->\n").arg(assignment.randomizeOrder ? "RANDOMIZED" : assignment.assignedBlocks.join(" → "));
    }
    catch (const std::exception &e)
    {
        qWarning() << "Exception generating configuration summary:" << e.what();
        xmlContent += QString("<!-- ERROR in configuration summary: %1 -->\n").arg(e.what());
    }
    catch (...)
    {
        qWarning() << "Unknown exception generating configuration summary";
        xmlContent += "<!-- ERROR: Unknown exception in configuration summary -->\n";
    }

    // Safely set the text content
    try
    {
        xmlPreviewEdit_->setText(xmlContent);
        qDebug() << "Set real XML content for agent" << agentIndex << "with" << assignment.assignedBlocks.size() << "blocks";
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception setting XML preview text:" << e.what();
    }
    catch (...)
    {
        qCritical() << "Unknown exception setting XML preview text";
    }
}

void BTConfigDialog::applyConfiguration()
{
    try
    {
        qDebug() << "applyConfiguration() started";

        // Validate the current configuration with enhanced error handling
        Config config;
        try
        {
            config = getConfig();
            qDebug() << "Configuration obtained successfully";
        }
        catch (const std::exception &e)
        {
            qCritical() << "Failed to get configuration:" << e.what();
            QMessageBox::critical(this, "Configuration Error",
                                  QString("Failed to retrieve configuration: %1").arg(e.what()));
            return;
        }
        catch (...)
        {
            qCritical() << "Unknown exception getting configuration";
            QMessageBox::critical(this, "Configuration Error",
                                  "Unknown error occurred while retrieving configuration.");
            return;
        }

        // Check if any blocks are assigned to agents with more robust checking
        bool hasAssignments = false;
        int totalAssignments = 0;

        try
        {
            for (const auto &agent : config.agentAssignments)
            {
                int agentAssignments = agent.assignedBlocks.size();
                if (agentAssignments > 0)
                {
                    hasAssignments = true;
                    totalAssignments += agentAssignments;
                }
            }

            qDebug() << "Assignments check completed. Has assignments:" << hasAssignments << "Total:" << totalAssignments;
        }
        catch (const std::exception &e)
        {
            qWarning() << "Exception checking assignments:" << e.what();
        }

        // Additional check: if we still don't have assignments, check the matrix directly
        if (!hasAssignments && assignmentMatrix_)
        {
            try
            {
                for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
                {
                    for (int col = 0; col < assignmentMatrix_->columnCount(); ++col)
                    {
                        QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, col);
                        if (checkbox && checkbox->isChecked())
                        {
                            hasAssignments = true;
                            totalAssignments++;
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                qWarning() << "Exception checking assignment matrix:" << e.what();
            }
        }

        if (!hasAssignments || totalAssignments == 0)
        {
            QMessageBox msgBox(this);
            msgBox.setWindowTitle("No Configuration");
            msgBox.setText("No behavior blocks have been assigned to agents.\n\n"
                           "Please go back and assign some blocks to agents before applying.");
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.setIcon(QMessageBox::Information);

            msgBox.setStyleSheet(
                "QMessageBox {"
                "  background-color: white;"
                "  color: black;"
                "}"
                "QMessageBox QLabel {"
                "  background-color: transparent;"
                "  color: #333333;"
                "  font-size: 12px;"
                "  padding: 10px;"
                "}"
                "QMessageBox QPushButton {"
                "  background-color: #007bff;"
                "  color: white;"
                "  border: none;"
                "  border-radius: 4px;"
                "  padding: 8px 16px;"
                "  font-size: 12px;"
                "  min-width: 80px;"
                "}"
                "QMessageBox QPushButton:hover {"
                "  background-color: #0056b3;"
                "}");

            msgBox.exec();
            return;
        }

        // Count unique blocks assigned across all agents for summary
        QSet<QString> uniqueBlocks;
        try
        {
            for (const auto &agent : config.agentAssignments)
            {
                for (const QString &blockId : agent.assignedBlocks)
                {
                    uniqueBlocks.insert(blockId);
                }
            }
        }
        catch (const std::exception &e)
        {
            qWarning() << "Exception counting unique blocks:" << e.what();
        }

        // Show confirmation dialog with summary
        QString summaryText = QString("Apply behavior configuration?\n\n"
                                      "📊 Summary:\n"
                                      "• %1 agent(s) configured\n"
                                      "• %2 total block assignment(s)\n"
                                      "• %3 different block type(s) used\n\n"
                                      "This will modify the behavior tree files for all configured agents.")
                                  .arg(config.agentAssignments.size())
                                  .arg(totalAssignments)
                                  .arg(uniqueBlocks.size());

        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Apply Configuration");
        msgBox.setText(summaryText);
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::Yes);
        msgBox.setIcon(QMessageBox::Question);

        msgBox.setStyleSheet(
            "QMessageBox {"
            "  background-color: white;"
            "  color: black;"
            "}"
            "QMessageBox QLabel {"
            "  background-color: transparent;"
            "  color: #333333;"
            "  font-size: 12px;"
            "  padding: 10px;"
            "}"
            "QMessageBox QPushButton {"
            "  background-color: #007bff;"
            "  color: white;"
            "  border: none;"
            "  border-radius: 4px;"
            "  padding: 8px 16px;"
            "  font-size: 12px;"
            "  min-width: 80px;"
            "}"
            "QMessageBox QPushButton:hover {"
            "  background-color: #0056b3;"
            "}"
            "QMessageBox QPushButton:pressed {"
            "  background-color: #004085;"
            "}"
            "QMessageBox QPushButton:default {"
            "  background-color: #28a745;"
            "}"
            "QMessageBox QPushButton:default:hover {"
            "  background-color: #1e7e34;"
            "}");

        int reply = msgBox.exec();

        if (reply != QMessageBox::Yes)
        {
            return;
        }

        qDebug() << "Accepting dialog";
        accept();
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception in applyConfiguration:" << e.what();
        QMessageBox::critical(this, "Error", QString("An error occurred: %1").arg(e.what()));
    }
    catch (...)
    {
        qCritical() << "Unknown exception in applyConfiguration";
        QMessageBox::critical(this, "Error", "An unknown error occurred while applying configuration.");
    }
}

BTConfigDialog::Config BTConfigDialog::getEnhancedConfig() const
{
    try
    {
        qDebug() << "getEnhancedConfig() started";

        Config config;

        // Copy basic configuration
        config.availableBlocks.clear();
        for (const auto &block : blocks_)
        {
            config.availableBlocks.append(block);
        }

        // Enhanced agent assignments with data from assignment matrix
        config.agentAssignments.clear();

        // Get agent count
        int agentCount = getAgentCount();
        qDebug() << "Processing" << agentCount << "agents";

        // Clean agent loop replacement
        for (int i = 0; i < agentCount; ++i)
        {
            qDebug() << "Processing agent" << i;
            AgentAssignment assignment;

            try
            {
                // Basic agent information
                assignment.agentIndex = i;

                // Safe access to agentNames_
                if (!agentNames_.isEmpty() && i < agentNames_.size())
                {
                    assignment.agentName = agentNames_[i];
                    qDebug() << "Agent name:" << assignment.agentName;
                }
                else
                {
                    assignment.agentName = QString("Agent_%1").arg(i + 1);
                    qDebug() << "Default agent name:" << assignment.agentName;
                }

                // Get selected blocks from assignment matrix
                if (assignmentMatrix_)
                {
                    for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
                    {
                        try
                        {
                            auto headerItem = assignmentMatrix_->verticalHeaderItem(row);
                            if (headerItem)
                            {
                                QString displayName = headerItem->text();
                                QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, i);
                                if (checkbox && checkbox->isChecked())
                                {
                                    // Convert display name to block ID
                                    QString blockId;
                                    for (const auto &block : blocks_)
                                    {
                                        if (block.displayName == displayName)
                                        {
                                            blockId = block.blockId;
                                            break;
                                        }
                                    }

                                    if (!blockId.isEmpty())
                                    {
                                        assignment.assignedBlocks.append(blockId);
                                        qDebug() << "Agent" << i << "assigned to block:" << displayName << "-> ID:" << blockId;
                                    }
                                    else
                                    {
                                        qWarning() << "Could not find block ID for display name:" << displayName;
                                    }
                                }
                            }
                            else
                            {
                                qWarning() << "No header item for row" << row;
                            }
                        }
                        catch (const std::exception &e)
                        {
                            qWarning() << "Exception accessing matrix row" << row << "col" << i << ":" << e.what();
                        }
                        catch (...)
                        {
                            qWarning() << "Unknown exception accessing matrix row" << row << "col" << i;
                        }
                    }
                }
                else
                {
                    qWarning() << "Assignment matrix is null!";
                }

                // Copy agent-specific settings from agents_ array if available
                if (i < agents_.size())
                {
                    assignment.randomizeOrder = agents_[i].randomizeOrder;
                    assignment.runOnceBlocks = agents_[i].runOnceBlocks;
                    assignment.randomExecutionBlocks = agents_[i].randomExecutionBlocks;

                    // Copy all agent-specific parameters directly from live data
                    for (auto it = agents_[i].agentSpecificParams.constBegin();
                         it != agents_[i].agentSpecificParams.constEnd(); ++it)
                    {
                        assignment.agentSpecificParams[it.key()] = it.value();
                        qDebug() << "Copied live parameter:" << it.key() << "=" << it.value() << "for agent" << i;
                    }
                }
                else
                {
                    qDebug() << "No agent data available for agent" << i;
                }

                config.agentAssignments.append(assignment);
            }
            catch (const std::exception &e)
            {
                qWarning() << "Exception processing agent" << i << ":" << e.what();
            }
            catch (...)
            {
                qWarning() << "Unknown exception processing agent" << i;
            }
        }

        // Collect global parameters
        config.parameters.clear();
        for (const auto &assignment : config.agentAssignments)
        {
            for (auto it = assignment.agentSpecificParams.constBegin();
                 it != assignment.agentSpecificParams.constEnd();
                 ++it)
            {
                config.parameters[it.key()] = it.value();
            }
        }

        qDebug() << "getEnhancedConfig() completed successfully";
        return config;
    }
    catch (const std::exception &e)
    {
        qCritical() << "Exception in getEnhancedConfig:" << e.what();
        // Return a minimal valid config to prevent crashes
        Config fallbackConfig;
        return fallbackConfig;
    }
    catch (...)
    {
        qCritical() << "Unknown exception in getEnhancedConfig";
        // Return a minimal valid config to prevent crashes
        Config fallbackConfig;
        return fallbackConfig;
    }
}

// ===================== OPTIMIZATION HELPER METHODS =====================

int BTConfigDialog::getAgentCount() const
{
    return agentNames_.isEmpty() ? 5 : agentNames_.size();
}

int BTConfigDialog::getAssignmentMatrixAgentCount() const
{
    if (assignmentMatrix_ && assignmentMatrix_->columnCount() > 0)
    {
        return assignmentMatrix_->columnCount();
    }
    else
    {
        return getAgentCount();
    }
}

void BTConfigDialog::updateButtonSelectionState(QPushButton *selectedButton)
{
    auto contentWidget = selectedButton->parentWidget();
    if (contentWidget)
    {
        auto buttons = contentWidget->findChildren<QPushButton *>();
        for (auto btn : buttons)
        {
            if (btn->objectName().startsWith("block_"))
            {
                btn->setChecked(btn == selectedButton);
            }
        }
    }
}

bool BTConfigDialog::validateConfiguration(const Config &config) const
{
    qDebug() << "validateConfiguration() called";
    // Check that at least one agent has block assignments
    bool hasAssignments = false;

    qDebug() << "Checking assignment matrix...";
    // Check assignment matrix first
    if (assignmentMatrix_)
    {
        for (int col = 0; col < assignmentMatrix_->columnCount(); ++col)
        {
            for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
            {
                QCheckBox *checkbox = getCheckBoxFromMatrixCell(row, col);
                if (checkbox && checkbox->isChecked())
                {
                    hasAssignments = true;
                    break;
                }
            }
            if (hasAssignments)
                break;
        }
    }
    qDebug() << "Assignment matrix check done, hasAssignments:" << hasAssignments;

    // Fallback: check agents_ array
    qDebug() << "Checking agents_ array as fallback...";
    if (!hasAssignments)
    {
        for (const auto &agent : agents_)
        {
            if (!agent.assignedBlocks.isEmpty())
            {
                hasAssignments = true;
                break;
            }
        }
    }
    qDebug() << "Agents array check done, hasAssignments:" << hasAssignments;

    if (!hasAssignments)
    {
        qDebug() << "No assignments found, showing warning";
        QMessageBox::warning(const_cast<BTConfigDialog *>(this),
                             "No Agent Assignments",
                             "Please assign at least one behavior block to an agent before applying configuration.");
        return false;
    }

    qDebug() << "Validating parameter ranges...";
    // Validate parameter ranges and required values using the config data
    QStringList validationErrors;

    qDebug() << "Validating" << config.agentAssignments.size() << "agents from config data";
    
    for (int i = 0; i < config.agentAssignments.size(); ++i)
    {
        const auto &agentConfig = config.agentAssignments[i];
        qDebug() << "Validating agent" << i << "with" << agentConfig.assignedBlocks.size() << "blocks";
        
        // Check if agent has FollowAgent block and validate target_agent_id parameter
        if (agentConfig.assignedBlocks.contains("FollowAgent"))
        {
            // Check if it's in "specific_target" mode - only then is target_agent_id required
            QString modeKey = "FollowAgent.interaction_mode";
            QString mode = "specific_target"; // Default to specific_target if no mode specified
            
            if (agentConfig.agentSpecificParams.contains(modeKey))
            {
                mode = agentConfig.agentSpecificParams[modeKey].toString();
            }
            
            // Only validate target_agent_id if in specific_target mode
            if (mode == "specific_target")
            {
                QString paramKey = "FollowAgent.target_agent_id";
                if (agentConfig.agentSpecificParams.contains(paramKey))
                {
                    int targetId = agentConfig.agentSpecificParams[paramKey].toInt();
                    // Validate that target agent ID is valid (1-based indexing)
                    if (targetId < 1 || targetId > config.agentAssignments.size())
                    {
                        validationErrors.append(QString("Agent %1: FollowAgent has invalid target agent ID %2").arg(i + 1).arg(targetId));
                    }
                }
                else
                {
                    validationErrors.append(QString("Agent %1: FollowAgent in specific_target mode requires a target agent").arg(i + 1));
                }
            }
            // If in "find_nearest" mode, target_agent_id is not required
        }
        
        // Check if agent has TalkInteract block and validate based on interaction mode
        if (agentConfig.assignedBlocks.contains("TalkInteract"))
        {
            // Check if it's in "specific_target" mode - only then might target_agent_id be required
            QString modeKey = "TalkInteract.interaction_mode";
            QString mode = "find_nearest"; // Default to find_nearest
            
            if (agentConfig.agentSpecificParams.contains(modeKey))
            {
                mode = agentConfig.agentSpecificParams[modeKey].toString();
            }
            
            // In specific_target mode, check if non_main_agent_ids is specified
            if (mode == "specific_target")
            {
                QString paramKey = "TalkInteract.non_main_agent_ids";
                if (!agentConfig.agentSpecificParams.contains(paramKey) || 
                    agentConfig.agentSpecificParams[paramKey].toString().trimmed().isEmpty())
                {
                    validationErrors.append(QString("Agent %1: TalkInteract in specific_target mode requires conversation participants").arg(i + 1));
                }
            }
            // If in "find_nearest" mode, no specific agents required
        }
        
        // Check if agent has ConversationFormation block and validate target_agent_id parameter
        if (agentConfig.assignedBlocks.contains("ConversationFormation"))
        {
            QString paramKey = "ConversationFormation.target_agent_id";
            if (agentConfig.agentSpecificParams.contains(paramKey))
            {
                int targetId = agentConfig.agentSpecificParams[paramKey].toInt();
                if (targetId < 1 || targetId > config.agentAssignments.size())
                {
                    validationErrors.append(QString("Agent %1: ConversationFormation has invalid target agent ID %2").arg(i + 1).arg(targetId));
                }
            }
            // ConversationFormation target_agent_id is optional (can use find_nearest mode)
        }
        
        // Check if agent has GoTo block and validate goal_id parameter
        if (agentConfig.assignedBlocks.contains("GoTo"))
        {
            QString paramKey = "GoTo.goal_id";
            if (agentConfig.agentSpecificParams.contains(paramKey))
            {
                int goalId = agentConfig.agentSpecificParams[paramKey].toInt();
                if (goalId < 1)
                {
                    validationErrors.append(QString("Agent %1: GoTo requires a valid goal ID (≥1)").arg(i + 1));
                }
            }
            else
            {
                validationErrors.append(QString("Agent %1: GoTo requires a goal ID").arg(i + 1));
            }
        }
        
        // Check if agent has GroupWalk block and validate agents list
        if (agentConfig.assignedBlocks.contains("GroupWalk"))
        {
            QString paramKey = "GroupWalk.group_agent_ids";
            if (agentConfig.agentSpecificParams.contains(paramKey))
            {
                QString agentIds = agentConfig.agentSpecificParams[paramKey].toString().trimmed();
                if (agentIds.isEmpty())
                {
                    validationErrors.append(QString("Agent %1: GroupWalk requires a list of agent IDs").arg(i + 1));
                }
                else
                {
                    // Validate agent ID format
                    QStringList ids = agentIds.split(",", Qt::SkipEmptyParts);
                    for (const QString &id : ids)
                    {
                        bool ok;
                        int agentId = id.trimmed().toInt(&ok);
                        if (!ok || agentId < 1 || agentId > config.agentAssignments.size())
                        {
                            validationErrors.append(QString("Agent %1: Invalid agent ID '%2' in GroupWalk list").arg(i + 1).arg(id.trimmed()));
                        }
                    }
                }
            }
            else
            {
                validationErrors.append(QString("Agent %1: GroupWalk requires a list of agent IDs").arg(i + 1));
            }
        }
    }

    if (!validationErrors.isEmpty())
    {
        QMessageBox::warning(const_cast<BTConfigDialog *>(this),
                             "Parameter Validation Failed",
                             "Please fix the following issues:\n\n" + validationErrors.join("\n"));
        return false;
    }

    if (!hasAssignments)
    {
        QMessageBox::warning(const_cast<BTConfigDialog *>(this),
                             "No Assignments",
                             "Please assign at least one behavior block to at least one agent.");
        return false;
    }

    // Validate agent-specific parameters using the passed configuration data
    for (int i = 0; i < config.agentAssignments.size(); ++i)
    {
        const auto &agent = config.agentAssignments[i];

        for (const QString &blockId : agent.assignedBlocks)
        {
            if (blockId == "FollowAgent")
            {
                // Check the interaction mode
                QString modeKey = QString("%1.interaction_mode").arg(blockId);
                QString mode = agent.agentSpecificParams.value(modeKey, "find_nearest").toString();
                
                // Only validate target agent in specific_target mode
                if (mode == "specific_target")
                {
                    QString targetKey = QString("%1.target_agent_id").arg(blockId);
                    if (!agent.agentSpecificParams.contains(targetKey))
                    {
                        QMessageBox::warning(const_cast<BTConfigDialog *>(this),
                                             "Invalid Configuration",
                                             QString("Agent %1 has FollowAgent behavior in specific target mode but no target agent specified.")
                                                 .arg(i + 1));
                        return false;
                    }

                    int targetId = agent.agentSpecificParams[targetKey].toInt();
                    if (targetId == i + 1)
                    { // Can't follow yourself
                        QMessageBox::warning(const_cast<BTConfigDialog *>(this),
                                             "Invalid Configuration",
                                             QString("Agent %1 cannot follow itself. Please select a different target.")
                                                 .arg(i + 1));
                        return false;
                    }
                }
                // In find_nearest mode, target is determined dynamically by FindNearestAgent
            }
        }
    }

    return true;
}

void BTConfigDialog::updateValidationFeedback()
{
    // Reset all widget styles first
    for (auto it = blockConfigWidgets_.begin(); it != blockConfigWidgets_.end(); ++it)
    {
        if (auto widget = it.value())
        {
            widget->setStyleSheet(widget->styleSheet().replace("border: 2px solid red;", ""));
        }
    }

    // Check for validation errors and highlight problematic widgets
    for (int i = 0; i < agentNames_.size(); ++i)
    {
        // Check FollowAgent target selection
        QString followKey = QString("FollowAgent_%1_target_agent_id").arg(i);
        if (blockConfigWidgets_.contains(followKey))
        {
            auto combo = qobject_cast<QComboBox *>(blockConfigWidgets_[followKey]);
            if (combo && combo->isEnabled() && combo->currentIndex() < 0)
            {
                QString style = combo->styleSheet();
                if (!style.contains("border: 2px solid red"))
                {
                    combo->setStyleSheet(style + " QComboBox { border: 2px solid red; }");
                }
            }
        }

        // Check ApproachAgent target selection
        QString approachKey = QString("ApproachAgent_%1_target_agent_id").arg(i);
        if (blockConfigWidgets_.contains(approachKey))
        {
            auto combo = qobject_cast<QComboBox *>(blockConfigWidgets_[approachKey]);
            if (combo && combo->isEnabled() && combo->currentIndex() < 0)
            {
                QString style = combo->styleSheet();
                if (!style.contains("border: 2px solid red"))
                {
                    combo->setStyleSheet(style + " QComboBox { border: 2px solid red; }");
                }
            }
        }

        // Check GoTo goal ID
        QString gotoKey = QString("GoTo_%1_goal_id").arg(i);
        if (blockConfigWidgets_.contains(gotoKey))
        {
            auto spin = qobject_cast<QSpinBox *>(blockConfigWidgets_[gotoKey]);
            if (spin && spin->isEnabled() && spin->value() < 1)
            {
                QString style = spin->styleSheet();
                if (!style.contains("border: 2px solid red"))
                {
                    spin->setStyleSheet(style + " QSpinBox { border: 2px solid red; }");
                }
            }
        }

        // Check ConversationFormation target selection
        QString convKey = QString("ConversationFormation_%1_target_agent_id").arg(i);
        if (blockConfigWidgets_.contains(convKey))
        {
            auto combo = qobject_cast<QComboBox *>(blockConfigWidgets_[convKey]);
            if (combo && combo->isEnabled() && combo->currentIndex() < 0)
            {
                QString style = combo->styleSheet();
                if (!style.contains("border: 2px solid red"))
                {
                    combo->setStyleSheet(style + " QComboBox { border: 2px solid red; }");
                }
            }
        }

        // Check SetGroupWalk agents list
        QString groupKey = QString("GroupWalk_%1_agents_list").arg(i);
        if (blockConfigWidgets_.contains(groupKey))
        {
            auto lineEdit = qobject_cast<QLineEdit *>(blockConfigWidgets_[groupKey]);
            if (lineEdit && lineEdit->isEnabled())
            {
                QString text = lineEdit->text().trimmed();
                bool hasError = false;

                if (text.isEmpty())
                {
                    hasError = true;
                }
                else
                {
                    QStringList ids = text.split(",", Qt::SkipEmptyParts);
                    for (const QString &id : ids)
                    {
                        bool ok;
                        int agentId = id.trimmed().toInt(&ok);
                        if (!ok || agentId < 1 || agentId > agentNames_.size())
                        {
                            hasError = true;
                            break;
                        }
                    }
                }

                if (hasError)
                {
                    QString style = lineEdit->styleSheet();
                    if (!style.contains("border: 2px solid red"))
                    {
                        lineEdit->setStyleSheet(style + " QLineEdit { border: 2px solid red; }");
                    }
                }
            }
        }
    }
}

void BTConfigDialog::accept()
{
    qDebug() << "BTConfigDialog::accept() called";
    
    // Get configuration once and use it for validation
    auto config = getEnhancedConfig();

    if (validateConfiguration(config))
    {
        qDebug() << "Configuration validated, calling QWizard::accept()";
        QWizard::accept();
        qDebug() << "QWizard::accept() returned successfully";
    }
    else
    {
        qDebug() << "Configuration validation failed, staying on current page";
    }
    // If validation fails, stay on current page
}

void BTConfigDialog::clearAllAssignments()
{
    for (auto &agent : agents_)
    {
        agent.assignedBlocks.clear();
        agent.runOnceBlocks.clear();
        agent.randomExecutionBlocks.clear();
        agent.agentSpecificParams.clear();
    }

    // Update matrix checkboxes
    if (assignmentMatrix_)
    {
        for (int row = 0; row < assignmentMatrix_->rowCount(); ++row)
        {
            for (int col = 0; col < assignmentMatrix_->columnCount(); ++col)
            {
                auto cellWidget = assignmentMatrix_->cellWidget(row, col);
                if (auto checkbox = qobject_cast<QCheckBox *>(cellWidget))
                {
                    checkbox->setChecked(false);
                }
                else if (cellWidget)
                {
                    // Handle composite widgets
                    auto container = cellWidget;
                    auto checkbox = container->findChild<QCheckBox *>();
                    if (checkbox)
                    {
                        checkbox->setChecked(false);
                    }
                    auto combo = container->findChild<QComboBox *>();
                    if (combo)
                    {
                        combo->setEnabled(false);
                    }
                }
            }
        }
    }
}

void BTConfigDialog::updateAgentAssignment(int agentIndex, const QString &blockId, bool assigned)
{
    if (agentIndex < 0 || agentIndex >= agents_.size())
        return;

    if (assigned)
    {
        if (!agents_[agentIndex].assignedBlocks.contains(blockId))
        {
            agents_[agentIndex].assignedBlocks.append(blockId);
        }
    }
    else
    {
        agents_[agentIndex].assignedBlocks.removeAll(blockId);
    }
}

BTConfigDialog::Config BTConfigDialog::getConfig() const
{
    return getEnhancedConfig();
}

// Helper method to validate wizard config
bool BTConfigDialog::validateWizardConfig(const Config &config, int totalAgents)
{
    // Check that we have at least one agent assignment with blocks
    bool hasAssignments = false;
    for (const auto &assignment : config.agentAssignments)
    {
        if (!assignment.assignedBlocks.isEmpty())
        {
            hasAssignments = true;
            break;
        }
    }

    if (!hasAssignments)
        return false;

    // Validate agent assignments
    for (const auto &assignment : config.agentAssignments)
    {
        if (assignment.agentIndex < 0 || assignment.agentIndex >= totalAgents)
        {
            return false;
        }

        // Validate FollowAgent parameters if assigned
        if (assignment.assignedBlocks.contains("FollowAgent"))
        {
            // Check the interaction mode
            QString modeKey = QString("FollowAgent.interaction_mode");
            QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
            
            // Only validate target agent in specific_target mode
            if (mode == "specific_target")
            {
                QString targetKey = QString("FollowAgent.target_agent_id");
                if (!assignment.agentSpecificParams.contains(targetKey))
                {
                    return false; // Missing required target_agent_id parameter in specific_target mode
                }

                int targetId = assignment.agentSpecificParams[targetKey].toInt();
                if (targetId == assignment.agentIndex + 1)
                {
                    return false; // Can't follow yourself
                }
            }
            // In find_nearest mode, target is determined dynamically by FindNearestAgent
        }
    }

    return true;
}

// Helper method to generate BT paths if needed
QStringList BTConfigDialog::generateBTPathsForScenario(const QString &scenarioName,
                                                       const QString &simulator,
                                                       int agentCount)
{
    QString packageName;
    if (simulator == "Gazebo Classic")
    {
        packageName = "hunav_gazebo_wrapper";
    }
    else if (simulator == "Gazebo Fortress")
    {
        packageName = "hunav_gazebo_fortress_wrapper";
    }
    else if (simulator == "Isaac Sim")
    {
        packageName = "hunav_isaac_wrapper";
    }
    else
    {
        packageName = "hunav_webots_wrapper";
    }

    QString btDir;
    try
    {
        QString shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory(packageName.toStdString()));
        
        std::vector<std::string> parts;
        std::stringstream ss(shareDir.toStdString());
        std::string item;
        while (std::getline(ss, item, '/')) {
            if (!item.empty()) parts.push_back(item);
        }
        
        auto it = std::find(parts.begin(), parts.end(), "install");
        if (it == parts.end() || (it + 1) == parts.end()) {
            qWarning() << "Path does not have expected structure ../install/share/package_name:" << shareDir;
            btDir = shareDir + "/behavior_trees";
        } else {
            size_t install_idx = std::distance(parts.begin(), it);
            std::string pkg_name = parts[install_idx + 1];
            
            std::ostringstream src_path;
            for (size_t i = 0; i < install_idx; ++i) {
                src_path << "/" << parts[i];
            }
            // src_path << "/src/" << pkg_name;
            src_path << "/src/"; // Temporary for local setup (not docker)
            
            btDir = QString::fromStdString(src_path.str() + "/behavior_trees");
            qDebug() << "Converted share path to src path:" << shareDir << "->" << btDir;
        }
    }
    catch (const std::exception &e)
    {
        // Package not found - use fallback paths
        qWarning() << "Could not find package" << packageName << ":" << e.what();
        QString homePath = QDir::homePath() + "/" + packageName + "/behavior_trees";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName + "/behavior_trees";
        btDir = QDir(dockerPath).exists() ? dockerPath : homePath;
    }

    QStringList paths;
    for (int i = 0; i < agentCount; ++i)
    {
        QString fileName = QString("%1__agent_%2_bt.xml").arg(scenarioName).arg(i + 1);
        paths.append(btDir + "/" + fileName);
    }
    return paths;
}

bool BTConfigDialog::patchAllAgentBTFiles(const QStringList &btPaths, const Config &config)
{
    qDebug() << "patchAllAgentBTFiles called with" << btPaths.size() << "files";
    bool allSuccess = true;
    for (int i = 0; i < btPaths.size(); ++i)
    {
        qDebug() << "Processing BT file" << (i+1) << "of" << btPaths.size() << ":" << btPaths[i];
        if (!patchAgentBtFile(btPaths[i], i, config))
        {
            qWarning() << "Failed to patch BT file for agent" << (i+1);
            allSuccess = false;
        }
        else
        {
            qDebug() << "Successfully patched BT file for agent" << (i+1);
        }
    }
    qDebug() << "patchAllAgentBTFiles completed. Success:" << allSuccess;
    return allSuccess;
}

bool BTConfigDialog::patchAgentBtFile(const QString &btPath, int agentIndex, const Config &cfg)
{
    qDebug() << "patchAgentBtFile called for agent" << (agentIndex+1) << "file:" << btPath;
    
    if (btPath.isEmpty() || !QFile::exists(btPath))
    {
        qWarning() << "BT file does not exist:" << btPath;
        return false;
    }

    qDebug() << "Loading XML file:" << btPath;
    tinyxml2::XMLDocument doc;
    std::string btPath_str = btPath.toStdString();
    if (doc.LoadFile(btPath_str.c_str()) != tinyxml2::XML_SUCCESS)
    {
        qWarning() << "Cannot parse BT file:" << btPath << "-" << doc.ErrorStr();
        return false;
    }

    // Find root element
    auto *root = doc.FirstChildElement();
    if (!root)
    {
        qWarning() << "No root element found in BT file:" << btPath;
        return false;
    }

    // Look for TreeNodesModel structure or direct BehaviorTree elements
    auto *modelRoot = doc.FirstChildElement("TreeNodesModel");
    if (modelRoot)
    {
        root = modelRoot; // Use TreeNodesModel as root if present
    }

    // Find any BehaviorTree element
    tinyxml2::XMLElement *bt = nullptr;

    // First, try to find a BehaviorTree anywhere in the document
    std::function<tinyxml2::XMLElement *(tinyxml2::XMLElement *)> findBehaviorTree =
        [&](tinyxml2::XMLElement *parent) -> tinyxml2::XMLElement *
    {
        for (auto *elem = parent->FirstChildElement(); elem; elem = elem->NextSiblingElement())
        {
            if (strcmp(elem->Name(), "BehaviorTree") == 0)
            {
                return elem;
            }
            // Recursively search in children
            if (auto *found = findBehaviorTree(elem))
            {
                return found;
            }
        }
        return nullptr;
    };

    bt = findBehaviorTree(root);

    if (!bt)
    {
        qWarning() << "No BehaviorTree element found in file" << btPath;
        return false;
    }

    const char *btId = bt->Attribute("ID");
    qDebug() << "Found BehaviorTree with ID:" << (btId ? btId : "unnamed") << "for agent" << (agentIndex + 1);

    // Find the main container
    tinyxml2::XMLElement *mainContainer = bt->FirstChildElement();
    if (!mainContainer)
    {
        qWarning() << "BehaviorTree has no child elements for agent" << (agentIndex + 1);
        return false;
    }

    qDebug() << "Using main container:" << mainContainer->Name() << "for agent" << (agentIndex + 1);

    // Find SetGoals sequence
    tinyxml2::XMLElement *setGoals = nullptr;

    std::function<tinyxml2::XMLElement *(tinyxml2::XMLElement *)> findSetGoals =
        [&](tinyxml2::XMLElement *parent) -> tinyxml2::XMLElement *
    {
        for (auto *elem = parent->FirstChildElement(); elem; elem = elem->NextSiblingElement())
        {
            const char *name = elem->Attribute("name");
            if (name && QString(name) == "SetGoals")
            {
                return elem;
            }
            // Recursively search in children
            if (auto *found = findSetGoals(elem))
            {
                return found;
            }
        }
        return nullptr;
    };

    setGoals = findSetGoals(mainContainer);

    if (!setGoals)
    {
        qDebug() << "SetGoals sequence not found - will create one and insert at beginning";
        // Create SetGoals sequence
        setGoals = doc.NewElement("Sequence");
        setGoals->SetAttribute("name", "SetGoals");
        auto *comment = doc.NewComment(" SetGoals sequence created by BT Config Dialog ");
        setGoals->InsertEndChild(comment);
        mainContainer->InsertFirstChild(setGoals);
    }
    else
    {
        qDebug() << "Found existing SetGoals sequence";
    }

    std::vector<tinyxml2::XMLElement *> toRemove;
    for (auto *child = mainContainer->FirstChildElement(); child; child = child->NextSiblingElement())
    {
        const char *name = child->Attribute("name");
        if (name)
        {
            QString childName = QString(name);
            // Keep SetGoals, RegNav, and RegularNavigation, remove everything else
            if (childName != "SetGoals" && childName != "RegNav" && childName != "RegularNavigation")
            {
                toRemove.push_back(child);
                qDebug() << "Marking for removal:" << childName;
            }
        }
        else
        {
            // Check element type - preserve UpdateGoal action nodes
            const char *elemType = child->Name();
            if (strcmp(elemType, "UpdateGoal") == 0)
            {
                qDebug() << "Keeping UpdateGoal action node";
            }
            else if (strcmp(child->Name(), "Sequence") == 0)
            {
                // Remove unnamed sequences (they shouldn't be there)
                toRemove.push_back(child);
                qDebug() << "Marking unnamed sequence for removal";
            }
        }

        // Also check for comment markers we inserted
        if (child->ToComment())
        {
            QString commentText = child->Value();
            if (commentText.contains("Block:") || commentText.contains("BT Config Dialog"))
            {
                auto *next = child->NextSibling();
                if (next && next->ToElement())
                {
                    toRemove.push_back(next->ToElement());
                }
            }
        }
    }

    for (auto *elem : toRemove)
    {
        const char *elemName = elem->Attribute("name");
        qDebug() << "Removing existing sequence:" << (elemName ? elemName : elem->Name());
        mainContainer->DeleteChild(elem);
    }

    // Get agent assignment from config
    if (agentIndex >= cfg.agentAssignments.size())
    {
        qDebug() << "No custom assignment found for agent" << (agentIndex + 1) << "- keeping default behavior";
        return true;
    }

    const auto &assignment = cfg.agentAssignments[agentIndex];

    if (assignment.assignedBlocks.isEmpty())
    {
        qDebug() << "No blocks assigned to agent" << (agentIndex + 1) << "- keeping default behavior";
        return true;
    }

    // Find RegNav or RegularNavigation sequence to ensure we insert blocks before it
    tinyxml2::XMLElement *regNavSeq = nullptr;
    std::function<tinyxml2::XMLElement *(tinyxml2::XMLElement *)> findRegNav =
        [&](tinyxml2::XMLElement *parent) -> tinyxml2::XMLElement *
    {
        for (auto *elem = parent->FirstChildElement(); elem; elem = elem->NextSiblingElement())
        {
            const char *name = elem->Attribute("name");
            if (name && (QString(name) == "RegNav" || QString(name) == "RegularNavigation"))
            {
                return elem;
            }
            // Recursively search in children
            if (auto *found = findRegNav(elem))
            {
                return found;
            }
        }
        return nullptr;
    };

    regNavSeq = findRegNav(mainContainer);

    if (regNavSeq)
    {
        const char *foundName = regNavSeq->Attribute("name");
        qDebug() << "Found" << foundName << "sequence - will insert custom blocks before it";
    }
    else
    {
        qDebug() << "RegNav/RegularNavigation sequence not found - will insert custom blocks after SetGoals";
    }

    // Insert each selected block for this agent after SetGoals but before RegNav
    tinyxml2::XMLElement *insertAfter = setGoals;

    for (const QString &blockId : assignment.assignedBlocks)
    {
        qDebug() << "Processing block:" << blockId << "for agent" << (agentIndex + 1);
        
        // Find the block definition
        auto blockDef = std::find_if(cfg.availableBlocks.begin(), cfg.availableBlocks.end(),
                                     [&blockId](const auto &block)
                                     { return block.blockId == blockId; });

        if (blockDef == cfg.availableBlocks.end())
        {
            qWarning() << "Block" << blockId << "not found in available blocks";
            continue;
        }

        qDebug() << "Creating XML container for block:" << blockId;
        
        // Create container for this block
        auto *blockSeq = doc.NewElement("Sequence");
        std::string name_str = ((blockId + "Block")).toStdString();
        blockSeq->SetAttribute("name", name_str.c_str());

        // Add comment for clarity
        std::string comment_str = (QString(" Block: %1 - %2 ").arg(blockId, blockDef->description)).toStdString();
        auto *comment = doc.NewComment(comment_str.c_str());
        blockSeq->InsertEndChild(comment);

        // Get nodes for this block
        QStringList nodes = getEnhancedNodesForBlock(blockId);
        
        // Special handling for TalkInteract based on interaction mode
        if (blockId == "TalkInteract")
        {
            // Use the param key format (with dot) to match how it's stored
            QString modeKey = QString("%1.interaction_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
            
            if (mode == "specific_target")
            {
                // Option B: Specific Target - Check specific agent(s)
                // Note: For multiple participants, we'll create multiple IsAgentVisible/IsAgentClose nodes
                // with different agent_id values during node creation
                nodes = QStringList({"IsAgentVisible", "IsAgentClose", "ConversationFormation"});
            }
            else // find_nearest
            {
                // Option C: Find Nearest - Dynamic discovery with looking
                nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "LookAtAgent", "ConversationFormation"});
            }
        }
        // Special handling for FollowAgent based on interaction mode
        else if (blockId == "FollowAgent")
        {
            QString modeKey = QString("%1.interaction_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
            
            if (mode == "specific_target")
            {
                // Specific Target - Follow a specific agent
                nodes = QStringList({"IsAgentVisible", "IsAgentClose", "FollowAgent"});
            }
            else // find_nearest
            {
                // Find Nearest - Dynamically find and follow nearest agent
                nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "IsAgentClose", "FollowAgent"});
            }
        }
        // Special handling for AttentionSeeking based on attention mode
        else if (blockId == "AttentionSeeking")
        {
            QString modeKey = QString("%1.attention_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "agent_attention").toString();
            
            if (mode == "robot_attention")
            {
                // Robot Attention - Detect robot looking and respond
                nodes = QStringList({"IsRobotFacingAgent", "LookAtRobot", "SaySomething"});
            }
            else // agent_attention
            {
                // Agent Attention - Detect other agents looking and respond
                nodes = QStringList({"IsAnyoneLookingAtMe", "LookAtAgent", "SaySomething"});
            }
        }
        // Special handling for GreetingInitiator based on greeting mode
        else if (blockId == "GreetingInitiator")
        {
            QString modeKey = QString("%1.greeting_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "specific_agent").toString();
            
            qDebug() << "GreetingInitiator XML Generation: modeKey=" << modeKey << "mode=" << mode;
            qDebug() << "Available params:" << assignment.agentSpecificParams.keys();
            
            // Map old mode names for backward compatibility
            if (mode == "specific_target")
                mode = "specific_agent";
            else if (mode == "find_nearest")
                mode = "nearest_agent";
            
            if (mode == "greet_robot")
            {
                // Greet Robot - Detect, look at, and greet robot
                nodes = QStringList({"IsRobotVisible", "LookAtRobot", "SaySomething"});
            }
            else if (mode == "nearest_agent")
            {
                // Greet Nearest Agent - Dynamically find, look at, and greet nearest agent
                nodes = QStringList({"FindNearestAgent", "IsAgentVisible", "LookAtAgent", "SaySomething"});
            }
            else // specific_agent
            {
                // Greet Specific Agent - Detect, look at, and greet specific agent
                nodes = QStringList({"IsAgentVisible", "LookAtAgent", "SaySomething"});
            }
        }
        // Special handling for ProtectiveGuardian based on protection mode
        else if (blockId == "ProtectiveGuardian")
        {
            QString modeKey = QString("%1.protection_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
            
            if (mode == "protect_nearest_threatened")
            {
                // Protect Nearest Threatened - Dynamically find any threatened agent
                nodes = QStringList({"IsRobotVisible", "FindNearestAgent", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"});
            }
            else // specific_protected
            {
                // Specific Protected - Protect specific agent from robot
                nodes = QStringList({"IsRobotVisible", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"});
            }
        }

        for (const QString &nodeId : nodes)
        {
            // Check if this is a RetryUntilSuccessful wrapper
            if (nodeId.startsWith("RetryUntilSuccessful:"))
            {
                QString innerContent = nodeId.mid(21); // Remove "RetryUntilSuccessful:"
                
                // Create RetryUntilSuccessful element
                auto *retryElem = doc.NewElement("RetryUntilSuccessful");
                retryElem->SetAttribute("num_attempts", "-1"); // Infinite retries
                
                // Check if there's an Inverter inside
                if (innerContent.startsWith("Inverter:"))
                {
                    QString wrappedNodeId = innerContent.section(':', 1);
                    
                    // Create Inverter element
                    auto *inverterElem = doc.NewElement("Inverter");
                    
                    // Create the wrapped condition
                    auto *conditionElem = doc.NewElement("Condition");
                    std::string id_str = (wrappedNodeId).toStdString();
                    conditionElem->SetAttribute("ID", id_str.c_str());
                    conditionElem->SetAttribute("agent_id", "{id}");
                    
                    // Apply parameters
                    applyBlockParametersToNode(conditionElem, blockId, wrappedNodeId, cfg);
                    applyAgentSpecificParametersToNode(conditionElem, agentIndex, blockId, wrappedNodeId, assignment);
                    
                    // Build the nested structure
                    inverterElem->InsertEndChild(conditionElem);
                    retryElem->InsertEndChild(inverterElem);
                }
                
                blockSeq->InsertEndChild(retryElem);
                continue;
            }
            
            // Check if this is a KeepRunningUntilFailure wrapper
            if (nodeId.startsWith("KeepRunningUntilFailure:"))
            {
                QString innerContent = nodeId.mid(24); // Remove "KeepRunningUntilFailure:"
                
                // Create KeepRunningUntilFailure element
                auto *keepRunningElem = doc.NewElement("KeepRunningUntilFailure");
                
                // Check if there's an Inverter inside
                if (innerContent.startsWith("Inverter:"))
                {
                    QString wrappedNodeId = innerContent.section(':', 1);
                    
                    // Create Inverter element
                    auto *inverterElem = doc.NewElement("Inverter");
                    
                    // Create the wrapped condition
                    auto *conditionElem = doc.NewElement("Condition");
                    std::string id_str = (wrappedNodeId).toStdString();
                    conditionElem->SetAttribute("ID", id_str.c_str());
                    conditionElem->SetAttribute("agent_id", "{id}");
                    
                    // Apply parameters
                    applyBlockParametersToNode(conditionElem, blockId, wrappedNodeId, cfg);
                    applyAgentSpecificParametersToNode(conditionElem, agentIndex, blockId, wrappedNodeId, assignment);
                    
                    // Build the nested structure
                    inverterElem->InsertEndChild(conditionElem);
                    keepRunningElem->InsertEndChild(inverterElem);
                }
                
                blockSeq->InsertEndChild(keepRunningElem);
                continue;
            }
            
            // Check if this is an Inverter wrapper
            if (nodeId.startsWith("Inverter:"))
            {
                QString wrappedNodeId = nodeId.section(':', 1);

                // Create simple Inverter element
                auto *inverterElem = doc.NewElement("Inverter");

                // Create the wrapped condition
                auto *conditionElem = doc.NewElement("Condition");
                std::string id_str = (wrappedNodeId).toStdString();
                conditionElem->SetAttribute("ID", id_str.c_str());
                conditionElem->SetAttribute("agent_id", "{id}");

                // Apply parameters to the wrapped condition
                applyBlockParametersToNode(conditionElem, blockId, wrappedNodeId, cfg);
                applyAgentSpecificParametersToNode(conditionElem, agentIndex, blockId, wrappedNodeId, assignment);

                // Add wrapped condition to inverter
                inverterElem->InsertEndChild(conditionElem);

                // Check if this block should be wrapped in RunOnce tags
                if (assignment.runOnceBlocks.contains(blockId))
                {
                    // Create RunOnce wrapper for the entire Inverter
                    auto *runOnceElem = doc.NewElement("RunOnce");
                    runOnceElem->InsertEndChild(inverterElem);
                    blockSeq->InsertEndChild(runOnceElem);
                    qDebug() << "Wrapped Inverter" << wrappedNodeId << "in RunOnce for block" << blockId;
                }
                else
                {
                    blockSeq->InsertEndChild(inverterElem);
                }

                continue;
            }

            // Special handling for TalkInteract with multiple participants
            // For multi-participant conversations, we need multiple condition check instances
            if (blockId == "TalkInteract" && (nodeId == "IsAgentVisible" || nodeId == "IsAgentClose"))
            {
                QString modeKey = QString("%1.interaction_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
                
                if (mode == "specific_target")
                {
                    QString participantsKey = blockId + ".non_main_agent_ids";
                    if (assignment.agentSpecificParams.contains(participantsKey))
                    {
                        QString participantsStr = assignment.agentSpecificParams[participantsKey].toString();
                        QStringList participants = participantsStr.split(",", Qt::SkipEmptyParts);
                        
                        if (participants.size() > 1)
                        {
                            // Multiple participants - create one node instance per participant
                            for (const QString &participantId : participants)
                            {
                                QString nodeType = "Condition";
                                std::string nodeType_str = nodeType.toStdString();
                                auto *nodeElem = doc.NewElement(nodeType_str.c_str());
                                std::string id_str = (nodeId).toStdString();
                                nodeElem->SetAttribute("ID", id_str.c_str());
                                
                                // Set participant-specific attributes
                                if (nodeId == "IsAgentVisible")
                                {
                                    nodeElem->SetAttribute("observer_id", "{id}");
                                    std::string agent_id_str = participantId.trimmed().toStdString();
                                    nodeElem->SetAttribute("agent_id", agent_id_str.c_str());
                                    nodeElem->SetAttribute("field_of_view", "3.14");
                                }
                                else if (nodeId == "IsAgentClose")
                                {
                                    nodeElem->SetAttribute("observer_id", "{id}");
                                    std::string target_agent_id_str = participantId.trimmed().toStdString();
                                    nodeElem->SetAttribute("target_agent_id", target_agent_id_str.c_str());
                                }
                                
                                // Apply parameters (distance/threshold from user config)
                                applyBlockParametersToNode(nodeElem, blockId, nodeId, cfg);
                                applyAgentSpecificParametersToNode(nodeElem, agentIndex, blockId, nodeId, assignment);
                                
                                // Add node to sequence
                                if (assignment.runOnceBlocks.contains(blockId))
                                {
                                    auto *runOnceElem = doc.NewElement("RunOnce");
                                    runOnceElem->InsertEndChild(nodeElem);
                                    blockSeq->InsertEndChild(runOnceElem);
                                }
                                else
                                {
                                    blockSeq->InsertEndChild(nodeElem);
                                }
                            }
                            continue; // Skip the normal single-node creation below
                        }
                    }
                }
            }

            // Determine correct XML element type based on node ID
            QString nodeType = "Action"; // Default to Action
            if (nodeId.contains("Is") || nodeId.contains("Condition") || nodeId == "RandomChanceCondition")
            {
                nodeType = "Condition";
            }

            std::string nodeType_str = nodeType.toStdString();
            auto *nodeElem = doc.NewElement(nodeType_str.c_str());
            std::string id_str = (nodeId).toStdString();
            nodeElem->SetAttribute("ID", id_str.c_str());

            // Apply global block parameters and agent-specific parameters
            applyBlockParametersToNode(nodeElem, blockId, nodeId, cfg);
            applyAgentSpecificParametersToNode(nodeElem, agentIndex, blockId, nodeId, assignment);

            // Check if this block should be wrapped in RunOnce tags
            if (assignment.runOnceBlocks.contains(blockId))
            {
                // Create RunOnce wrapper
                auto *runOnceElem = doc.NewElement("RunOnce");
                runOnceElem->InsertEndChild(nodeElem);
                blockSeq->InsertEndChild(runOnceElem);
                qDebug() << "Wrapped" << nodeId << "in RunOnce for block" << blockId;
            }
            else
            {
                // Add node directly
                blockSeq->InsertEndChild(nodeElem);
            }
        }

        // Check if this block should use random execution
        tinyxml2::XMLElement *elementToInsert = blockSeq;
        
        if (assignment.randomExecutionBlocks.contains(blockId))
        {
            // Wrap the entire block sequence in RandomChanceCondition
            double probability = assignment.randomExecutionBlocks[blockId];
            
            auto *randomCondition = doc.NewElement("Condition");
            randomCondition->SetAttribute("ID", "RandomChanceCondition");
            randomCondition->SetAttribute("agent_id", "{id}");
            std::string probability_str = (QString::number(probability, 'f', 2)).toStdString();
            randomCondition->SetAttribute("probability", probability_str.c_str());
            
            // Create a new sequence to contain the random condition + block sequence
            auto *randomWrapper = doc.NewElement("Sequence");
            std::string name_str = ((blockId + "RandomBlock")).toStdString();
            randomWrapper->SetAttribute("name", name_str.c_str());
            
            randomWrapper->InsertEndChild(randomCondition);
            randomWrapper->InsertEndChild(blockSeq);
            
            elementToInsert = randomWrapper;
            
            qDebug() << "Wrapped block" << blockId << "in RandomChanceCondition with probability" << probability;
        }

        // Insert the block sequence (or wrapped version) in the correct position
        if (regNavSeq)
        {
            // Insert before RegNav sequence by finding the element immediately before RegNav
            tinyxml2::XMLElement *beforeRegNav = nullptr;
            for (auto *elem = mainContainer->FirstChildElement(); elem; elem = elem->NextSiblingElement())
            {
                if (elem->NextSiblingElement() == regNavSeq)
                {
                    beforeRegNav = elem;
                    break;
                }
            }

            if (beforeRegNav)
            {
                // Insert after the element before RegNav
                mainContainer->InsertAfterChild(beforeRegNav, elementToInsert);
            }
            else
            {
                // RegNav is the first child, insert at the beginning after SetGoals
                if (setGoals && setGoals != regNavSeq)
                {
                    mainContainer->InsertAfterChild(setGoals, elementToInsert);
                }
                else
                {
                    // Fallback: insert at the beginning of mainContainer
                    auto *firstChild = mainContainer->FirstChildElement();
                    if (firstChild)
                    {
                        mainContainer->InsertAfterChild(firstChild, elementToInsert);
                    }
                    else
                    {
                        mainContainer->InsertFirstChild(elementToInsert);
                    }
                }
            }
        }
        else
        {
            // Fallback: insert after the previous element if RegNav not found
            mainContainer->InsertAfterChild(insertAfter, elementToInsert);
            insertAfter = elementToInsert; // Update for next insertion
        }

        qDebug() << "Inserted" << blockId << "block for agent" << (agentIndex + 1);
    }

    // Save the modified XML
    std::string btPath_save_str = btPath.toStdString();
    if (doc.SaveFile(btPath_save_str.c_str()) != tinyxml2::XML_SUCCESS)
    {
        qWarning() << "Failed to save patched BT:" << btPath << "-" << doc.ErrorStr();
        return false;
    }

    qDebug() << "Successfully patched BT file:" << btPath << "for agent" << (agentIndex + 1);
    return true;
}

// Helper method to apply block parameters to XML node - using configuration values
void BTConfigDialog::applyBlockParametersToNode(tinyxml2::XMLElement *nodeElem,
                                                const QString &blockId,
                                                const QString &nodeId,
                                                const BTConfigDialog::Config & /*cfg*/)
{
    if (blockId == "FollowAgent")
    {
        if (nodeId == "FindNearestAgent")
        {
            // FindNearestAgent only needs agent_id (input) and outputs target_agent_id
            // No additional parameters needed - it finds the nearest agent automatically
        }
        else if (nodeId == "FollowAgent")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
        }
        else if (nodeId == "IsAgentVisible")
        {
            nodeElem->SetAttribute("distance", "10.0");
            nodeElem->SetAttribute("field_of_view", "3.14");
        }
        else if (nodeId == "IsAgentClose")
        {
            nodeElem->SetAttribute("threshold", "1.0");
        }
    }
    else if (blockId == "EngageRobot")
    {
        if (nodeId == "IsRobotVisible")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            nodeElem->SetAttribute("distance", "5.0");
        }
        else if (nodeId == "ApproachRobot")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            nodeElem->SetAttribute("closest_dist", "1.5");
            nodeElem->SetAttribute("max_vel", "1.8");
            nodeElem->SetAttribute("duration", "5.0");
        }
    }
    else if (blockId == "TalkInteract")
    {
        if (nodeId == "FindNearestAgent")
        {
            // FindNearestAgent only needs agent_id (input) and outputs target_agent_id
            // No additional parameters needed - it finds the nearest agent automatically
        }
        else if (nodeId == "IsAgentVisible")
        {
            nodeElem->SetAttribute("distance", "10.0");
            nodeElem->SetAttribute("field_of_view", "3.14");
        }
        else if (nodeId == "IsAgentClose")
        {
            nodeElem->SetAttribute("threshold", "2.0");
        }
        else if (nodeId == "LookAtAgent")
        {
            nodeElem->SetAttribute("yaw_tolerance", "0.1");
        }
        else if (nodeId == "IsAnyoneSpeaking")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            nodeElem->SetAttribute("distance_threshold", "5.0");
            nodeElem->SetAttribute("duration", "5.0");
        }
        else if (nodeId == "ConversationFormation")
        {
            nodeElem->SetAttribute("conversation_duration", "15.0");
            nodeElem->SetAttribute("time_step", "{dt}");
        }
    }
    else if (blockId == "RobotAvoidance")
    {
        if (nodeId == "IsRobotVisible")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            nodeElem->SetAttribute("distance", "5.0");
        }
        else if (nodeId == "IsRobotClose")
        {
            nodeElem->SetAttribute("threshold", "2.0");
        }
    }
    else if (blockId == "LookAround")
    {
    }
    else if (blockId == "RandomExplore")
    {
        if (nodeId == "RandomChanceCondition")
        {
            nodeElem->SetAttribute("probability", "0.3");
        }
        else if (nodeId == "GoTo")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
        }
    }
    else if (blockId == "BlockRobot")
    {
        if (nodeId == "IsRobotVisible")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            nodeElem->SetAttribute("distance", "5.0");
        }
        else if (nodeId == "BlockRobot")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            nodeElem->SetAttribute("front_dist", "1.0");
            nodeElem->SetAttribute("duration", "30.0");
        }
    }
    else if (blockId == "StopAndWait")
    {
        if (nodeId == "StopAndWaitTimerAction")
        {
            nodeElem->SetAttribute("wait_duration", "10.0");
        }
    }
    else if (blockId == "BlockingBehavior")
    {
        if (nodeId == "IsRobotVisible")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            nodeElem->SetAttribute("distance", "5.0");
        }
        else if (nodeId == "BlockRobot")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            nodeElem->SetAttribute("front_dist", "1.0");
            nodeElem->SetAttribute("duration", "5.0");
        }
    }
    else if (blockId == "GroupFormation")
    {
        if (nodeId == "SetGroupWalk")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
        }
    }
    else if (blockId == "GreetingInitiator")
    {
        if (nodeId == "IsAgentVisible")
        {
            nodeElem->SetAttribute("field_of_view", "3.14");
            // distance will be applied from user parameters
        }
        else if (nodeId == "SaySomething")
        {
            // greeting_message will be applied from user parameters
        }
    }
    else if (blockId == "ProtectiveGuardian")
    {
        if (nodeId == "IsRobotVisible")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            // robot_detection_distance will be applied from user parameters
        }
        else if (nodeId == "IsAgentVisible")
        {
            nodeElem->SetAttribute("field_of_view", "3.14");
            // agent_visibility_distance will be applied from user parameters
        }
        else if (nodeId == "IsRobotClose")
        {
            // robot_close_threshold will be applied from user parameters
        }
        else if (nodeId == "LookAtAgent")
        {
            // Instantaneous action, no parameters needed
        }
        else if (nodeId == "ApproachAgent")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            // protective_approach_distance, protective_approach_velocity will be applied from user parameters
        }
        else if (nodeId == "BlockRobot")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            // protective_front_distance, blocking_duration will be applied from user parameters
        }
    }
    else if (blockId == "SpeechDetection")
    {
        if (nodeId == "IsAnyoneSpeaking")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            // distance_threshold and duration will be set from agent-specific parameters
        }
        else if (nodeId == "ApproachAgent")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            // closest_dist, max_vel, and duration will be set from agent-specific parameters
        }
        else if (nodeId == "ConversationFormation")
        {
            // All parameters will be set in applyAgentSpecificParametersToNode
        }
    }
    else if (blockId == "AttentionSeeking")
    {
        if (nodeId == "IsAnyoneLookingAtMe")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
            nodeElem->SetAttribute("angle_threshold", "0.2");
            // distance_threshold and duration will be applied from user parameters
        }
        else if (nodeId == "LookAtAgent")
        {
            // LookAtAgent has no time_step or duration - it's an instantaneous action
        }
        else if (nodeId == "SaySomething")
        {
            // response_message will be applied from user parameters
        }
    }
}

// Helper method to apply agent-specific parameters
void BTConfigDialog::applyAgentSpecificParametersToNode(tinyxml2::XMLElement *nodeElem,
                                                        int /*agentIndex*/,
                                                        const QString &blockId,
                                                        const QString &nodeId,
                                                        const BTConfigDialog::AgentAssignment &assignment)
{
    try
    {
        if (!nodeElem)
        {
            qWarning() << "Null nodeElem passed to applyAgentSpecificParametersToNode";
            return;
        }

        // Apply agent-specific parameters from configuration widgets
        for (auto it = assignment.agentSpecificParams.begin(); it != assignment.agentSpecificParams.end(); ++it)
        {
            try
            {
                QString paramKey = it.key();

                // Map parameter keys to XML attributes based on block and node context
                if (paramKey.startsWith(blockId + "."))
                {
                    QString param = paramKey.section('.', 1);

                    // Only apply parameters that are relevant to this specific node
                    bool shouldApplyParameter = false;

                    if (nodeId == "IsRobotVisible" && (param == "detection_distance" || param == "robot_detection_distance"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsRobotClose" && (param == "close_threshold" || param == "proximity_threshold" || param == "robot_close_threshold"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsAgentVisible" && (param == "target_agent_id" || param == "visibility_distance" || param == "agent_visibility_distance" || param == "detection_distance"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsAgentClose" && (param == "target_agent_id" || param == "is_close_threshold" || param == "social_distance_threshold"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "FollowAgent" && (param == "target_agent_id" || param == "closest_dist" || param == "following_distance" || param == "max_vel" || param == "duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "ConversationFormation" && (param == "goal_id" || param == "non_main_agent_ids" || param == "conversation_duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "ApproachRobot" && (param == "closest_dist" || param == "approach_distance" || param == "max_vel" || param == "approach_velocity" || param == "duration" || param == "engagement_duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "GoTo" && (param == "goal_id" || param == "tolerance"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "BlockRobot" && (param == "front_dist" || param == "protective_front_distance" || param == "duration" || param == "blocking_duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "SetGroupWalk" && (param == "non_main_agent_ids" || param == "group_agent_ids" || param == "duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsAnyoneSpeaking" && (param == "speech_distance" || param == "speaking_duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "ApproachAgent" && (param == "duration" || param == "interaction_duration" || param == "protective_approach_distance" || param == "protective_approach_velocity"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsAnyoneLookingAtMe" && (param == "attention_distance" || param == "looking_duration"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "IsRobotFacingAgent" && (param == "attention_distance"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "LookAtAgent" && (param == "yaw_tolerance"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "SaySomething" && (param == "response_message" || param == "greeting_message"))
                    {
                        shouldApplyParameter = true;
                    }
                    else if (nodeId == "FindNearestAgent")
                    {
                        // FindNearestAgent doesn't have user-configurable parameters
                        shouldApplyParameter = false;
                    }

                    if (shouldApplyParameter)
                    {
                        QString xmlAttr = mapParameterToXMLAttribute(blockId, nodeId, param);
                        if (!xmlAttr.isEmpty())
                        {
                            QString value = it.value().toString();
                            if (!value.isEmpty())
                            {
                                // Convert to std::string first to avoid dangling pointer from temporary
                                std::string attrName = xmlAttr.toStdString();
                                std::string attrValue = value.toStdString();
                                nodeElem->SetAttribute(attrName.c_str(), attrValue.c_str());
                                qDebug() << "Applied parameter:" << xmlAttr << "=" << value << "for node" << nodeId;
                            }
                        }
                        else
                        {
                            qWarning() << "No XML attribute mapping found for parameter:" << param << "on node:" << nodeId;
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                qWarning() << "Exception processing parameter" << it.key() << "for node" << nodeId << ":" << e.what();
            }
            catch (...)
            {
                qWarning() << "Unknown exception processing parameter" << it.key() << "for node" << nodeId;
            }
        }
    }
    catch (const std::exception &e)
    {
        qWarning() << "Exception in applyAgentSpecificParametersToNode:" << e.what();
    }
    catch (...)
    {
        qWarning() << "Unknown exception in applyAgentSpecificParametersToNode for node" << nodeId;
    }

    // Apply agent IDs
    try
    {
        // Special handling for GreetingInitiator block based on greeting mode
        if (blockId == "GreetingInitiator")
        {
            QString modeKey = QString("%1.greeting_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "specific_agent").toString();
            
            // Map old mode names for backward compatibility
            if (mode == "specific_target")
                mode = "specific_agent";
            else if (mode == "find_nearest")
                mode = "nearest_agent";
            
            if (nodeId == "FindNearestAgent")
            {
                // FindNearestAgent uses agent_id (input) and outputs target_agent_id
                nodeElem->SetAttribute("agent_id", "{id}");
                nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
            }
            else if (nodeId == "IsAgentVisible")
            {
                nodeElem->SetAttribute("observer_id", "{id}");
                nodeElem->SetAttribute("field_of_view", "3.14");
                
                if (mode == "nearest_agent")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                }
                else // specific_agent mode
                {
                    // Use the selected target agent
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_agent_id_str = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("agent_id", target_agent_id_str.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("agent_id", "2"); // Default fallback
                    }
                }
            }
            else if (nodeId == "LookAtAgent")
            {
                nodeElem->SetAttribute("observer_id", "{id}");
                
                if (mode == "nearest_agent")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("target_id", "{target_agent_id}");
                }
                else // specific_agent mode
                {
                    // Use the selected target agent
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_agent_id_str = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("target_id", target_agent_id_str.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("target_id", "2"); // Default fallback
                    }
                }
            }
            else if (nodeId == "IsRobotVisible")
            {
                // IsRobotVisible for greet_robot mode
                nodeElem->SetAttribute("agent_id", "{id}");
                
                // Apply user-configured detection distance
                if (assignment.agentSpecificParams.contains(blockId + ".detection_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".detection_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance", "6.0");
                }
            }
            else if (nodeId == "LookAtRobot")
            {
                // LookAtRobot for greet_robot mode
                nodeElem->SetAttribute("agent_id", "{id}");
            }
            else if (nodeId == "SaySomething")
            {
                // SaySomething is used in all modes
                nodeElem->SetAttribute("agent_id", "{id}");
                
                // message parameter is handled by the general parameter application above
            }
        }
        // Special handling for TalkInteract block based on interaction mode
        else if (blockId == "TalkInteract")
        {
            QString modeKey = QString("%1.interaction_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
            
            if (nodeId == "FindNearestAgent")
            {
                // FindNearestAgent uses agent_id (input) and outputs target_agent_id
                nodeElem->SetAttribute("agent_id", "{id}");
                nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
                // No other parameters needed
            }
            else if (nodeId == "IsAgentVisible")
            {
                // Note: For multi-participant mode, participant-specific attributes are already set
                // during node creation. Skip if agent_id is already set.
                if (!nodeElem->Attribute("agent_id"))
                {
                    nodeElem->SetAttribute("observer_id", "{id}");
                    nodeElem->SetAttribute("field_of_view", "3.14");
                    if (mode == "find_nearest")
                    {
                        // Use target_agent_id from FindNearestAgent output
                        nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                    }
                    else // specific_target mode - single participant
                    {
                        // Use the single conversation participant
                        if (assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
                        {
                            QString participantsStr = assignment.agentSpecificParams[blockId + ".non_main_agent_ids"].toString();
                            QStringList participants = participantsStr.split(",", Qt::SkipEmptyParts);
                            if (!participants.isEmpty())
                            {
                                std::string agent_id_str = participants.first().trimmed().toStdString();
                                nodeElem->SetAttribute("agent_id", agent_id_str.c_str());
                            }
                            else
                            {
                                nodeElem->SetAttribute("agent_id", "2"); // Default fallback
                            }
                        }
                        else
                        {
                            nodeElem->SetAttribute("agent_id", "2"); // Default fallback
                        }
                    }
                }
            }
            else if (nodeId == "IsAgentClose")
            {
                // Note: For multi-participant mode, participant-specific attributes are already set
                // during node creation. Skip if target_agent_id is already set.
                if (!nodeElem->Attribute("target_agent_id"))
                {
                    nodeElem->SetAttribute("observer_id", "{id}");
                    if (mode == "find_nearest")
                    {
                        // Use target_agent_id from FindNearestAgent output
                        nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
                    }
                    else // specific_target mode - single participant
                    {
                        // Use the single conversation participant
                        if (assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
                        {
                            QString participantsStr = assignment.agentSpecificParams[blockId + ".non_main_agent_ids"].toString();
                            QStringList participants = participantsStr.split(",", Qt::SkipEmptyParts);
                            if (!participants.isEmpty())
                            {
                                std::string target_agent_id_str = participants.first().trimmed().toStdString();
                                nodeElem->SetAttribute("target_agent_id", target_agent_id_str.c_str());
                            }
                            else
                            {
                                nodeElem->SetAttribute("target_agent_id", "2"); // Default fallback
                            }
                        }
                        else
                        {
                            nodeElem->SetAttribute("target_agent_id", "2"); // Default fallback
                        }
                    }
                }
            }
            else if (nodeId == "LookAtAgent")
            {
                // Used in find_nearest mode
                // LookAtAgent uses observer_id and target_id (from TreeNodesModel.xml)
                nodeElem->SetAttribute("observer_id", "{id}");
                nodeElem->SetAttribute("target_id", "{target_agent_id}");
            }
            else if (nodeId == "ConversationFormation")
            {
                // ConversationFormation uses main_agent_id (from TreeNodesModel.xml)
                nodeElem->SetAttribute("main_agent_id", "{id}");
                nodeElem->SetAttribute("time_step", "{dt}");
                
                // goal_id comes from user selection in the wizard
                if (assignment.agentSpecificParams.contains(blockId + ".goal_id"))
                {
                    std::string goal_id_value = assignment.agentSpecificParams[blockId + ".goal_id"].toString().toStdString();
                    nodeElem->SetAttribute("goal_id", goal_id_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("goal_id", "1");
                }
                
                // non_main_agent_ids depends on the interaction mode
                if (mode == "find_nearest")
                {
                    // For find_nearest mode, use the target_agent_id from FindNearestAgent
                    nodeElem->SetAttribute("non_main_agent_ids", "{target_agent_id}");
                }
                else // specific_target mode
                {
                    // For specific_target mode, use conversation participants parameter
                    if (assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
                    {
                        std::string non_main_agent_ids_value = assignment.agentSpecificParams[blockId + ".non_main_agent_ids"].toString().toStdString();
                        nodeElem->SetAttribute("non_main_agent_ids", non_main_agent_ids_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("non_main_agent_ids", "2");
                    }
                }
            }
            // Continue to regular agent ID handling below for any other nodes
            else if (nodeId != "FindNearestAgent" && nodeId != "IsAgentVisible" && 
                     nodeId != "IsAgentClose" && nodeId != "LookAtAgent" && 
                     nodeId != "ConversationFormation")
            {
                nodeElem->SetAttribute("agent_id", "{id}");
            }
        }
        // Special handling for FollowAgent block based on interaction mode
        else if (blockId == "FollowAgent")
        {
            QString modeKey = QString("%1.interaction_mode").arg(blockId);
            QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
            
            if (nodeId == "FindNearestAgent")
            {
                // FindNearestAgent uses agent_id (input) and outputs target_agent_id
                nodeElem->SetAttribute("agent_id", "{id}");
                nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
            }
            else if (nodeId == "IsAgentVisible")
            {
                nodeElem->SetAttribute("observer_id", "{id}");
                if (mode == "find_nearest")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                }
                else // specific_target mode
                {
                    // Use specific target from user selection
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("agent_id", agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("agent_id", "2"); // Default fallback
                    }
                }
                // Apply user-configured visibility distance
                if (assignment.agentSpecificParams.contains(blockId + ".visibility_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".visibility_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
            }
            else if (nodeId == "IsAgentClose")
            {
                nodeElem->SetAttribute("observer_id", "{id}");
                if (mode == "find_nearest")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
                }
                else // specific_target mode
                {
                    // Use specific target from user selection
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("target_agent_id", target_agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("target_agent_id", "2"); // Default fallback
                    }
                }
                // Apply user-configured close threshold
                if (assignment.agentSpecificParams.contains(blockId + ".is_close_threshold"))
                {
                    std::string threshold_value = assignment.agentSpecificParams[blockId + ".is_close_threshold"].toString().toStdString();
                    nodeElem->SetAttribute("threshold", threshold_value.c_str());
                }
            }
            else if (nodeId == "FollowAgent")
            {
                nodeElem->SetAttribute("agent_id", "{id}");
                if (mode == "find_nearest")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
                }
                else // specific_target mode
                {
                    // Use specific target from user selection
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("target_agent_id", target_agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("target_agent_id", "2"); // Default fallback
                    }
                }
                // Apply user-configured parameters
                if (assignment.agentSpecificParams.contains(blockId + ".following_distance"))
                {
                    std::string closest_dist_value = assignment.agentSpecificParams[blockId + ".following_distance"].toString().toStdString();
                    nodeElem->SetAttribute("closest_dist", closest_dist_value.c_str());
                }
                if (assignment.agentSpecificParams.contains(blockId + ".max_vel"))
                {
                    std::string max_vel_value = assignment.agentSpecificParams[blockId + ".max_vel"].toString().toStdString();
                    nodeElem->SetAttribute("max_vel", max_vel_value.c_str());
                }
                if (assignment.agentSpecificParams.contains(blockId + ".duration"))
                {
                    std::string duration_value = assignment.agentSpecificParams[blockId + ".duration"].toString().toStdString();
                    nodeElem->SetAttribute("duration", duration_value.c_str());
                }
            }
        }
        else if (nodeId == "IsAgentVisible")
        {
            nodeElem->SetAttribute("observer_id", "{id}");
            
            if (blockId == "GreetingInitiator")
            {
                // Check greeting mode
                QString modeKey = QString("%1.greeting_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_target").toString();
                
                if (mode == "find_nearest")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                }
                else // specific_target
                {
                    // Apply user-selected target agent
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("agent_id", agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("agent_id", "2");
                    }
                }
                
                // Apply user-configured detection distance (used in both modes)
                if (assignment.agentSpecificParams.contains(blockId + ".detection_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".detection_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance", "6.0");
                }
            }
            else if (blockId == "ProtectiveGuardian")
            {
                // Check protection mode
                QString modeKey = QString("%1.protection_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
                
                if (mode == "protect_nearest_threatened")
                {
                    // Use target_agent_id from FindNearestAgent output
                    nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                }
                else // specific_protected
                {
                    // Apply user-selected protected agent
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("agent_id", agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("agent_id", "2");
                    }
                }
                
                // Apply user-configured agent visibility distance (used in both modes)
                if (assignment.agentSpecificParams.contains(blockId + ".agent_visibility_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".agent_visibility_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance", "10.0");
                }
            }
            else if (!assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
            {
                nodeElem->SetAttribute("agent_id", "2");
            }
        }
        else if (nodeId == "IsAgentClose")
        {
            nodeElem->SetAttribute("observer_id", "{id}");
            if (!assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
            {
                nodeElem->SetAttribute("target_agent_id", "2");
            }
        }
        else if (nodeId == "IsRobotClose")
        {
            if (blockId == "ProtectiveGuardian")
            {
                // Check protection mode
                QString modeKey = QString("%1.protection_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
                
                if (mode == "protect_nearest_threatened")
                {
                    // Check if robot is close to the agent found by FindNearestAgent
                    nodeElem->SetAttribute("agent_id", "{target_agent_id}");
                }
                else // specific_protected
                {
                    // Check if robot is close to the protected agent (not to this agent)
                    if (!assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        nodeElem->SetAttribute("agent_id", "2");
                    }
                    else
                    {
                        QString targetAgent = assignment.agentSpecificParams.value(blockId + ".target_agent_id").toString();
                        std::string targetAgent_str = targetAgent.toStdString();
                        nodeElem->SetAttribute("agent_id", targetAgent_str.c_str());
                    }
                }
                
                // Apply user-configured robot close threshold (used in both modes)
                if (assignment.agentSpecificParams.contains(blockId + ".robot_close_threshold"))
                {
                    std::string threshold_value = assignment.agentSpecificParams[blockId + ".robot_close_threshold"].toString().toStdString();
                    nodeElem->SetAttribute("threshold", threshold_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("threshold", "3.0");
                }
            }
            else
            {
                nodeElem->SetAttribute("agent_id", "{id}");
            }
        }
        else if (nodeId == "IsRobotVisible")
        {
            // Generic robot visibility check - needs agent_id and distance
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "GreetingInitiator")
            {
                // Apply user-configured detection distance
                if (assignment.agentSpecificParams.contains(blockId + ".detection_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".detection_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance", "6.0");
                }
            }
            else if (blockId == "ProtectiveGuardian")
            {
                // Apply user-configured robot detection distance
                if (assignment.agentSpecificParams.contains(blockId + ".robot_detection_distance"))
                {
                    std::string distance_value = assignment.agentSpecificParams[blockId + ".robot_detection_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance", distance_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance", "15.0");
                }
            }
        }
        else if (nodeId == "LookAtRobot")
        {
            // LookAtRobot needs agent_id
            nodeElem->SetAttribute("agent_id", "{id}");
        }
        else if (nodeId == "BlockRobot")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "ProtectiveGuardian")
            {
                // Apply user-configured protective front distance
                if (assignment.agentSpecificParams.contains(blockId + ".protective_front_distance"))
                {
                    std::string front_dist_value = assignment.agentSpecificParams[blockId + ".protective_front_distance"].toString().toStdString();
                    nodeElem->SetAttribute("front_dist", front_dist_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("front_dist", "1.2");
                }
                
                // Apply user-configured blocking duration
                if (assignment.agentSpecificParams.contains(blockId + ".blocking_duration"))
                {
                    std::string duration_value = assignment.agentSpecificParams[blockId + ".blocking_duration"].toString().toStdString();
                    nodeElem->SetAttribute("duration", duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("duration", "20.0");
                }
            }
        }
        else if (nodeId == "LookAtAgent")
        {
            nodeElem->SetAttribute("observer_id", "{id}");

            if (blockId == "AttentionSeeking")
            {
                // For AttentionSeeking, look at whoever was looking at us
                nodeElem->SetAttribute("target_id", "{observer_id}");
            }
            else if (blockId == "ProtectiveGuardian")
            {
                // Check protection mode
                QString modeKey = QString("%1.protection_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
                
                if (mode == "protect_nearest_threatened")
                {
                    // Look at the agent found by FindNearestAgent
                    nodeElem->SetAttribute("target_id", "{target_agent_id}");
                }
                else // specific_protected
                {
                    // For ProtectiveGuardian, look at the protected agent
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("target_id", target_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("target_id", "2");
                    }
                }
            }
            else if (!assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
            {
                nodeElem->SetAttribute("target_id", "2");
            }
        }
        else if (nodeId == "FollowAgent")
        {
            if (!assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
            {
                nodeElem->SetAttribute("target_agent_id", "2");
            }
        }
        else if (nodeId == "ConversationFormation")
        {
            nodeElem->SetAttribute("main_agent_id", "{id}");
            nodeElem->SetAttribute("time_step", "{dt}");
            
            // Handle different blocks that use ConversationFormation
            if (blockId == "TalkInteract")
            {
                // For TalkInteract, use the selected conversation participants
                // Check interaction mode to determine how to set non_main_agent_ids
                QString modeKey = QString("%1.interaction_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "find_nearest").toString();
                
                if (mode == "find_nearest")
                {
                    // In find_nearest mode, conversation partner is the found agent
                    nodeElem->SetAttribute("non_main_agent_ids", "{target_agent_id}");
                }
                else // specific_target
                {
                    // In specific_target mode, use the selected participants
                    if (assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
                    {
                        QString participants = assignment.agentSpecificParams[blockId + ".non_main_agent_ids"].toString();
                        std::string participants_str = participants.toStdString();
                        nodeElem->SetAttribute("non_main_agent_ids", participants_str.c_str());
                    }
                    else
                    {
                        // Fallback to default
                        nodeElem->SetAttribute("non_main_agent_ids", "2");
                    }
                }
                
                // Apply goal_id from user selection
                if (assignment.agentSpecificParams.contains(blockId + ".goal_id"))
                {
                    std::string goal_id_value = assignment.agentSpecificParams[blockId + ".goal_id"].toString().toStdString();
                    nodeElem->SetAttribute("goal_id", goal_id_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("goal_id", "1");
                }
                
                // Apply conversation_duration from user selection
                if (assignment.agentSpecificParams.contains(blockId + ".conversation_duration"))
                {
                    std::string conversation_duration_value = assignment.agentSpecificParams[blockId + ".conversation_duration"].toString().toStdString();
                    nodeElem->SetAttribute("conversation_duration", conversation_duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("conversation_duration", "30.0");
                }
            }
            else if (blockId == "SpeechDetection")
            {
                // For SpeechDetection, the speaker is the conversation partner
                nodeElem->SetAttribute("non_main_agent_ids", "{speaker_id}");
                
                // Apply goal_id from user selection
                if (assignment.agentSpecificParams.contains(blockId + ".goal_id"))
                {
                    std::string goal_id_value = assignment.agentSpecificParams[blockId + ".goal_id"].toString().toStdString();
                    nodeElem->SetAttribute("goal_id", goal_id_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("goal_id", "1");
                }
                
                // Apply conversation_duration from user selection
                if (assignment.agentSpecificParams.contains(blockId + ".conversation_duration"))
                {
                    std::string conversation_duration_value = assignment.agentSpecificParams[blockId + ".conversation_duration"].toString().toStdString();
                    nodeElem->SetAttribute("conversation_duration", conversation_duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("conversation_duration", "45.0");
                }
            }
            else
            {
                // Generic handling for other blocks (e.g., ProtectiveGuardian)
                if (assignment.agentSpecificParams.contains(blockId + ".goal_id"))
                {
                    std::string goal_id_value = assignment.agentSpecificParams[blockId + ".goal_id"].toString().toStdString();
                    nodeElem->SetAttribute("goal_id", goal_id_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("goal_id", "1");
                }
                if (assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
                {
                    std::string non_main_agent_ids_value = assignment.agentSpecificParams[blockId + ".non_main_agent_ids"].toString().toStdString();
                    nodeElem->SetAttribute("non_main_agent_ids", non_main_agent_ids_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("non_main_agent_ids", "2,3");
                }
            }
        }
        else if (nodeId == "SetGroupWalk")
        {
            nodeElem->SetAttribute("main_agent_id", "{id}");
            if (blockId == "GroupFormation")
            {
                nodeElem->SetAttribute("non_main_agent_ids", "{target_agent_id}");
            }
            else if (!assignment.agentSpecificParams.contains(blockId + ".non_main_agent_ids"))
            {
                nodeElem->SetAttribute("non_main_agent_ids", "2");
            }
        }
        else if (nodeId == "ApproachAgent")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "SpeechDetection")
            {
                nodeElem->SetAttribute("target_agent_id", "{speaker_id}");
                
                // Apply user-configured approach parameters
                if (assignment.agentSpecificParams.contains(blockId + ".approach_distance"))
                {
                    std::string closest_dist_value = assignment.agentSpecificParams[blockId + ".approach_distance"].toString().toStdString();
                    nodeElem->SetAttribute("closest_dist", closest_dist_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("closest_dist", "1.5");
                }
                
                if (assignment.agentSpecificParams.contains(blockId + ".approach_velocity"))
                {
                    std::string max_vel_value = assignment.agentSpecificParams[blockId + ".approach_velocity"].toString().toStdString();
                    nodeElem->SetAttribute("max_vel", max_vel_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("max_vel", "1.2");
                }
                
                // Apply user-configured approach duration
                if (assignment.agentSpecificParams.contains(blockId + ".approach_duration"))
                {
                    std::string duration_value = assignment.agentSpecificParams[blockId + ".approach_duration"].toString().toStdString();
                    nodeElem->SetAttribute("duration", duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("duration", "15.0");
                }
            }
            else if (blockId == "ProtectiveGuardian")
            {
                // Check protection mode
                QString modeKey = QString("%1.protection_mode").arg(blockId);
                QString mode = assignment.agentSpecificParams.value(modeKey, "specific_protected").toString();
                
                if (mode == "protect_nearest_threatened")
                {
                    // Approach the agent found by FindNearestAgent
                    nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
                }
                else // specific_protected
                {
                    // Approach the protected agent to intervene
                    if (assignment.agentSpecificParams.contains(blockId + ".target_agent_id"))
                    {
                        std::string target_agent_id_value = assignment.agentSpecificParams[blockId + ".target_agent_id"].toString().toStdString();
                        nodeElem->SetAttribute("target_agent_id", target_agent_id_value.c_str());
                    }
                    else
                    {
                        nodeElem->SetAttribute("target_agent_id", "2");
                    }
                }
                
                // Apply user-configured protective approach distance (used in both modes)
                if (assignment.agentSpecificParams.contains(blockId + ".protective_approach_distance"))
                {
                    std::string closest_dist_value = assignment.agentSpecificParams[blockId + ".protective_approach_distance"].toString().toStdString();
                    nodeElem->SetAttribute("closest_dist", closest_dist_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("closest_dist", "1.8");
                }
                
                // Apply user-configured protective approach velocity (used in both modes)
                if (assignment.agentSpecificParams.contains(blockId + ".protective_approach_velocity"))
                {
                    std::string max_vel_value = assignment.agentSpecificParams[blockId + ".protective_approach_velocity"].toString().toStdString();
                    nodeElem->SetAttribute("max_vel", max_vel_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("max_vel", "1.8");
                }
                
                // No duration limit for protective intervention (approach until close enough)
            }
        }
        else if (nodeId == "IsAnyoneSpeaking")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "SpeechDetection")
            {
                // Explicitly set the output port speaker_id (like FindNearestAgent's target_agent_id)
                nodeElem->SetAttribute("speaker_id", "{speaker_id}");
                
                // Apply user-configured speech detection distance
                if (assignment.agentSpecificParams.contains(blockId + ".speech_distance"))
                {
                    std::string distance_threshold_value = assignment.agentSpecificParams[blockId + ".speech_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance_threshold", distance_threshold_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance_threshold", "8.0");
                }
                
                // Apply user-configured speech detection duration
                if (assignment.agentSpecificParams.contains(blockId + ".speaking_duration"))
                {
                    std::string duration_value = assignment.agentSpecificParams[blockId + ".speaking_duration"].toString().toStdString();
                    nodeElem->SetAttribute("duration", duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("duration", "5.0");
                }
            }
        }
        else if (nodeId == "IsAnyoneLookingAtMe")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "AttentionSeeking")
            {
                // Explicitly set the output port (like FindNearestAgent's target_agent_id or IsAnyoneSpeaking's speaker_id)
                nodeElem->SetAttribute("observer_id", "{observer_id}");
                
                // Apply user-configured attention detection distance
                if (assignment.agentSpecificParams.contains(blockId + ".attention_distance"))
                {
                    std::string distance_threshold_value = assignment.agentSpecificParams[blockId + ".attention_distance"].toString().toStdString();
                    nodeElem->SetAttribute("distance_threshold", distance_threshold_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("distance_threshold", "6.0");
                }
                
                // Apply user-configured attention detection duration
                if (assignment.agentSpecificParams.contains(blockId + ".looking_duration"))
                {
                    std::string duration_value = assignment.agentSpecificParams[blockId + ".looking_duration"].toString().toStdString();
                    nodeElem->SetAttribute("duration", duration_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("duration", "5.0");
                }
            }
        }
        else if (nodeId == "SaySomething")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            
            if (blockId == "AttentionSeeking")
            {
                // Apply user-configured response message
                if (assignment.agentSpecificParams.contains(blockId + ".response_message"))
                {
                    std::string message_value = assignment.agentSpecificParams[blockId + ".response_message"].toString().toStdString();
                    nodeElem->SetAttribute("message", message_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("message", "I see you looking at me!");
                }
            }
            else if (blockId == "GreetingInitiator")
            {
                // Apply user-configured greeting message
                if (assignment.agentSpecificParams.contains(blockId + ".greeting_message"))
                {
                    std::string message_value = assignment.agentSpecificParams[blockId + ".greeting_message"].toString().toStdString();
                    nodeElem->SetAttribute("message", message_value.c_str());
                }
                else
                {
                    nodeElem->SetAttribute("message", "Hello there!");
                }
            }
        }
        else if (nodeId == "FindNearestAgent")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            // Explicitly set the output port (like IsAnyoneSpeaking's speaker_id)
            nodeElem->SetAttribute("target_agent_id", "{target_agent_id}");
        }
        else if (nodeId == "SetGroupId")
        {
            nodeElem->SetAttribute("agent_id", "{id}");
            nodeElem->SetAttribute("group_id", "{target_agent_id}");
        }
        else
        {
            nodeElem->SetAttribute("agent_id", "{id}");

            if ((nodeId == "GoTo" || nodeId == "LookAtPoint") &&
                !assignment.agentSpecificParams.contains(blockId + ".goal_id"))
            {
                nodeElem->SetAttribute("goal_id", "1");
            }
        }

        if (nodeId == "FollowAgent" || nodeId == "ApproachAgent" || nodeId == "ApproachRobot" ||
            nodeId == "BlockAgent" || nodeId == "BlockRobot" || nodeId == "SetGroupWalk" ||
            nodeId == "ConversationFormation" || nodeId == "StopAndWaitTimerAction" ||
            nodeId == "TimeDelayDecorator" || nodeId == "TimeExpiredCondition" ||
            nodeId == "IsAnyoneSpeaking" || nodeId == "IsSpeaking" ||
            nodeId == "IsAnyoneLookingAtMe" || nodeId == "IsLookingAtMe" || nodeId == "GoTo")
        {
            nodeElem->SetAttribute("time_step", "{dt}");
        }
    }
    catch (const std::exception &e)
    {
        qWarning() << "Exception setting default attributes for node" << nodeId << ":" << e.what();
    }
    catch (...)
    {
        qWarning() << "Unknown exception setting default attributes for node" << nodeId;
    }
}

// Helper method to map parameter names to XML attributes
QString BTConfigDialog::mapParameterToXMLAttribute(const QString & /*blockId*/,
                                                   const QString &nodeId,
                                                   const QString &paramName)
{

    if (nodeId == "IsRobotVisible")
    {
        if (paramName == "detection_distance")
            return "distance";
        if (paramName == "robot_detection_distance")
            return "distance";
        return "";
    }

    if (nodeId == "IsRobotClose")
    {
        if (paramName == "close_threshold")
            return "threshold";
        if (paramName == "proximity_threshold")
            return "threshold";
        if (paramName == "robot_close_threshold")
            return "threshold";
        return "";
    }

    if (nodeId == "IsAgentVisible")
    {
        if (paramName == "target_agent_id")
            return "agent_id";
        if (paramName == "visibility_distance")
            return "distance";
        if (paramName == "agent_visibility_distance")
            return "distance";
        if (paramName == "detection_distance")
            return "distance";
        return "";
    }

    if (nodeId == "IsAgentClose")
    {
        if (paramName == "target_agent_id")
            return "target_agent_id";
        if (paramName == "is_close_threshold")
            return "threshold";
        if (paramName == "social_distance_threshold")
            return "threshold";
        return "";
    }

    if (nodeId == "FollowAgent")
    {
        if (paramName == "target_agent_id")
            return "target_agent_id";
        if (paramName == "closest_dist" || paramName == "following_distance")
            return "closest_dist";
        if (paramName == "max_vel")
            return "max_vel";
        if (paramName == "duration" || paramName == "follow_duration")
            return "duration";
        return "";
    }

    if (nodeId == "ConversationFormation")
    {
        if (paramName == "goal_id")
            return "goal_id";
        if (paramName == "non_main_agent_ids")
            return "non_main_agent_ids";
        if (paramName == "conversation_duration")
            return "conversation_duration";
        return "";
    }

    if (nodeId == "ApproachRobot")
    {
        if (paramName == "approach_distance" || paramName == "closest_dist")
            return "closest_dist";
        if (paramName == "max_velocity" || paramName == "max_vel")
            return "max_vel";
        if (paramName == "duration" || paramName == "approach_duration")
            return "duration";
        return "";
    }

    if (nodeId == "GoTo")
    {
        if (paramName == "goal_id")
            return "goal_id";
        if (paramName == "tolerance")
            return "tolerance";
        return "";
    }

    if (nodeId == "LookAtRobot")
    {
        if (paramName == "yaw_tolerance")
            return "yaw_tolerance";
        return "";
    }

    if (nodeId == "BlockRobot")
    {
        if (paramName == "front_dist")
            return "front_dist";
        if (paramName == "protective_front_distance")
            return "front_dist";
        if (paramName == "duration")
            return "duration";
        if (paramName == "blocking_duration")
            return "duration";
        return "";
    }

    if (nodeId == "FindNearestAgent")
    {
        // FindNearestAgent doesn't need configurable parameters - it uses agent_id from context
        // and outputs target_agent_id automatically
        return "";
    }

    if (nodeId == "LookAtAgent")
    {
        if (paramName == "yaw_tolerance")
            return "yaw_tolerance";
        return "";
    }

    if (nodeId == "SetGroupWalk")
    {
        if (paramName == "non_main_agent_ids")
            return "non_main_agent_ids";
        if (paramName == "duration")
            return "duration";
        return "";
    }
    
    if (nodeId == "IsAnyoneLookingAtMe")
    {
        if (paramName == "attention_distance")
            return "distance_threshold";
        if (paramName == "looking_duration")
            return "duration";
        return "";
    }
    
    if (nodeId == "IsRobotFacingAgent")
    {
        if (paramName == "attention_distance")
            return "distance";
        return "";
    }
    
    if (nodeId == "IsAnyoneSpeaking")
    {
        if (paramName == "speech_distance")
            return "distance_threshold";
        if (paramName == "speaking_duration")
            return "duration";
        return "";
    }
    
    if (nodeId == "SaySomething")
    {
        if (paramName == "response_message")
            return "message";
        if (paramName == "greeting_message")
            return "message";
        return "";
    }
    
    if (nodeId == "ApproachAgent")
    {
        if (paramName == "protective_approach_distance")
            return "closest_dist";
        if (paramName == "protective_approach_velocity")
            return "max_vel";
        if (paramName == "protective_approach_duration")
            return "duration";
        if (paramName == "interaction_duration")
            return "duration";
        return "";
    }

    if (paramName == "goal_id")
        return "goal_id";
    if (paramName == "duration")
        return "duration";
    if (paramName == "target_agent_id")
        return "target_agent_id";

    return ""; // Return empty for unrecognized parameters
}

QStringList BTConfigDialog::getEnhancedNodesForBlock(const QString &blockId)
{
    static const QMap<QString, QStringList> enhancedBlockNodes = {
        {"EngageRobot", {"IsRobotVisible", "LookAtRobot", "ApproachRobot"}},
        {"TalkInteract", {"IsAgentVisible", "IsAgentClose", "ConversationFormation"}},
        {"RobotAvoidance", {"IsRobotVisible", "IsRobotClose", "StopMovement", "LookAtRobot", "RetryUntilSuccessful:Inverter:IsRobotClose", "ResumeMovement"}},
        {"FollowAgent", {"IsAgentVisible", "IsAgentClose", "FollowAgent"}},
        {"BlockingBehavior", {"IsRobotVisible", "LookAtRobot", "BlockRobot"}},
        {"GroupFormation", {"FindNearestAgent", "SetGroupWalk"}},
        {"SpeechDetection", {"IsAnyoneSpeaking", "ApproachAgent", "ConversationFormation"}},
        {"AttentionSeeking", {"IsAnyoneLookingAtMe", "LookAtAgent", "SaySomething"}},
        {"GreetingInitiator", {"IsAgentVisible", "SaySomething"}},
        {"ProtectiveGuardian", {"IsRobotVisible", "IsAgentVisible", "IsRobotClose", "LookAtAgent", "ApproachAgent", "BlockRobot"}}};

    return enhancedBlockNodes.value(blockId, QStringList());
}

// Method to setup direct parameter update connections for real-time sync
void BTConfigDialog::setupParameterUpdateConnections(const QString &blockId, int agentIndex)
{
    // Ensure agents_ is properly sized
    while (agents_.size() <= agentIndex)
    {
        AgentAssignment newAgent;
        newAgent.agentIndex = agents_.size();
        newAgent.agentName = QString("Agent_%1").arg(agents_.size() + 1);
        newAgent.behaviorType = "Regular";
        newAgent.randomizeOrder = false;
        agents_.append(newAgent);
    }

    // Track connections to prevent duplicates

    // Helper to create parameter update connections
    auto createParameterUpdater = [this, agentIndex](const QString &paramKey, QWidget *widget)
    {
        if (!widget)
            return;
        
        // Bounds check for agentIndex
        if (agentIndex < 0 || agentIndex >= agents_.size())
        {
            qWarning() << "createParameterUpdater: agentIndex" << agentIndex << "out of range (size:" << agents_.size() << ")";
            return;
        }

        // Create a unique key for this widget connection
        QString connectionKey = QString("%1_%2").arg(reinterpret_cast<quintptr>(widget)).arg(paramKey);

        // Skip if already connected
        if (connectedWidgets_.contains(connectionKey))
        {
            return;
        }

        qDebug() << "Setting up parameter updater for" << paramKey << "with widget" << widget;

        // Restore stored parameter value to the widget if available
        if (agentIndex < agents_.size() && agents_[agentIndex].agentSpecificParams.contains(paramKey))
        {
            QVariant storedValue = agents_[agentIndex].agentSpecificParams[paramKey];

            if (auto spinBox = qobject_cast<QSpinBox *>(widget))
            {
                spinBox->setValue(storedValue.toInt());
                qDebug() << "RESTORED: " << paramKey << "=" << storedValue.toInt() << "to QSpinBox";
            }
            else if (auto doubleSpinBox = qobject_cast<QDoubleSpinBox *>(widget))
            {
                doubleSpinBox->setValue(storedValue.toDouble());
                qDebug() << "RESTORED: " << paramKey << "=" << storedValue.toDouble() << "to QDoubleSpinBox";
            }
            else if (auto lineEdit = qobject_cast<QLineEdit *>(widget))
            {
                lineEdit->setText(storedValue.toString());
                qDebug() << "RESTORED: " << paramKey << "=" << storedValue.toString() << "to QLineEdit";
            }
            else if (auto comboBox = qobject_cast<QComboBox *>(widget))
            {
                int index = comboBox->findData(storedValue);
                if (index == -1)
                {
                    index = comboBox->findText(storedValue.toString());
                }
                if (index != -1)
                {
                    comboBox->setCurrentIndex(index);
                    qDebug() << "RESTORED: " << paramKey << "=" << storedValue << "to QComboBox at index" << index;
                }
            }
        }

        // Connect different widget types with immediate parameter storage
        if (auto spinBox = qobject_cast<QSpinBox *>(widget))
        {
            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), spinBox, [this, agentIndex, paramKey](int value)
                    {
                if (agentIndex >= agents_.size()) return;
                agents_[agentIndex].agentSpecificParams[paramKey] = value;
                qDebug() << "PARAM UPDATE: " << paramKey << "=" << value << "for agent" << agentIndex; });
            // Set initial value if not already restored
            if (!agents_[agentIndex].agentSpecificParams.contains(paramKey))
            {
                int initialValue = spinBox->value();
                agents_[agentIndex].agentSpecificParams[paramKey] = initialValue;
                qDebug() << "PARAM INIT: " << paramKey << "=" << initialValue << "for agent" << agentIndex;
            }
        }
        else if (auto doubleSpinBox = qobject_cast<QDoubleSpinBox *>(widget))
        {
            connect(doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), doubleSpinBox, [this, agentIndex, paramKey](double value)
                    {
                if (agentIndex >= agents_.size()) return;
                agents_[agentIndex].agentSpecificParams[paramKey] = value;
                qDebug() << "PARAM UPDATE: " << paramKey << "=" << value << "for agent" << agentIndex; });
            // Set initial value if not already restored
            if (!agents_[agentIndex].agentSpecificParams.contains(paramKey))
            {
                double initialValue = doubleSpinBox->value();
                agents_[agentIndex].agentSpecificParams[paramKey] = initialValue;
                qDebug() << "PARAM INIT: " << paramKey << "=" << initialValue << "for agent" << agentIndex;
            }
        }
        else if (auto lineEdit = qobject_cast<QLineEdit *>(widget))
        {
            connect(lineEdit, &QLineEdit::textChanged, lineEdit, [this, agentIndex, paramKey](const QString &value)
                    {
                if (agentIndex >= agents_.size()) return;
                agents_[agentIndex].agentSpecificParams[paramKey] = value;
                qDebug() << "PARAM UPDATE: " << paramKey << "=" << value << "for agent" << agentIndex; });

            if (!agents_[agentIndex].agentSpecificParams.contains(paramKey))
            {
                QString initialValue = lineEdit->text();
                agents_[agentIndex].agentSpecificParams[paramKey] = initialValue;
                qDebug() << "PARAM INIT: " << paramKey << "=" << initialValue << "for agent" << agentIndex;
            }
        }
        else if (auto comboBox = qobject_cast<QComboBox *>(widget))
        {
            // Use QPointer to safely capture the comboBox
            QPointer<QComboBox> comboBoxPtr(comboBox);
            connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), comboBox, [this, agentIndex, paramKey, comboBoxPtr](int)
                    {
                if (agentIndex >= agents_.size() || !comboBoxPtr) return;
                QVariant value = comboBoxPtr->currentData();
                if (!value.isValid()) value = comboBoxPtr->currentText();
                agents_[agentIndex].agentSpecificParams[paramKey] = value;
                qDebug() << "PARAM UPDATE: " << paramKey << "=" << value << "for agent" << agentIndex; });

            if (!agents_[agentIndex].agentSpecificParams.contains(paramKey))
            {
                QVariant initialValue = comboBox->currentData();
                if (!initialValue.isValid())
                    initialValue = comboBox->currentText();
                agents_[agentIndex].agentSpecificParams[paramKey] = initialValue;
                qDebug() << "PARAM INIT: " << paramKey << "=" << initialValue << "for agent" << agentIndex;
            }
        }

        // Mark this widget as connected
        connectedWidgets_.insert(connectionKey);
    };

    // Find all widgets for this block and agent and create connections
    QString blockPrefix = QString("%1_%2_").arg(blockId).arg(agentIndex);
    int connectionsCreated = 0;

    for (auto it = blockConfigWidgets_.begin(); it != blockConfigWidgets_.end(); ++it)
    {
        QString widgetKey = it.key();
        QWidget *widget = it.value();

        // Safety checks: widget exists, not null, has valid parent, and matches our block
        if (!widget || !widgetKey.startsWith(blockPrefix))
        {
            continue;
        }

        // Skip widgets without a parent
        if (!widget->parent())
        {
            qDebug() << "Skipping orphaned widget for key:" << widgetKey;
            continue;
        }

        // Extract parameter name from widget key
        QString paramName = widgetKey.mid(blockPrefix.length());
        QString paramKey = QString("%1.%2").arg(blockId).arg(paramName);

        createParameterUpdater(paramKey, widget);
        connectionsCreated++;

        qDebug() << "Created parameter connection:" << widgetKey << "->" << paramKey;
    }

    qDebug() << "setupParameterUpdateConnections: Created" << connectionsCreated << "connections for block" << blockId << "agent" << agentIndex;

    // Verify parameter storage immediately (with bounds check)
    if (agentIndex >= 0 && agentIndex < agents_.size())
    {
        qDebug() << "Current parameters for agent" << agentIndex << ":" << agents_[agentIndex].agentSpecificParams;
    }
    else
    {
        qWarning() << "Cannot verify parameters: agentIndex" << agentIndex << "out of range (size:" << agents_.size() << ")";
    }
}

void BTConfigDialog::setupAllParameterConnections()
{
    qDebug() << "setupAllParameterConnections: Setting up parameter connections for all assigned blocks";

    // Iterate through all agents and their assigned blocks
    for (int agentIndex = 0; agentIndex < agents_.size(); ++agentIndex)
    {
        const auto &agent = agents_[agentIndex];
        qDebug() << "Processing parameter connections for agent" << agentIndex << "with" << agent.assignedBlocks.size() << "assigned blocks";

        for (const QString &blockId : agent.assignedBlocks)
        {
            QString setupKey = QString("%1_%2").arg(blockId).arg(agentIndex);

            // Skip if we've already set up connections for this agent-block combination
            if (setupTracker_.contains(setupKey))
            {
                qDebug() << "Skipping already configured agent" << agentIndex << "block" << blockId;
                continue;
            }

            qDebug() << "Setting up parameters for agent" << agentIndex << "block" << blockId;

            // Check if widgets exist for this block/agent combination
            bool widgetsExist = false;
            QString blockPrefix = QString("%1_%2_").arg(blockId).arg(agentIndex);
            for (auto it = blockConfigWidgets_.begin(); it != blockConfigWidgets_.end(); ++it)
            {
                if (it.key().startsWith(blockPrefix) && it.value() != nullptr)
                {
                    widgetsExist = true;
                    break;
                }
            }

            if (!widgetsExist)
            {
                qDebug() << "Widgets don't exist yet for" << blockId << "agent" << agentIndex << "- will be created when agent is selected in config panel";
                continue;
            }

            // Set up parameter update connections for this block/agent
            setupParameterUpdateConnections(blockId, agentIndex);

            // Mark this combination as set up
            setupTracker_.insert(setupKey);
        }
    }

    qDebug() << "setupAllParameterConnections: Completed parameter connection setup";
}

void BTConfigDialog::clearParameterConnectionTracking()
{
    qDebug() << "clearParameterConnectionTracking: Clearing connection tracking data";
    qDebug() << "Clearing setupTracker_...";
    setupTracker_.clear();
    qDebug() << "setupTracker_ cleared";
    
    qDebug() << "Clearing connectedWidgets_...";
    connectedWidgets_.clear();
    qDebug() << "connectedWidgets_ cleared";

    qDebug() << "Clearing blockConfigWidgets_ (had" << blockConfigWidgets_.size() << "entries)...";
    blockConfigWidgets_.clear();
    qDebug() << "blockConfigWidgets_ cleared";

    qDebug() << "Connection tracking cleared - ready for fresh parameter setup";
}

QList<BTConfigDialog::BlockConfig> BTConfigDialog::getSelectedBlocks() const
{
    QList<BlockConfig> selectedBlocks;
    for (const auto &block : blocks_)
    {
        if (block.isSelected)
        {
            selectedBlocks.append(block);
        }
    }
    return selectedBlocks;
}

// ===================== XML CONFIGURATION LOADING =====================

bool BTConfigDialog::loadExistingConfiguration(const QStringList &btPaths)
{
    qDebug() << "=========== LOADING EXISTING BT CONFIGURATION ===========";
    qDebug() << "Loading from" << btPaths.size() << "files";
    qDebug() << "Current agents_.size():" << agents_.size();
    qDebug() << "Current blocks_.size():" << blocks_.size();

    if (btPaths.isEmpty())
    {
        qDebug() << "No BT paths provided for loading";
        return false;
    }

    // Reset current configuration
    for (auto &agent : agents_)
    {
        agent.assignedBlocks.clear();
        agent.runOnceBlocks.clear();
        agent.randomExecutionBlocks.clear();
        agent.agentSpecificParams.clear();
    }

    qDebug() << "Reset all agent assignments";

    bool anyLoaded = false;

    // Parse each agent's XML file
    for (int i = 0; i < btPaths.size() && i < agents_.size(); ++i)
    {
        const QString &xmlPath = btPaths[i];

        qDebug() << "Checking file" << i << ":" << xmlPath;

        if (!QFile::exists(xmlPath))
        {
            qDebug() << "  -> File does NOT exist";
            continue;
        }

        qDebug() << "  -> File exists, parsing...";

        if (parseAgentXML(xmlPath, i))
        {
            anyLoaded = true;
            qDebug() << "  -> Successfully parsed! Agent" << i << "now has" << agents_[i].assignedBlocks.size() << "blocks";
            qDebug() << "  -> Assigned blocks:" << agents_[i].assignedBlocks;
        }
        else
        {
            qDebug() << "  -> Failed to parse";
        }
    }

    if (anyLoaded)
    {
        qDebug() << "========== CONFIGURATION LOADED ==========";

        // Mark blocks as selected if they're assigned to any agent
        QSet<QString> usedBlocks;
        for (const auto &agent : agents_)
        {
            for (const QString &blockId : agent.assignedBlocks)
            {
                usedBlocks.insert(blockId);
            }
        }

        qDebug() << "Total unique blocks used across all agents:" << usedBlocks.size();
        qDebug() << "Used blocks:" << usedBlocks;

        // Update block selection status
        int markedCount = 0;
        for (auto &block : blocks_)
        {
            if (usedBlocks.contains(block.blockId))
            {
                block.isSelected = true;
                markedCount++;
                qDebug() << "Marked block as selected:" << block.blockId;
            }
        }

        qDebug() << "Marked" << markedCount << "blocks as selected";

        // Update UI to reflect loaded configuration
        qDebug() << "Calling updateAssignmentMatrix()...";
        updateAssignmentMatrix();
        qDebug() << "Configuration loaded successfully - UI updated";
        qDebug() << "========================================";
        return true;
    }

    qDebug() << "No configurations could be loaded";
    return false;
}

bool BTConfigDialog::parseAgentXML(const QString &xmlPath, int agentIndex)
{
    tinyxml2::XMLDocument doc;
    std::string xmlPath_str = xmlPath.toStdString();
    if (doc.LoadFile(xmlPath_str.c_str()) != tinyxml2::XML_SUCCESS)
    {
        qWarning() << "Cannot parse XML file:" << xmlPath << "-" << doc.ErrorStr();
        return false;
    }

    // Find the BehaviorTree element
    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (!root)
    {
        qWarning() << "No root element in XML file:" << xmlPath;
        return false;
    }

    // Navigate to BehaviorTree
    tinyxml2::XMLElement *bt = nullptr;
    if (strcmp(root->Name(), "BehaviorTree") == 0)
    {
        bt = root;
    }
    else
    {
        // Search for BehaviorTree recursively
        std::function<tinyxml2::XMLElement *(tinyxml2::XMLElement *)> findBT =
            [&](tinyxml2::XMLElement *parent) -> tinyxml2::XMLElement *
        {
            for (auto *elem = parent->FirstChildElement(); elem; elem = elem->NextSiblingElement())
            {
                if (strcmp(elem->Name(), "BehaviorTree") == 0)
                {
                    return elem;
                }
                if (auto *found = findBT(elem))
                {
                    return found;
                }
            }
            return nullptr;
        };
        bt = findBT(root);
    }

    if (!bt)
    {
        qWarning() << "No BehaviorTree element found in" << xmlPath;
        return false;
    }

    qDebug() << "Parsing BehaviorTree for agent" << agentIndex;

    // Find the main container (usually ReactiveSequence or Sequence)
    tinyxml2::XMLElement *mainContainer = bt->FirstChildElement();
    if (!mainContainer)
    {
        qWarning() << "BehaviorTree has no child elements";
        return false;
    }

    // Iterate through sequences looking for our configured blocks
    for (auto *sequence = mainContainer->FirstChildElement("Sequence");
         sequence;
         sequence = sequence->NextSiblingElement("Sequence"))
    {

        const char *name = sequence->Attribute("name");
        if (!name)
            continue;

        QString seqName = QString(name);
        qDebug() << "Found sequence:" << seqName;

        // Check if this is a SetGoals sequence (skip it)
        if (seqName == "SetGoals" || seqName == "RegNav")
        {
            continue;
        }

        // Check if it's a random execution wrapper
        bool isRandom = false;
        double randomProbability = 0.5;
        tinyxml2::XMLElement *actualSequence = sequence;
        
        if (seqName.endsWith("RandomBlock"))
        {
            // This is a random execution wrapper - look for RandomChanceCondition
            if (auto *randomCondition = sequence->FirstChildElement("Condition"))
            {
                const char *condId = randomCondition->Attribute("ID");
                if (condId && QString(condId) == "RandomChanceCondition")
                {
                    isRandom = true;
                    const char *prob = randomCondition->Attribute("probability");
                    if (prob)
                    {
                        randomProbability = QString(prob).toDouble();
                    }
                    
                    // Get the actual block sequence inside
                    if (auto *innerSeq = sequence->FirstChildElement("Sequence"))
                    {
                        actualSequence = innerSeq;
                        const char *innerName = innerSeq->Attribute("name");
                        if (innerName)
                        {
                            seqName = QString(innerName);
                        }
                    }
                }
            }
        }

        // Check if it's a RunOnce wrapper (could be inside or outside random wrapper)
        bool isRunOnce = false;

        // Check if there's a RunOnce decorator wrapping this sequence
        if (auto *runOnce = actualSequence->FirstChildElement("RunOnce"))
        {
            if (auto *innerSeq = runOnce->FirstChildElement("Sequence"))
            {
                actualSequence = innerSeq;
                isRunOnce = true;
                const char *innerName = innerSeq->Attribute("name");
                if (innerName)
                {
                    seqName = QString(innerName);
                }
            }
        }

        // Identify which block this sequence represents
        QString blockId = identifyBlockFromSequence(actualSequence);

        if (!blockId.isEmpty())
        {
            qDebug() << "Identified block:" << blockId << "for agent" << agentIndex;
            
            // Bounds check for agentIndex before accessing agents_ array
            if (agentIndex < 0 || agentIndex >= agents_.size())
            {
                qWarning() << "agentIndex" << agentIndex << "out of range (size:" << agents_.size() << ") - skipping block" << blockId;
                continue;
            }

            // Add to agent's assigned blocks
            if (!agents_[agentIndex].assignedBlocks.contains(blockId))
            {
                agents_[agentIndex].assignedBlocks.append(blockId);
            }

            // Track if it's run-once
            if (isRunOnce && !agents_[agentIndex].runOnceBlocks.contains(blockId))
            {
                agents_[agentIndex].runOnceBlocks.append(blockId);
            }
            
            // Track if it's random execution
            if (isRandom)
            {
                agents_[agentIndex].randomExecutionBlocks[blockId] = randomProbability;
            }

            // Extract parameters from nodes in this sequence
            QStringList participantIds; // Collect agent IDs from multiple condition nodes
            for (auto *node = actualSequence->FirstChildElement();
                 node;
                 node = node->NextSiblingElement())
            {
                extractParametersFromNode(node, blockId, agentIndex);
                
                // For TalkInteract, collect agent IDs from multiple IsAgentVisible/IsAgentClose nodes
                if (blockId == "TalkInteract")
                {
                    QString nodeName = QString(node->Name());
                    if (nodeName == "Condition")
                    {
                        const char *nodeId = node->Attribute("ID");
                        if (nodeId && QString(nodeId) == "IsAgentVisible")
                        {
                            const char *agentId = node->Attribute("agent_id");
                            if (agentId)
                            {
                                QString idStr = QString(agentId);
                                // Skip blackboard references
                                if (!idStr.startsWith("{") && !participantIds.contains(idStr))
                                {
                                    participantIds.append(idStr);
                                }
                            }
                        }
                    }
                }
            }
            
            // Store collected participant IDs for multi-participant conversations
            if (blockId == "TalkInteract" && participantIds.size() > 0)
            {
                QString key = QString("%1.non_main_agent_ids").arg(blockId);
                // Only override if we collected more than what ConversationFormation had
                // or if ConversationFormation had a blackboard reference
                if (participantIds.size() > 1 || !agents_[agentIndex].agentSpecificParams.contains(key))
                {
                    agents_[agentIndex].agentSpecificParams[key] = participantIds.join(",");
                    qDebug() << "Collected" << participantIds.size() << "participant IDs for TalkInteract:" << participantIds.join(",");
                }
            }
            
            // Detect and store mode for dual-mode blocks
            inferBlockMode(actualSequence, blockId, agentIndex);
        }
    }

    qDebug() << "Agent" << agentIndex << "assigned blocks:" << agents_[agentIndex].assignedBlocks;
    return !agents_[agentIndex].assignedBlocks.isEmpty();
}

void BTConfigDialog::inferBlockMode(tinyxml2::XMLElement *sequence, const QString &blockId, int agentIndex)
{
    if (!sequence)
        return;
    
    // Check for FindNearestAgent node to determine mode
    bool hasFindNearest = false;
    for (auto *node = sequence->FirstChildElement(); node; node = node->NextSiblingElement())
    {
        QString nodeName = QString(node->Name());
        if (nodeName == "FindNearestAgent" || 
            (nodeName == "Action" && node->Attribute("ID") && QString(node->Attribute("ID")) == "FindNearestAgent"))
        {
            hasFindNearest = true;
            break;
        }
    }
    
    // Infer and store mode for dual-mode blocks
    if (blockId == "TalkInteract" || blockId == "FollowAgent")
    {
        QString modeKey = QString("%1.interaction_mode").arg(blockId);
        QString mode = hasFindNearest ? "find_nearest" : "specific_target";
        agents_[agentIndex].agentSpecificParams[modeKey] = mode;
        qDebug() << "Inferred" << blockId << "mode:" << mode << "for agent" << agentIndex;
    }
    else if (blockId == "AttentionSeeking")
    {
        // Check for IsRobotFacingAgent to determine robot vs agent attention
        bool isRobotAttention = false;
        for (auto *node = sequence->FirstChildElement(); node; node = node->NextSiblingElement())
        {
            QString nodeName = QString(node->Name());
            if (nodeName == "IsRobotFacingAgent" ||
                (nodeName == "Condition" && node->Attribute("ID") && QString(node->Attribute("ID")) == "IsRobotFacingAgent"))
            {
                isRobotAttention = true;
                break;
            }
        }
        QString modeKey = QString("%1.attention_mode").arg(blockId);
        QString mode = isRobotAttention ? "robot_attention" : "agent_attention";
        agents_[agentIndex].agentSpecificParams[modeKey] = mode;
        qDebug() << "Inferred" << blockId << "mode:" << mode << "for agent" << agentIndex;
    }
    else if (blockId == "GreetingInitiator")
    {
        // Check for IsRobotVisible to detect greet_robot mode
        bool isRobotGreeting = false;
        for (auto *node = sequence->FirstChildElement(); node; node = node->NextSiblingElement())
        {
            QString nodeName = QString(node->Name());
            if (nodeName == "IsRobotVisible" ||
                (nodeName == "Condition" && node->Attribute("ID") && QString(node->Attribute("ID")) == "IsRobotVisible"))
            {
                isRobotGreeting = true;
                break;
            }
        }
        
        QString modeKey = QString("%1.greeting_mode").arg(blockId);
        QString mode;
        if (isRobotGreeting)
        {
            mode = "greet_robot";
        }
        else if (hasFindNearest)
        {
            mode = "nearest_agent";
        }
        else
        {
            mode = "specific_agent";
        }
        
        // Apply backward compatibility mapping for old mode names
        if (mode == "find_nearest")
            mode = "nearest_agent";
        else if (mode == "specific_target")
            mode = "specific_agent";
            
        agents_[agentIndex].agentSpecificParams[modeKey] = mode;
        qDebug() << "Inferred" << blockId << "mode:" << mode << "for agent" << agentIndex;
    }
    else if (blockId == "ProtectiveGuardian")
    {
        QString modeKey = QString("%1.protection_mode").arg(blockId);
        QString mode = hasFindNearest ? "protect_nearest_threatened" : "specific_protected";
        agents_[agentIndex].agentSpecificParams[modeKey] = mode;
        qDebug() << "Inferred" << blockId << "mode:" << mode << "for agent" << agentIndex;
    }
}

QString BTConfigDialog::identifyBlockFromSequence(tinyxml2::XMLElement *sequence)
{
    if (!sequence)
        return QString();

    // Get the sequence name
    const char *name = sequence->Attribute("name");
    if (name)
    {
        QString seqName = QString(name);

        // Check if it matches any of our block IDs
        for (const auto &block : blocks_)
        {
            if (seqName.contains(block.blockId, Qt::CaseInsensitive) ||
                seqName.endsWith("Block"))
            {
                // Extract potential block ID from name
                QString potentialId = seqName;
                potentialId.replace("Block", "");
                potentialId.replace("_", "");

                // Try to match with known blocks
                for (const auto &knownBlock : blocks_)
                {
                    if (knownBlock.blockId.contains(potentialId, Qt::CaseInsensitive))
                    {
                        return knownBlock.blockId;
                    }
                }
            }
        }
    }

    // Identify by nodes present in the sequence
    QStringList nodesPresent;
    for (auto *node = sequence->FirstChildElement(); node; node = node->NextSiblingElement())
    {
        QString nodeName = QString(node->Name());
        nodesPresent.append(nodeName);
    }

    // Match against known block patterns
    if (nodesPresent.contains("IsRobotVisible") && nodesPresent.contains("ApproachRobot"))
    {
        return "EngageRobot";
    }
    if (nodesPresent.contains("IsAgentVisible") && nodesPresent.contains("FollowAgent"))
    {
        return "FollowAgent";
    }
    if (nodesPresent.contains("IsAnyoneSpeaking") && nodesPresent.contains("ConversationFormation"))
    {
        return "SpeechDetection";
    }
    if (nodesPresent.contains("GoTo"))
    {
        return "GoalOriented";
    }
    if (nodesPresent.contains("LookAtAgent") && nodesPresent.contains("ApproachAgent"))
    {
        return "SocialInteraction";
    }
    if (nodesPresent.contains("BlockRobot") && nodesPresent.contains("IsRobotClose"))
    {
        return "ProtectiveGuardian";
    }
    if (nodesPresent.contains("GroupWalk"))
    {
        return "GroupFormation";
    }
    if (nodesPresent.contains("AgentSay"))
    {
        return "TalkInteract";
    }
    if (nodesPresent.contains("BlockAgent"))
    {
        return "BlockAgent";
    }
    if (nodesPresent.contains("LookAtPoint"))
    {
        return "LookAround";
    }

    qDebug() << "Could not identify block from nodes:" << nodesPresent;
    return QString();
}

void BTConfigDialog::extractParametersFromNode(tinyxml2::XMLElement *node,
                                               const QString &blockId,
                                               int agentIndex)
{
    if (!node)
        return;
    
    // Bounds check for agentIndex
    if (agentIndex < 0 || agentIndex >= agents_.size())
    {
        qWarning() << "extractParametersFromNode: agentIndex" << agentIndex << "out of range (size:" << agents_.size() << ")";
        return;
    }

    QString nodeName = QString(node->Name());
    QString nodeId;
    
    // Get the node ID (for Condition and Action nodes)
    if (nodeName == "Condition" || nodeName == "Action")
    {
        const char *id = node->Attribute("ID");
        if (id)
        {
            nodeId = QString(id);
        }
    }
    else
    {
        nodeId = nodeName;
    }

    // Map XML attributes to our internal parameter keys
    if (nodeId == "IsRobotVisible")
    {
        if (const char *dist = node->Attribute("distance"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "EngageRobot" || blockId == "RobotAvoidance" || blockId == "BlockingBehavior")
            {
                paramName = "detection_distance";
            }
            else
            {
                paramName = "robot_detection_distance"; // Default for other blocks
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
            qDebug() << "Extracted IsRobotVisible distance:" << dist << "for block" << blockId << "agent" << agentIndex;
        }
    }
    else if (nodeId == "ApproachRobot")
    {
        if (const char *dist = node->Attribute("closest_dist"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "EngageRobot")
            {
                paramName = "closest_dist";
            }
            else
            {
                paramName = "approach_distance"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
            qDebug() << "Extracted ApproachRobot closest_dist:" << dist << "for block" << blockId << "agent" << agentIndex;
        }
        if (const char *vel = node->Attribute("max_vel"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "EngageRobot")
            {
                paramName = "max_vel";
            }
            else
            {
                paramName = "approach_velocity"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(vel).toDouble();
            qDebug() << "Extracted ApproachRobot max_vel:" << vel << "for block" << blockId << "agent" << agentIndex;
        }
        if (const char *dur = node->Attribute("duration"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "EngageRobot")
            {
                paramName = "duration";
            }
            else
            {
                paramName = "engagement_duration"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
            qDebug() << "Extracted ApproachRobot duration:" << dur << "for block" << blockId << "agent" << agentIndex;
        }
    }
    else if (nodeId == "IsRobotClose")
    {
        if (const char *thresh = node->Attribute("threshold"))
        {
            // Different blocks use different parameter names
            QString paramName;
            if (blockId == "RobotAvoidance")
            {
                paramName = "close_threshold";
            }
            else if (blockId == "EngageRobot")
            {
                paramName = "proximity_threshold";
            }
            else if (blockId == "ProtectiveGuardian")
            {
                paramName = "robot_close_threshold";
            }
            else
            {
                paramName = "proximity_threshold"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(thresh).toDouble();
            qDebug() << "Extracted IsRobotClose threshold:" << thresh << "for block" << blockId << "agent" << agentIndex;
        }
    }
    else if (nodeId == "IsAgentVisible")
    {
        if (const char *dist = node->Attribute("distance"))
        {
            // Different blocks use different parameter names
            QString paramName;
            if (blockId == "ProtectiveGuardian")
            {
                paramName = "agent_visibility_distance";
            }
            else if (blockId == "GreetingInitiator")
            {
                paramName = "detection_distance";
            }
            else
            {
                paramName = "visibility_distance"; // Default for TalkInteract, FollowAgent, etc.
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
        }
    }
    else if (nodeId == "IsAgentClose")
    {
        if (const char *thresh = node->Attribute("threshold"))
        {
            // Different blocks use different parameter names for IsAgentClose threshold
            QString paramName;
            if (blockId == "TalkInteract")
            {
                paramName = "social_distance_threshold";
            }
            else if (blockId == "FollowAgent")
            {
                paramName = "is_close_threshold";
            }
            else
            {
                paramName = "social_distance_threshold"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(thresh).toDouble();
        }
    }
    else if (nodeId == "FollowAgent")
    {
        if (const char *agentId = node->Attribute("target_agent_id"))
        {
            QString agentIdStr = QString(agentId);
            // Skip if it's a blackboard reference (e.g., "{target_agent_id}")
            if (!agentIdStr.startsWith("{"))
            {
                QString key = QString("%1.target_agent_id").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = agentIdStr.toInt();
                qDebug() << "Extracted FollowAgent target_agent_id:" << agentIdStr << "for agent" << agentIndex;
            }
        }
        if (const char *dist = node->Attribute("closest_dist"))
        {
            QString key = QString("%1.following_distance").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
        }
        if (const char *vel = node->Attribute("max_vel"))
        {
            QString key = QString("%1.max_vel").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(vel).toDouble();
        }
        if (const char *dur = node->Attribute("duration"))
        {
            QString key = QString("%1.duration").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
        }
    }
    else if (nodeId == "ConversationFormation")
    {
        if (const char *agentId = node->Attribute("target_agent_id"))
        {
            QString agentIdStr = QString(agentId);
            // Skip if it's a blackboard reference (e.g., "{target_agent_id}")
            if (!agentIdStr.startsWith("{"))
            {
                QString key = QString("%1.target_agent_id").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = agentIdStr.toInt();
                qDebug() << "Extracted ConversationFormation target_agent_id:" << agentIdStr << "for agent" << agentIndex;
            }
        }
        if (const char *dur = node->Attribute("conversation_duration"))
        {
            QString key = QString("%1.conversation_duration").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
        }
        if (const char *goalId = node->Attribute("goal_id"))
        {
            QString key = QString("%1.goal_id").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(goalId).toInt();
        }
        if (const char *participants = node->Attribute("non_main_agent_ids"))
        {
            QString participantsStr = QString(participants);
            // Skip if it's a blackboard reference (e.g., "{target_agent_id}")
            if (!participantsStr.startsWith("{"))
            {
                QString key = QString("%1.non_main_agent_ids").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = participantsStr;
            }
        }
    }
    else if (nodeId == "GoTo")
    {
        if (const char *goals = node->Attribute("goal_ids"))
        {
            QString key = QString("%1.goal_ids").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(goals);
        }
    }
    else if (nodeId == "ApproachAgent")
    {
        // Extract target_agent_id
        if (const char *agentId = node->Attribute("target_agent_id"))
        {
            QString agentIdStr = QString(agentId);
            // Skip if it's a blackboard reference (e.g., "{target_agent_id}")
            if (!agentIdStr.startsWith("{"))
            {
                QString key = QString("%1.target_agent_id").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = agentIdStr.toInt();
                qDebug() << "Extracted ApproachAgent target_agent_id:" << agentIdStr << "for agent" << agentIndex;
            }
        }
        
        // Block-specific parameter extraction
        if (blockId == "ProtectiveGuardian")
        {
            if (const char *dist = node->Attribute("closest_dist"))
            {
                QString key = QString("%1.protective_approach_distance").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
            }
            if (const char *vel = node->Attribute("max_vel"))
            {
                QString key = QString("%1.protective_approach_velocity").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = QString(vel).toDouble();
            }
        }
        else // Default for other blocks
        {
            if (const char *dur = node->Attribute("duration"))
            {
                QString key = QString("%1.interaction_duration").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
            }
        }
    }
    else if (nodeId == "BlockRobot")
    {
        if (const char *dist = node->Attribute("front_dist"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "BlockingBehavior")
            {
                paramName = "front_dist";
            }
            else if (blockId == "ProtectiveGuardian")
            {
                paramName = "protective_front_distance";
            }
            else
            {
                paramName = "front_dist"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
            qDebug() << "Extracted BlockRobot front_dist:" << dist << "for block" << blockId << "agent" << agentIndex;
        }
        if (const char *dur = node->Attribute("duration"))
        {
            // Map to block-specific parameter name
            QString paramName;
            if (blockId == "BlockingBehavior")
            {
                paramName = "duration";
            }
            else if (blockId == "ProtectiveGuardian")
            {
                paramName = "blocking_duration";
            }
            else
            {
                paramName = "duration"; // Default
            }
            QString key = QString("%1.%2").arg(blockId).arg(paramName);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
            qDebug() << "Extracted BlockRobot duration:" << dur << "for block" << blockId << "agent" << agentIndex;
        }
    }
    else if (nodeId == "GroupWalk" || nodeId == "SetGroupWalk")
    {
        if (const char *agents = node->Attribute("agent_ids"))
        {
            QString agentsStr = QString(agents);
            // Skip if it's a blackboard reference
            if (!agentsStr.startsWith("{"))
            {
                QString key = QString("%1.group_agent_ids").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = agentsStr;
                qDebug() << "Extracted" << nodeId << "agent_ids:" << agentsStr << "for agent" << agentIndex;
            }
        }
        if (const char *dur = node->Attribute("duration"))
        {
            QString key = QString("%1.duration").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
            qDebug() << "Extracted" << nodeId << "duration:" << dur << "for block" << blockId << "agent" << agentIndex;
        }
    }
    else if (nodeId == "IsAnyoneLookingAtMe")
    {
        if (const char *dist = node->Attribute("distance_threshold"))
        {
            QString key = QString("%1.attention_distance").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
        }
        if (const char *dur = node->Attribute("duration"))
        {
            QString key = QString("%1.looking_duration").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
        }
    }
    else if (nodeId == "IsRobotFacingAgent")
    {
        if (const char *dist = node->Attribute("distance"))
        {
            QString key = QString("%1.attention_distance").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
        }
    }
    else if (nodeId == "SaySomething")
    {
        if (const char *msg = node->Attribute("message"))
        {
            QString msgStr = QString(msg);
            // Determine which block this belongs to based on context
            if (blockId == "AttentionSeeking")
            {
                QString key = QString("%1.response_message").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = msgStr;
            }
            else if (blockId == "GreetingInitiator")
            {
                QString key = QString("%1.greeting_message").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = msgStr;
            }
            else if (blockId == "SpeechDetection")
            {
                QString key = QString("%1.response_message").arg(blockId);
                agents_[agentIndex].agentSpecificParams[key] = msgStr;
            }
        }
    }
    else if (nodeId == "IsAnyoneSpeaking")
    {
        if (const char *dist = node->Attribute("distance_threshold"))
        {
            QString key = QString("%1.speech_distance").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dist).toDouble();
        }
        if (const char *dur = node->Attribute("duration"))
        {
            QString key = QString("%1.speaking_duration").arg(blockId);
            agents_[agentIndex].agentSpecificParams[key] = QString(dur).toDouble();
        }
    }

    qDebug() << "Extracted parameters from" << nodeId << "for agent" << agentIndex;
}

#include "BTConfigDialog.moc"