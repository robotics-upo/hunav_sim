#ifndef BTCONFIGDIALOG_H
#define BTCONFIGDIALOG_H

#include <QWizard>
#include <QWizardPage>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QTextEdit>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QListWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QSplitter>
#include <QFrame>
#include <QTableWidget>
#include <QTabWidget>
#include <QPushButton>
#include <QMap>
#include <QStringList>
#include <QScrollArea>
#include <QRandomGenerator>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <QFile>
#include <QDir>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QMessageBox>
#include <QDebug>

namespace hunav_rviz2_panel {
    class ActorPanel;
}

/**
 * @brief Enhanced Behavior Tree Configuration Dialog
 * 
 * This wizard provides a user-friendly interface for configuring agent behaviors
 * using high-level behavior blocks with proper agent ID handling.
 */
class BTConfigDialog : public QWizard
{
    Q_OBJECT

public:
    enum PageId {
        Page_Configuration = 0,     ///< Comprehensive configuration page (select + assign)
        Page_AgentConfig = 1,       ///< Per-agent configuration (ordering + parameters)
        Page_Preview = 2            ///< Preview the generated configuration
    };

    /**
     * @brief Configuration for a behavior block
     */
    struct BlockConfig {
        QString blockId;                        ///< Unique identifier for the block
        QString displayName;                    ///< User-friendly display name
        QString description;                    ///< Description of what the block does
        QStringList nodes;                      ///< BT nodes used by this block
        QMap<QString,QVariant> parameters;     ///< Block-level parameters
        bool isSelected = false;               ///< Whether this block is selected
        
        BlockConfig() = default;
        BlockConfig(const QString &id, const QString &name, const QString &desc, const QStringList &nodeList)
            : blockId(id), displayName(name), description(desc), nodes(nodeList) {}
    };

    /**
     * @brief Assignment of behaviors to a specific agent
     */
    struct AgentAssignment {
        int agentIndex;                         ///< Index of the agent
        QString agentName;                      ///< Name of the agent
        QString behaviorType;                   ///< Base behavior type (Regular, Scared, etc.)
        QStringList assignedBlocks;            ///< Which blocks this agent gets
        bool randomizeOrder = false;            ///< Whether to randomize block execution order
        QStringList runOnceBlocks;              ///< Blocks that should be wrapped in RunOnce tags
        QMap<QString, double> randomExecutionBlocks;  ///< Blocks with random execution (blockId -> probability)
        QMap<QString, QVariant> agentSpecificParams;  ///< Agent-specific parameter overrides
        QMap<QString,QVariant> parameters;     ///< Legacy parameter storage
        
        AgentAssignment() = default;
    };

    /**
     * @brief Complete configuration returned by the wizard
     */
    struct Config {
        QList<BlockConfig> availableBlocks;    ///< All available blocks with their config
        QList<AgentAssignment> agentAssignments; ///< Per-agent assignments
        QMap<QString,QVariant> parameters;     ///< Global parameters for all blocks
        
        Config() = default;
    };

    explicit BTConfigDialog(QWidget *parent = nullptr);
    ~BTConfigDialog();

    Config getConfig() const;

    void setAgentList(const QStringList &agentNames, const QStringList &behaviorTypes);
    void setAvailableGoals(const QList<int> &goalIds);
    
    // Load existing configuration from XML files
    bool loadExistingConfiguration(const QStringList &btPaths);
    
    static bool validateWizardConfig(const Config &config, int totalAgents);
    static QStringList generateBTPathsForScenario(const QString &scenarioName, 
                                                 const QString &simulator, 
                                                 int agentCount);
    static bool patchAllAgentBTFiles(const QStringList &btPaths, 
                                   const Config &config);

private slots:
    void onTemplateChanged(int index);
    void onBlockConfigChanged();

private:
    // ===================== UI CONSTRUCTION =====================
    
    void buildPages();
    void buildPreviewPage();
    
    // ===================== COMBINED CONFIGURATION PAGE =====================
    
    // Main page building and updating
    void buildCombinedConfigurationPage();
    
    // Assignment matrix (right side)
    QWidget* createCellWidgetForBlock(const BlockConfig &block, int agentIndex, int blockRow = -1);
    void clearAllAssignments();

    // New comprehensive configuration methods
    void initializeBlocks();
    QWidget* createAssignmentWidget(); 
    QString getBlockIcon(const QString &blockId) const; 
    QWidget* createPerAgentConfigWidget();
    
    // assignment methods
    void initializeFullAssignmentMatrix();
    void updateAssignmentMatrix();
    QCheckBox* getCheckBoxFromMatrixCell(int row, int col) const;
    void updateAgentAssignmentFromMatrix(int row, int col, bool checked);
    void assignSelectedBlocksToAllAgents();
    void buildAgentConfigurationPage();
    void updateAgentDropdowns();
    void updateConfigurationPanel();
    void populateConfigurationContent();  
    void createBlockParametersForAgent(const QString &blockId, QFormLayout *formLayout, int agentIndex);
    
    // Execution order list creation
    QWidget* createOrderingList(const QStringList &blockOrder, int selectedAgent);
    void recreateVisualBlocks(QWidget* orderingContainer, const QStringList& blockOrder, int selectedAgent, const QString& selectedBlockId = QString());
    
    // Parameter display methods for click-to-show functionality
    void updateParametersForSelectedBlock(const QString &blockId, int agentIndex, QVBoxLayout *parametersLayout, QWidget *parametersContentWidget);
    void showParametersInstruction(QVBoxLayout *parametersLayout, QWidget *parametersContentWidget);
    void setupParameterUpdateConnections(const QString &blockId, int agentIndex); 
    void setupAllParameterConnections(); 
    void clearParameterConnectionTracking(); 
    
    // Parameter connection tracking to prevent duplicates
    QSet<QString> setupTracker_;  
    QSet<QString> connectedWidgets_;   
    
    // New preview page methods
    QWidget* createAssignmentSummary();
    QWidget* createXMLPreview();
    void updateAssignmentSummaryWidget();
    void updateXMLPreview();
    void updateXMLPreviewForAgent(int agentIndex);
    void applyConfiguration();
    
    // Helper methods
    QList<BlockConfig> getSelectedBlocks() const;
    
    // ===================== PAGE TRANSITIONS AND VALIDATION =====================
    
    void setupPageTransitions();
    bool validateConfiguration(const Config &config) const;
    void updateValidationFeedback();
    void accept() override;
    
    // ===================== PREVIEW GENERATION =====================
    
    void generateFinalPreview();

    // ===================== AGENT ASSIGNMENT METHODS =====================
    
    void updateAgentAssignment(int agentIndex, const QString &blockId, bool assigned);
    

    // ===================== CONFIGURATION AND FILE MANAGEMENT =====================
    
    static bool patchAgentBtFile(const QString &btPath, int agentIndex, const Config &cfg);
    static void applyBlockParametersToNode(tinyxml2::XMLElement *nodeElem, 
                                         const QString &blockId, 
                                         const QString &nodeId,
                                         const Config &cfg);
    static void applyAgentSpecificParametersToNode(tinyxml2::XMLElement *nodeElem,
                                                  int agentIndex,
                                                  const QString &blockId, 
                                                  const QString &nodeId,
                                                  const AgentAssignment &assignment);
    static QString mapParameterToXMLAttribute(const QString &blockId, 
                                            const QString &nodeId, 
                                            const QString &paramName);
    static QStringList getEnhancedNodesForBlock(const QString &blockId);
    
    // ===================== XML PARSING HELPERS =====================
    
    bool parseAgentXML(const QString &xmlPath, int agentIndex);
    QString identifyBlockFromSequence(tinyxml2::XMLElement *sequence);
    void extractParametersFromNode(tinyxml2::XMLElement *node, 
                                   const QString &blockId,
                                   int agentIndex);
    void inferBlockMode(tinyxml2::XMLElement *sequence,
                       const QString &blockId,
                       int agentIndex);
    
    // ===================== HELPER METHODS =====================
    
    Config getEnhancedConfig() const;
    
    // Optimization helper methods
    int getAgentCount() const;                ///< Get effective agent count
    int getAssignmentMatrixAgentCount() const; ///< Get agent count from assignment matrix
    void updateButtonSelectionState(QPushButton* selectedButton); ///< Update button selection visual state
    
    // ===================== UI STATE MANAGEMENT =====================
    
    // Combined page components
    QTableWidget *assignmentMatrix_;          ///< Matrix showing block-to-agent assignments  
    QVBoxLayout *blockConfigContainer_;       ///< Container for block configuration widgets
    QWidget *simplifiedAssignmentMatrix_;     ///< Reference to simplified assignment matrix
    QTabWidget *configTabs_;                  ///< Configuration tabs widget
    
    // New comprehensive configuration components
    QComboBox *agentConfigCombo_;             ///< Agent selection for configuration
    QScrollArea *configScrollArea_;           ///< Scroll area for configuration widgets
    QWidget *configContentWidget_;           ///< Content widget for configuration
    QVBoxLayout *configContentLayout_;       ///< Layout for configuration content
    
    // Block selection page components
    QListWidget *selectedBlocksList_;         ///< List of selected blocks
    
    // Preview page components
    QVBoxLayout *summaryLayout_;              ///< Layout for summary content
    QComboBox *agentPreviewCombo_;            ///< Combo for selecting preview agent
    QTextEdit *previewEdit_;                  ///< XML preview text area
    
    // New preview page components
    QScrollArea *assignmentSummaryArea_;      ///< Scroll area for assignment summary
    QWidget *assignmentSummaryWidget_;        ///< Widget for assignment summary content
    QVBoxLayout *assignmentSummaryLayout_;    ///< Layout for assignment summary
    QTextEdit *xmlPreviewEdit_;               ///< XML preview text editor
    QPushButton *applyBtn_;                   ///< Apply configuration button
    
    // Data storage
    QList<BlockConfig> blocks_;               ///< Available behavior blocks
    QList<AgentAssignment> agents_;           ///< Agent assignments
    QMap<QString, QWidget*> blockConfigWidgets_; ///< Parameter configuration widgets
    QMap<int, QLabel*> statusIndicators_;    ///< Agent status indicators (legacy)
    
    // Agent information
    QStringList agentNames_;                  ///< List of agent names
    QStringList behaviorTypes_;               ///< List of agent behavior types
    QList<int> availableGoals_;               ///< List of available goal IDs
    
    // Documentation and tooltips
    QMap<QString, QString> nodeDescriptions_; ///< Node descriptions for tooltips
    
    // Statistics display for preview page
    struct {
        QLabel *totalBlocks;                  ///< Total selected blocks label
        QLabel *totalAgents;                  ///< Total agents label
        QLabel *totalAssignments;             ///< Total assignments label
        QLabel *executionMode;                ///< Execution mode label
    } statsLabels_;
    
    // ===================== INTERNAL STATE =====================
    
    Config config_;                           ///< Current configuration state
    QString templateFile_;                    ///< Selected template file
    tinyxml2::XMLDocument *xmlDoc_;          ///< Loaded XML document
    bool isUpdatingConfiguration_;            ///< Guard to prevent concurrent configuration updates
    hunav_rviz2_panel::ActorPanel* actorPanel_;  ///< Pointer to parent ActorPanel for path conversion
};

#endif // BTCONFIGDIALOG_H