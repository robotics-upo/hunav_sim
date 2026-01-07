/**
 * @file actor_panel.hpp
 * @brief ActorPanel class for creating and managing human agents in RViz2
 * 
 * This panel provides functionality to:
 * - Create new agent configurations or edit existing ones
 * - Define agent initial poses and goals
 * - Configure agent behaviors and appearance
 * - Generate YAML configuration files and behavior trees
 * - Visualize agents and goals in RViz2
 * 
 */

#ifndef ACTOR_PANEL_HPP
#define ACTOR_PANEL_HPP

// ================================ SYSTEM INCLUDES ================================
#include <memory>
#include <string>
#include <vector>

// ================================ QT INCLUDES ================================
#include <QtWidgets>
#include <QBasicTimer>

#undef NO_ERROR

// ================================ ROS2 INCLUDES ================================
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"

// ================================ RVIZ INCLUDES ================================
#include "rviz_common/panel.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/message_filter_display.hpp"

// ================================ MESSAGE INCLUDES ================================
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

// ================================ TRANSFORM INCLUDES ================================
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

// ================================ THIRD-PARTY INCLUDES ================================
#include "yaml-cpp/yaml.h"
#include "nav2_util/geometry_utils.hpp"

// ================================ LOCAL INCLUDES ================================
#include "BTConfigDialog.h"

// ================================ FORWARD DECLARATIONS ================================
class QLineEdit;
class BTConfigDialog;

namespace hunav_rviz2_panel
{
  /**
   * @brief Operating modes for the ActorPanel
   */
  enum PanelMode
  {
    CREATE_MODE,    ///< Mode for creating new agent configurations
    EDIT_MODE,      ///< Mode for editing existing agent configurations
  };

  /**
   * @brief RViz2 Panel for creating and managing human agents
   * 
   * This panel provides a comprehensive interface for:
   * - Creating and editing agent configurations
   * - Loading/saving YAML configuration files
   * - Interactive goal assignment through map clicking
   * - Behavior tree generation and editing
   * - Multi-simulator support (Gazebo, Isaac Sim, Webots)
   */
  class ActorPanel : public rviz_common::Panel, public rclcpp::Node
  {
    Q_OBJECT

    public:
    /**
     * @brief Constructor for ActorPanel
     * @param parent Parent widget
     */
    ActorPanel(QWidget *parent = 0);
    
    /**
     * @brief Destructor
     */
    ~ActorPanel();
    
    /**
     * @brief Initialize the panel after RViz context is available
     */
    virtual void onInitialize() override;

    /**
     * @brief Load panel configuration from RViz config
     * @param config RViz configuration object
     */
    virtual void load(const rviz_common::Config &config);
    
    /**
     * @brief Save panel configuration to RViz config
     * @param config RViz configuration object
     */
    virtual void save(rviz_common::Config config) const;

    /**
     * @brief Convert share directory path to src directory path
     * @param share_path Path to shared package directory
     * @return Path to source directory
     * 
     * Converts install/share paths to src paths
     * Example:
     * Input: /home/workspace/install/hunav_gazebo_wrapper/share/hunav_gazebo_wrapper
     * Output: /home/workspace/src/hunav_gazebo_wrapper
     */
    std::string share_to_src_path(const std::string& install_share_path);

  public Q_SLOTS:
    /**
     * @brief Set the topic name
     * @param topic Topic name
     */
    void setTopic(const QString &topic);

  protected Q_SLOTS:
    // ================================ AGENT MANAGEMENT ================================
    /**
     * @brief Add a new agent to the configuration
     */
    void addAgent();

    /**
     * @brief Handle agent addition button click (only if in edit mode)
     */
    void onAddAgent();

    /**
     * @brief Save all agents and generate behavior trees
     */
    void saveAndGenerateAll();
    
    /**
     * @brief Handle create or edit agents button click
     */
    void onCreateOrEditAgents();

    /**
     * @brief Reset panel to initial state
     */
    void resetPanel();

    // ================================ POSE AND GOAL HANDLING ================================
    /**
     * @brief Handle initial pose setting
     * @param x X coordinate
     * @param y Y coordinate  
     * @param theta Orientation angle
     * @param frame Reference frame
     */
    void onInitialPose(double x, double y, double theta, QString frame);
    
    /**
     * @brief Set initial pose for current agent
     */
    void setInitialPose();
    
    /**
     * @brief Close initial pose window
     */
    void closeInitialPoseWindow();
    
    /**
     * @brief Enter goal picking mode for interactive goal assignment
     */
    void onEnterGoalPickingMode();
    
    /**
     * @brief Handle goal assignment to agents
     */
    void onAssignGoalsClicked();
    
    /**
     * @brief Handle clicked goal point from RViz
     * @param msg Point stamped message from clicked point
     */
    void onGoalPicked(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief Reset loaded goals to original state
     */
    void onResetLoadedGoals();

    void rebuildGoalListWidget();

    // ================================ UI CONFIGURATION ================================
    /**
     * @brief Check combo box selection and return value
     * @return Selected combo box value
     */
    int checkComboBox();
    
    /**
     * @brief Check skin combo box selection
     * @return Selected skin value
     */
    int checkComboBoxSkin();
    
    /**
     * @brief Check behavior configuration combo box
     */
    void checkComboBoxConf();
    
    /**
     * @brief Parse skin configuration
     * @param skin Skin identifier
     */
    void checkParserSkin(int skin);

    /**
     * @brief Update buttons layout based on panel mode
     */
    void switchButtonLayout(PanelMode mode);

    // ================================ FILE OPERATIONS ================================
    /**
     * @brief Parse YAML configuration file
     */
    void parseYaml();
    
    /**
     * @brief Open file explorer dialog
     * @param file Whether to select file or directory
     * @return Selected path
     */
    std::string openFileExplorer(bool file);
    
    /**
     * @brief Handle map selection
     */
    void onSelectMap();

    // ================================ VISUALIZATION ================================
    /**
     * @brief Generate random RGB color values
     */
    void randomRGB();
    
    /**
     * @brief Create marker for visualization
     * @param point1_x X coordinate
     * @param point1_y Y coordinate
     * @param ids Marker ID
     * @param marker_shape Shape of marker
     * @param create_or_parser Creation mode
     * @return Created marker
     */
    visualization_msgs::msg::Marker createMarker(double point1_x, double point1_y, double ids, std::string marker_shape,
                                                 std::string create_or_parser);
    
    /**
     * @brief Create arrow marker for visualization
     * @param point1_x Start X coordinate
     * @param point1_y Start Y coordinate
     * @param point2_x End X coordinate
     * @param point2_y End Y coordinate
     * @param ids Marker ID
     * @return Created arrow marker
     */
    visualization_msgs::msg::Marker createArrowMarker(double point1_x, double point1_y, double point2_x, double point2_y,
                                                      double ids);
    
    /**
     * @brief Create agent label marker
     * @param x X coordinate
     * @param y Y coordinate
     * @param id Agent ID
     * @param frame_id Reference frame
     * @return Created label marker
     */
    visualization_msgs::msg::Marker createAgentLabel(
        double x, double y, int id, const std::string &frame_id);
    
    /**
     * @brief Remove current markers from display
     */
    void removeCurrentMarkers();
    
    /**
     * @brief Clear non-agent markers
     */
    void clearNonAgentMarkers();
    
    /**
     * @brief Initialize agent colors for visualization
     * @param num_agents Number of agents
     */
    void initAgentColors(int num_agents);
    
    /**
     * @brief Reset goal marker colors
     */
    void resetGoalMarkerColors();
    
    /**
     * @brief Publish agent markers
     */
    void publishAgentMarkers();
    
    /**
     * @brief Clear displayed map
     */
    void clearDisplayedMap();

    // ================================ BEHAVIOR TREE OPERATIONS ================================
    /**
     * @brief Edit all behavior trees in Groot
     */
    void onEditAllInGroot();

  public:
    // ================================ UI COMPONENTS ================================
    
    // Input fields
    QLineEdit *actors;                    ///< Input field for number of agents
    QLineEdit *agent_desired_vel;         ///< Input field for agent desired velocity
    QLineEdit *coordinates;               ///< Input field for coordinates
    QLineEdit *coordinates1;              ///< Input field for coordinates 1
    QLineEdit *coordinates2;              ///< Input field for coordinates 2
    
    // Dialog windows
    QWidget *window = nullptr;            ///< Main agent configuration window
    QWidget *window1 = nullptr;           ///< Secondary window
    QWidget *window2 = nullptr;           ///< Tertiary window
    QDialog *agent_dialog_ = nullptr;     ///< Agent configuration dialog
    QDialog *initial_pose_dlg_ = nullptr;               ///< General dialog for initial pose
    QWidget *edit_mode_widget_;           ///< Widget for edit mode
    
    // Combo boxes
    QComboBox *skin_combobox;             ///< Combo box for agent skin selection
    QComboBox *simulator_combo_{nullptr}; ///< Combo box for simulator selection
    QComboBox *behavior_type_combobox;    ///< Combo box for behavior type selection
    QComboBox *behavior_conf_combobox;    ///< Combo box for behavior configuration
    QComboBox *simulator_in_dialog_;      ///< Simulator combo box in dialog
    
    // Buttons
    QPushButton *actor_button_ = nullptr;           ///< Main actor creation button
    QPushButton *edit_goals_button_ = nullptr;      ///< Edit goals button
    QPushButton *add_agent_button_;                 ///< Add agent button
    QPushButton *map_select_btn_;                   ///< Map selection button
    QPushButton *initial_pose_button;               ///< Initial pose setting button
    QPushButton *initial_pose_button_dlg_;          ///< Initial pose dialog button
    QPushButton *save_button_;                      ///< Save configuration button
    QPushButton *edit_bt_btn_;                      ///< Edit behavior tree button
    QPushButton *save_bt_btn_{nullptr};             ///< Save behavior tree button
    QPushButton *open_button_;                      ///< Open YAML file button
    QPushButton *create_button_;                    ///< Create new configuration button
    QPushButton *edit_agents_button_;               ///< Edit agents button
    QPushButton *next_agent_button_ = nullptr;      ///< Next agent button
    QPushButton *enter_goal_mode_btn_{nullptr};     ///< Enter goal picking mode button
    QPushButton *reset_goals_button_;               ///< Reset goals button
    QPushButton *assign_goals_btn_{nullptr};        ///< Assign goals to agents button
    QPushButton *reset_button_;                     ///< Reset panel button
    QAbstractButton *finishBtn_ = nullptr;          ///< Finish goals assignment button

    // Labels
    QLabel *current_map_label_;           ///< Label showing current map
    QLabel *sim_map_label_ = nullptr;     ///< Simulator and map label
    QLabel *n_agents_label_ = nullptr;    ///< Number of agents label
    QLabel *yaml_file_label_;             ///< YAML file label
    QLabel *skin_label_ = nullptr;        ///< Skin selection label
    QLabel *goals_remaining;              ///< Goals remaining label
    QLabel *statusLabel_;

    // Behavior configuration labels and inputs
    QLabel *dur;                          ///< Duration label
    QLineEdit *beh_duration;              ///< Behavior duration input
    QLabel *once;                         ///< Once label
    QLineEdit *beh_once;                  ///< Behavior once input
    QLabel *vel;                          ///< Velocity label
    QLineEdit *beh_vel;                   ///< Behavior velocity input
    QLabel *dist;                         ///< Distance label
    QLineEdit *beh_dist;                  ///< Behavior distance input
    QLabel *gff;                          ///< Goal force factor label
    QLineEdit *beh_gff;                   ///< Goal force factor input
    QLabel *off;                          ///< Obstacle force factor label
    QLineEdit *beh_off;                   ///< Obstacle force factor input
    QLabel *sff;                          ///< Social force factor label
    QLineEdit *beh_sff;                   ///< Social force factor input
    QLabel *other;                        ///< Other force factor label
    QLineEdit *beh_otherff;               ///< Other force factor input
    QLabel *stop;                         ///< Stop distance label
    QLineEdit *beh_stop_dist;             ///< Stop distance input
    QLabel *front;                        ///< Front distance label
    QLineEdit *beh_front_dist;            ///< Front distance input
    QLabel *safe_dist;                    ///< Safe distance label
    QLineEdit *beh_safe_distance;         ///< Safe distance input

    // Layouts
    QHBoxLayout *main_layout_;            ///< Main horizontal layout
    QVBoxLayout *topic_button_layout_;    ///< Topic button layout
    QVBoxLayout *topic_layout_init_pose_;  ///< Initial pose layout
    QVBoxLayout *goals_layout;            ///< Goals layout
    QVBoxLayout *topic_layout = nullptr;  ///< Topic layout
    QVBoxLayout *summary_area_{nullptr};  ///< Summary area layout
    QVBoxLayout *create_mode_layout_;     ///< Create mode layout
    QHBoxLayout *edit_mode_layout_;       ///< Edit mode layout
    
    // Group boxes
    QGroupBox *map_group;                 ///< Map selection group
    QGroupBox *bt_group_;                 ///< Behavior tree group
    QGroupBox *goal_group_{nullptr};      ///< Goal assignment group
    
    // Other UI components
    QCheckBox *checkbox;                  ///< Default directory checkbox
    QCheckBox *cyclic_goals_checkbox;     ///< Cyclic goals checkbox
    QListWidget *goal_list_widget_{nullptr}; ///< List widget for goals
    
    // ================================ CONFIGURATION DATA ================================
    
    // String data
    QString output_topic_;                ///< Current output topic name
    QString defaultName_;                 ///< Default name
    QString yaml_base_name_;              ///< Base name for YAML file
    QString orig_yaml_base_name_;         ///< Original YAML base name
    QString map_file_;                    ///< Selected map file path
    QString map_name_;                    ///< Map name
    QString btBlock_;                     ///< Behavior tree block name
    std::string person_skin;              ///< Selected person skin
    std::string pkg_shared_tree_dir_;     ///< Package shared tree directory
    std::string dir;                      ///< Directory path
    
    // Numerical data
    int iterate_actors_ = 1;              ///< Actor iteration counter
    int num_actors_ = 0;                  ///< Number of actors
    int num_actors;                       ///< Number of actors (duplicate)
    int num_agents = 1;                   ///< Number of agents
    int iterate_actors = 1;               ///< Actor iteration counter (duplicate)
    int goals_number = 1;                 ///< Number of goals
    int goals_to_remove = 0;              ///< Goals to remove counter
    int moving_goal_id_ = -1;             ///< 0 means “not currently moving any goal”
    int marker_id = 1;                    ///< Marker ID counter
    int agent_count = 1;                  ///< Agent count
    int next_marker_id_ = 0;              ///< Next marker ID
    int current_edit_idx_ = 0;            ///< Current edit index
    double simulator_z_offset_{0.0};      ///< Z offset for simulator
    
    // Boolean flags
    PanelMode panel_mode_ = CREATE_MODE;  ///< Current panel mode
    bool goal_picking_mode_{false};       ///< Goal picking mode flag
    bool is_edit_mode_ = false;           ///< Edit mode flag
    bool first_pose_popup_shown_ = false; ///< First pose popup shown flag
    bool first_actor_ = false;            ///< First actor flag
    bool initial_pose_set = false;        ///< Initial pose set flag
    bool show_file_selector_once = true;  ///< Show file selector once flag
    bool first_goal_picking_info_shown_{false}; ///< First goal picking info shown flag
    bool initial_pose_tip_shown_{false};  ///< Initial pose dialog shown flag
    bool adding_new_agent_ = false;       ///< Adding new agent flag

    // ================================ DATA CONTAINERS ================================
    
    // YAML and configuration data
    std::vector<YAML::Node> actors_info_; ///< Actor information from YAML
    std::vector<YAML::Node> actors_info;  ///< Actor information (duplicate)
    YAML::Node params_;                   ///< YAML parameters
    QLineEdit *yaml_name_edit_;           ///< YAML name edit field
    
    // Agent data
    std::vector<std::string> names;                        ///< Agent names
    std::vector<std::string> point;                        ///< Point data
    std::vector<std::vector<int>> agent_goals_;            ///< Agent goals mapping
    std::vector<QColor> agent_colors_;                     ///< Agent colors for visualization
    std::vector<QString> goals;                            ///< Goals list
    std::vector<geometry_msgs::msg::Pose> goals_;          ///< Stored clicked goals
    
    // Loaded data for editing mode
    std::map<int, geometry_msgs::msg::Point> loaded_global_goals_;  ///< Loaded global goals
    std::vector<std::string> loaded_agent_names_;                   ///< Loaded agent names
    std::vector<YAML::Node> loaded_agent_nodes_;                    ///< Loaded agent nodes
    std::vector<std::vector<int>> loaded_agent_goals_;              ///< Loaded agent goals
    std::vector<int> loaded_initial_marker_ids_;                    ///< Loaded initial marker IDs
    std::vector<int> goal_ids_;                                     ///< Goal IDs
    
    // ================================ GEOMETRY AND VISUALIZATION ================================
    
    // Pose data
    geometry_msgs::msg::PoseStamped initial_pose;          ///< Initial pose
    std::vector<geometry_msgs::msg::PoseStamped> poses;    ///< All poses
    geometry_msgs::msg::PoseStamped pose;                  ///< Current pose
    geometry_msgs::msg::PoseStamped oldPose;               ///< Previous pose
    geometry_msgs::msg::PoseStamped stored_pose;           ///< Stored pose
    
    // Markers and visualization
    std::vector<visualization_msgs::msg::Marker> markers_array_to_remove; ///< Markers to remove
    std::vector<visualization_msgs::msg::Marker> arrows_markers_array;    ///< Arrow markers
    visualization_msgs::msg::MarkerArray initial_pose_marker_array;       ///< Initial pose markers
    visualization_msgs::msg::MarkerArray marker_array;                    ///< General marker array
    visualization_msgs::msg::MarkerArray goal_markers_;                   ///< Goal markers
    
    // Colors for visualization
    std::vector<double> rgb{255, 0};      ///< RGB color values
    double red;                           ///< Red color component
    double green;                         ///< Green color component
    double blue;                          ///< Blue color component
    
    // ================================ ROS2 COMMUNICATION ================================
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr initial_pose_publisher; ///< Initial pose publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goals_publisher;        ///< Goals publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_markers_pub_;      ///< Goal markers publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;                       ///< Map publisher
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;               ///< Goal subscription
    
    // Node handles
    rclcpp::Node::SharedPtr node_handle_;                  ///< Node handle
    rclcpp::Node::SharedPtr client_node_;                  ///< Client node handle
    
    // ================================ QT CONNECTIONS ================================
    
    QObject *initial_pose_connection_;                      ///< Initial pose connection object
    QObject *goals_connection;                             ///< Goals connection object
    QMetaObject::Connection *conn_delete = new QMetaObject::Connection(); ///< Connection to delete
    QMetaObject::Connection initial_pose_conn_;            ///< Initial pose connection
  
  
    // ===================== BT WIZARD INTEGRATION =====================

    // Block Definition Structure
    struct BlockDefinition {
        QString blockId;
        QString displayName;
        QString description;
        QStringList requiredNodes;
        QMap<QString, QVariant> defaultParameters;
        
        BlockDefinition() = default;
        BlockDefinition(const QString &id, const QString &name, const QString &desc, const QStringList &nodes)
            : blockId(id), displayName(name), description(desc), requiredNodes(nodes) {}
    };

  private:               

    QString buildGoalSequence(int agentIndex);

    // Enhanced parameter handling
    QMap<QString, QVariant> resolveEnhancedParametersForAgent(int agentIndex, const QString &blockId, const QString &nodeId) const;
        
    // Multi-Agent Block Configuration Structure
    struct AgentBTConfig {
        QString templateFile;
        QStringList selectedBlockIds;                    // Which blocks were selected
        QMap<QString, QVariant> globalBlockParameters;  // Global block-level parameters
        QMap<int, QStringList> agentBlockAssignments;   // Per-agent block assignments
        QMap<int, QMap<QString, QVariant>> agentOverrides; // Per-agent parameter overrides
        QMap<int, bool> agentRandomization;              // Per-agent randomization setting
        bool isCustomized = false;

        BTConfigDialog::Config lastWizardConfig_;
        
        // Constructor
        AgentBTConfig() = default;
        
        // Clear all configuration
        void clear() {
            templateFile.clear();
            selectedBlockIds.clear();
            globalBlockParameters.clear();
            agentBlockAssignments.clear();
            agentOverrides.clear();
            agentRandomization.clear();
            isCustomized = false;
        }
        
        // Check if configuration is valid
        bool isValid() const {
            return isCustomized && !selectedBlockIds.isEmpty() && !agentBlockAssignments.isEmpty();
        }
    };
    
    // ===================== STATE MANAGEMENT =====================
    
    // Current agent BT configuration
    AgentBTConfig agentBTConfig_;
    
    // Wizard state
    std::vector<QString> agentBtPaths_;
    QString currentXmlPath_;
    BTConfigDialog::Config lastWizardConfig_;
    
    // ===================== CORE WORKFLOW METHODS =====================
    
    // Main wizard integration
    void launchBTWizard();
    void resetBTConfiguration(const QStringList &btPaths = QStringList());
    
    // BT generation
    QString generateDefaultBTForAgent(int agentIndex);

    // LLM-based BT generation (future extension)
    void launchLLMBTGenerator(); 
    
    // ===================== UTILITIES =====================
    
    // File operations
    QString loadFile(const QString &filepath);
    
    // ===================== CONSTANTS =====================
    
    static const QString DEFAULT_BT_TEMPLATE;
    static const double DEFAULT_TIME_STEP;
    static const double DEFAULT_DISTANCE_TOLERANCE;
    static const double DEFAULT_SPEED_MULTIPLIER;
};

} // namespace hunav_rviz2_panel

#endif // ACTOR_PANEL_HPP