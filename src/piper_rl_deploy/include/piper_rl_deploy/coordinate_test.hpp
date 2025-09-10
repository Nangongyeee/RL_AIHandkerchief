#ifndef COORDINATE_TEST_HPP
#define COORDINATE_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vicon_msgs/msg/markers.hpp>
#include <vicon_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <algorithm>

/**
 * @brief CoordinateTestNode class for coordinate transformation and data publishing
 * 
 * This node handles coordinate transformations between different reference frames:
 * - Transforms Vicon world coordinates to Piper_root coordinate system
 * - Processes handkerchief position and velocity data
 * - Handles end effector pose transformations
 * - Publishes transformed data and TF frames for visualization
 */
class CoordinateTestNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for CoordinateTestNode
     */
    CoordinateTestNode();

    /**
     * @brief Destructor for CoordinateTestNode
     */
    ~CoordinateTestNode() = default;

private:
    // === Member Variables ===
    
    // Pose data storage
    geometry_msgs::msg::PoseStamped::SharedPtr root_pose_;           ///< Root pose from Vicon
    geometry_msgs::msg::PoseStamped::SharedPtr handkerchief_pose_;   ///< Handkerchief pose from Vicon
    geometry_msgs::msg::Pose::SharedPtr end_effector_pose_;          ///< End effector pose from robot
    vicon_msgs::msg::Markers::SharedPtr markers_data_;               ///< All Vicon markers data
    
    // Specific marker coordinates storage
    std::map<std::string, geometry_msgs::msg::Point> marker_positions_;
    geometry_msgs::msg::Point root_markers_center_;                  ///< Geometric center of root markers
    bool root_markers_center_valid_ = false;                        ///< Flag indicating if root center is valid
    geometry_msgs::msg::Point cloth_markers_center_;                 ///< Geometric center of cloth markers
    bool cloth_markers_center_valid_ = false;                       ///< Flag indicating if cloth center is valid
    Eigen::Vector3d prev_handkerchief_position_{0.0, 0.0, 0.0};     ///< Previous handkerchief position for fallback
    bool has_previous_handkerchief_position_ = false;               ///< Flag for valid previous position
    
    // Velocity calculation state
    std::vector<double> prev_relative_pos_{0.0, 0.0, 0.0};          ///< Previous handkerchief position in Piper_root frame
    std::vector<double> prev_velocity_{0.0, 0.0, 0.0};              ///< Previous velocity for filtering
    rclcpp::Time prev_time_;                                         ///< Previous timestamp for velocity calculation
    bool first_callback_ = true;                                     ///< Flag for first callback handling
    
    // === ROS2 Publishers and Subscribers ===
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr root_sub_;           ///< Root pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_sub_;   ///< Handkerchief pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr end_effector_sub_;          ///< End effector pose subscriber
    rclcpp::Subscription<vicon_msgs::msg::Markers>::SharedPtr markers_sub_;               ///< Vicon markers subscriber
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_twist_pub_;        ///< Handkerchief velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr root_new_pub_;          ///< Piper_root pose publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_pub_;      ///< Handkerchief in Piper_root frame publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pub_;      ///< End effector in Piper_root frame publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr markers_pub_;            ///< 8 target markers positions publisher
    
    // Timer and TF broadcaster
    rclcpp::TimerBase::SharedPtr timer_;                                                  ///< Main processing timer
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                       ///< TF broadcaster for visualization

    // === Callback Functions ===
    
    /**
     * @brief Callback for root pose messages from Vicon
     * @param msg Pose message from /vicon/root0908/root0908 topic
     */
    void rootCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Callback for handkerchief pose messages from Vicon
     * @param msg Pose message from /vicon/cloth0908/cloth0908 topic
     */
    void handkerchiefCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Callback for end effector pose messages from robot
     * @param msg Pose message from /end_pose topic
     */
    void endEffectorCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

    /**
     * @brief Callback for Vicon markers data
     * @param msg Markers message from /vicon/markers topic
     */
    void markersCallback(const vicon_msgs::msg::Markers::SharedPtr msg);

    // === Main Processing Functions ===
    
    /**
     * @brief Main processing function called by timer
     * 
     * This function performs:
     * 1. Coordinate transformation from Vicon world to Piper_root
     * 2. Handkerchief position and velocity calculation in Piper_root frame
     * 3. End effector pose transformation to Piper_root frame
     * 4. Publishing all transformed data and TF frames
     */
    void publishData();

    // === Utility Functions ===
    
    /**
     * @brief Calculate Piper_root coordinate system from root pose
     * @param root_position Root position in world coordinates
     * @param root_quat Root orientation quaternion in world coordinates
     * @param[out] piper_root_position Calculated Piper_root position
     * @param[out] piper_root_quat Calculated Piper_root orientation
     * @param[out] rotation_matrix Rotation matrix for coordinate transformation
     */
    void calculatePiperRootTransform(
        const Eigen::Vector3d& root_position,
        const Eigen::Quaterniond& root_quat,
        Eigen::Vector3d& piper_root_position,
        Eigen::Quaterniond& piper_root_quat,
        Eigen::Matrix3d& rotation_matrix
    );

    /**
     * @brief Transform handkerchief pose to Piper_root coordinate system
     * @param handkerchief_world Handkerchief position in world coordinates
     * @param handkerchief_quat Handkerchief orientation in world coordinates
     * @param piper_root_position Piper_root position in world coordinates
     * @param piper_root_quat Piper_root orientation quaternion
     * @param rotation_matrix Piper_root rotation matrix
     * @param[out] handkerchief_in_piper Handkerchief position in Piper_root frame
     * @param[out] handkerchief_quat_in_piper Handkerchief orientation in Piper_root frame
     */
    void transformHandkerchiefToPiperRoot(
        const Eigen::Vector3d& handkerchief_world,
        const Eigen::Quaterniond& handkerchief_quat,
        const Eigen::Vector3d& piper_root_position,
        const Eigen::Quaterniond& piper_root_quat,
        const Eigen::Matrix3d& rotation_matrix,
        Eigen::Vector3d& handkerchief_in_piper,
        Eigen::Quaterniond& handkerchief_quat_in_piper
    );

    /**
     * @brief Calculate handkerchief velocity in Piper_root frame with filtering
     * @param current_position Current handkerchief position in Piper_root frame
     * @param current_time Current timestamp
     * @param[out] velocity_vec Calculated and filtered velocity vector
     * @return true if velocity calculation is valid, false otherwise
     */
    bool calculateHandkerchiefVelocity(
        const Eigen::Vector3d& current_position,
        const rclcpp::Time& current_time,
        Eigen::Vector3d& velocity_vec
    );

    /**
     * @brief Transform end effector pose to Piper_root coordinate system
     * @param end_effector_pose End effector pose in robot base frame
     * @param[out] end_effector_in_piper End effector position in Piper_root frame
     * @param[out] end_effector_quat_in_piper End effector orientation in Piper_root frame
     */
    void transformEndEffectorToPiperRoot(
        const geometry_msgs::msg::Pose& end_effector_pose,
        Eigen::Vector3d& end_effector_in_piper,
        Eigen::Quaterniond& end_effector_quat_in_piper
    );

    /**
     * @brief Publish TF transform
     * @param parent_frame Parent frame ID
     * @param child_frame Child frame ID
     * @param position Position vector
     * @param orientation Orientation quaternion
     */
    void publishTF(
        const std::string& parent_frame,
        const std::string& child_frame,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation
    );

    /**
     * @brief Process specific markers and update marker positions map
     */
    void processSpecificMarkers();

    /**
     * @brief Get marker position by name
     * @param marker_name Name of the marker
     * @return Pointer to marker position if found, nullptr otherwise
     */
    const geometry_msgs::msg::Point* getMarkerPosition(const std::string& marker_name) const;

    /**
     * @brief Find a specific marker by name
     * @param marker_name Name of the marker to find
     * @return Pointer to marker if found, nullptr otherwise
     */
    const vicon_msgs::msg::Marker* findMarker(const std::string& marker_name) const;

    // === Constants ===
    static constexpr double MAX_VELOCITY = 3.0;        ///< Maximum allowed velocity (m/s)
    static constexpr double VELOCITY_FILTER_ALPHA = 0.4; ///< Low-pass filter coefficient for velocity
    static constexpr double MIN_DT = 0.001;            ///< Minimum time interval for velocity calculation (s)
    static constexpr double MAX_DT = 0.05;             ///< Maximum time interval before considering frame drop (s)
    static constexpr double HANDKERCHIEF_OFFSET_X = 0.02; ///< Handkerchief position offset in X (m)
    static constexpr double HANDKERCHIEF_OFFSET_Y = 0.15; ///< Handkerchief position offset in Y (m)
    static constexpr double HANDKERCHIEF_OFFSET_Z = 0.0;  ///< Handkerchief position offset in Z (m)
    static constexpr double PIPER_ROOT_Z_OFFSET = 0.009;  ///< Piper_root Z-axis offset (m)
    static constexpr double END_EFFECTOR_Z_OFFSET = 0.17; ///< End effector Z-axis offset (m)
    static constexpr double TIMER_PERIOD_MS = 20;         ///< Timer period in milliseconds (50Hz)
};

#endif // COORDINATE_TEST_HPP
