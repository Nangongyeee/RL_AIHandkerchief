#ifndef PIPER_RL_SAFE_CONTROLLER_HPP
#define PIPER_RL_SAFE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <vector>

#include "piper_rl_deploy/model_loader.hpp"

namespace piper_rl_deploy {

/**
 * @brief PiperRLSafeController class for simplified RL control
 * 
 * This is a simplified version of PiperRLController that directly subscribes
 * to coordinate_test published messages without any coordinate transformations.
 * It directly uses the transformed data for model inference and joint control.
 */
class PiperRLSafeController : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS2 node
     */
    explicit PiperRLSafeController(const std::string& node_name);

    /**
     * @brief Destructor
     */
    ~PiperRLSafeController();

private:
    // === Model and Configuration ===
    std::unique_ptr<ModelLoader> model_loader_;
    ModelType model_type_;
    std::string model_path_;
    bool model_ready_;
    bool robot_ready_;
    bool emergency_stop_;
    
    // Dimensions
    size_t obs_dim_;
    size_t action_dim_;
    
    // Control parameters
    double control_frequency_;
    double inference_frequency_;
    
    // Joint configuration
    std::vector<std::string> joint_names_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> action_scale_;
    std::vector<double> joint_pos_offset_;
    
    // === Current State ===
    struct ObservationData {
        std::vector<float> joint_positions;     // Current joint angles
        std::vector<float> joint_velocities;    // Current joint velocities
        std::vector<float> stick_tip_position;  // End effector position (from coordinate_test)
        std::vector<float> handkerchief_position; // Handkerchief position in Piper_root frame
        std::vector<float> handkerchief_velocity; // Handkerchief velocity in Piper_root frame
    } current_obs_;
    
    // === ROS2 Interface ===
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr handkerchief_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr inference_timer_;
    
    // === Callback Functions ===
    
    /**
     * @brief Joint state callback - receives current joint angles and velocities
     * @param msg Joint state message from /joint_states_single
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    /**
     * @brief Handkerchief pose callback - receives handkerchief position in Piper_root frame
     * @param msg Transformed handkerchief pose from coordinate_test (handkerchief_piperroot topic)
     */
    void handkerchiefPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Handkerchief velocity callback - receives handkerchief velocity in Piper_root frame
     * @param msg Handkerchief velocity from coordinate_test (handkerchief_velocity topic)
     */
    void handkerchiefVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    
    /**
     * @brief End effector pose callback - receives end effector position in Piper_root frame
     * @param msg End effector pose from coordinate_test (end_effector_piperroot topic)
     */
    void endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // === Main Processing Functions ===
    
    /**
     * @brief Main control loop
     */
    void controlLoop();
    
    /**
     * @brief Main inference loop
     */
    void inferenceLoop();
    
    /**
     * @brief Compute observation vector for model inference
     * @return Observation vector for the RL model
     */
    std::vector<float> computeObservation();
    
    /**
     * @brief Process raw model actions
     * @param raw_actions Raw actions from model
     * @return Processed actions ready for joint control
     */
    std::vector<float> processActions(const std::vector<float>& raw_actions);
    
    /**
     * @brief Publish joint commands
     * @param actions Processed joint position commands
     */
    void publishJointCommands(const std::vector<float>& actions);
    
    /**
     * @brief Publish status information
     */
    void publishStatus();
    
    // === Utility Functions ===
    
    /**
     * @brief Load parameters from ROS2 parameter server
     */
    void loadParameters();
    
    /**
     * @brief Initialize the RL model
     */
    void initializeModel();
    
    /**
     * @brief Initialize robot state
     */
    void initializeRobot();
    
    /**
     * @brief Safety check for actions
     * @param actions Actions to check
     * @return True if actions are safe
     */
    bool safetyCheck(const std::vector<float>& actions);
    
    /**
     * @brief Emergency stop function
     */
    void emergencyStop();
    
    /**
     * @brief Clip actions to safe range
     * @param actions Input actions
     * @return Clipped actions
     */
    std::vector<float> clipActions(const std::vector<float>& actions);
};

} // namespace piper_rl_deploy

#endif // PIPER_RL_SAFE_CONTROLLER_HPP
