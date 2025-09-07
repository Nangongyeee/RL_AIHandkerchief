#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CoordinateTestNode : public rclcpp::Node {
public:
    CoordinateTestNode()
    : Node("coordinate_test_node")
    {
        root_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/root0907/root0907", 10,
            std::bind(&CoordinateTestNode::rootCallback, this, std::placeholders::_1)
        );
        handkerchief_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/cloth090602/cloth090602", 10,
            std::bind(&CoordinateTestNode::handkerchiefCallback, this, std::placeholders::_1)
        );
        
        // 订阅机械臂末端位姿
        end_effector_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/end_pose", 10,
            std::bind(&CoordinateTestNode::endEffectorCallback, this, std::placeholders::_1)
        );

        // 去掉重复的 pos 和 vel 发布，只保留更完整的消息格式
        vel_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("handkerchief_velocity_twist", 10);
        vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("handkerchief_velocity_marker", 10);
        root_new_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Piper_root", 10);
        handkerchief_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("handkerchief_piperroot", 10);
        
        // 机械臂末端位姿发布者和坐标系可视化
        end_effector_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_piperroot", 10);
        end_effector_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("end_effector_coordinate_marker", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CoordinateTestNode::publishData, this)
        );

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    geometry_msgs::msg::PoseStamped::SharedPtr root_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr handkerchief_pose_;
    geometry_msgs::msg::Pose::SharedPtr end_effector_pose_; // 机械臂末端位姿
    std::vector<double> prev_relative_pos_{0.0, 0.0, 0.0};
    std::vector<double> prev_velocity_{0.0, 0.0, 0.0}; // 用于速度滤波
    rclcpp::Time prev_time_;
    bool first_callback_ = true; // 第一次回调标志

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr root_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr end_effector_sub_; // 机械臂末端订阅者
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_twist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr root_new_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pub_; // 机械臂末端发布者
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr end_effector_marker_pub_; // 末端坐标系可视化
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void rootCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        root_pose_ = msg;
    }

    void handkerchiefCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handkerchief_pose_ = msg;
    }
    
    void endEffectorCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        end_effector_pose_ = msg;
        
        // 添加调试信息
        RCLCPP_INFO(this->get_logger(), "End effector callback received: pos=[%.3f, %.3f, %.3f], quat=[%.3f, %.3f, %.3f, %.3f]",
            msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    }

    void publishData() {
        if (!root_pose_ || !handkerchief_pose_) return;

        // 1. 获取原 root0907 的位置和姿态
        Eigen::Vector3d root_position(
            root_pose_->pose.position.x, 
            root_pose_->pose.position.y, 
            root_pose_->pose.position.z
        );
        // 获取原 root0907 的四元数姿态
        Eigen::Quaterniond root_quat(
            root_pose_->pose.orientation.w,
            root_pose_->pose.orientation.x,
            root_pose_->pose.orientation.y,
            root_pose_->pose.orientation.z
        );
        
        // 2. 构造旋转变换矩阵：先绕 z 轴逆时针转 90°，再绕 x 轴逆时针转 90°
        Eigen::AngleAxisd rot_z(-M_PI/2, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rot_x(-M_PI/2, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond additional_rotation = rot_z * rot_x;
        // 计算 Piper_root 坐标系的姿态四元数：原姿态 * 附加旋转
        Eigen::Quaterniond piper_root_quat = root_quat * additional_rotation;
        Eigen::Matrix3d R = piper_root_quat.toRotationMatrix();

        // 3. 计算 Piper_root 坐标系的位置：沿新坐标系 -z 轴平移 9mm
        // 新坐标系的 -z 轴在世界坐标系中的方向向量
        Eigen::Vector3d new_z_axis = R.col(2);  // 新坐标系的 z 轴
        Eigen::Vector3d piper_root_position = root_position - 0.009 * new_z_axis;  // 沿 -z 轴平移

        // 4. 发布 Piper_root 的 PoseStamped 消息
        geometry_msgs::msg::PoseStamped piper_root_pose;
        piper_root_pose.header.stamp = this->now();
        piper_root_pose.header.frame_id = "vicon/World";
        piper_root_pose.pose.position.x = piper_root_position.x();
        piper_root_pose.pose.position.y = piper_root_position.y();
        piper_root_pose.pose.position.z = piper_root_position.z();
        piper_root_pose.pose.orientation.x = piper_root_quat.x();
        piper_root_pose.pose.orientation.y = piper_root_quat.y();
        piper_root_pose.pose.orientation.z = piper_root_quat.z();
        piper_root_pose.pose.orientation.w = piper_root_quat.w();
        root_new_pub_->publish(piper_root_pose);

        // 通过 TF 发布 Piper_root 坐标系，在 Foxglove 中可视化
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "vicon/World";
        tf_msg.child_frame_id = "Piper_root";
        tf_msg.transform.translation.x = piper_root_position.x();
        tf_msg.transform.translation.y = piper_root_position.y();
        tf_msg.transform.translation.z = piper_root_position.z();
        tf_msg.transform.rotation.x = piper_root_quat.x();
        tf_msg.transform.rotation.y = piper_root_quat.y();
        tf_msg.transform.rotation.z = piper_root_quat.z();
        tf_msg.transform.rotation.w = piper_root_quat.w();
        tf_broadcaster_->sendTransform(tf_msg);

        // 5. 计算手绢在 Piper_root 坐标系下的位置和姿态
        // 获取手绢在世界坐标系下的位置、四元数
        Eigen::Vector3d handkerchief_world(
            handkerchief_pose_->pose.position.x, 
            handkerchief_pose_->pose.position.y, 
            handkerchief_pose_->pose.position.z
        );
        Eigen::Quaterniond handkerchief_quat(
            handkerchief_pose_->pose.orientation.w,
            handkerchief_pose_->pose.orientation.x,
            handkerchief_pose_->pose.orientation.y,
            handkerchief_pose_->pose.orientation.z
        );
        // 手绢在 Piper_root 坐标系下的位置
        Eigen::Vector3d handkerchief_in_piper = R.transpose() * (handkerchief_world - piper_root_position);
        
        // 手绢在 Piper_root 坐标系下的姿态（相对于 Piper_root 的姿态变换）
        Eigen::Quaterniond handkerchief_quat_in_piper = piper_root_quat.inverse() * handkerchief_quat;

        // 6. 发布 handkerchief_piperroot PoseStamped 消息
        geometry_msgs::msg::PoseStamped handkerchief_piperroot;
        handkerchief_piperroot.header.stamp = this->now();
        handkerchief_piperroot.header.frame_id = "Piper_root";
        handkerchief_piperroot.pose.position.x = handkerchief_in_piper.x();
        handkerchief_piperroot.pose.position.y = handkerchief_in_piper.y();
        handkerchief_piperroot.pose.position.z = handkerchief_in_piper.z();
        handkerchief_piperroot.pose.orientation.x = handkerchief_quat_in_piper.x();
        handkerchief_piperroot.pose.orientation.y = handkerchief_quat_in_piper.y();
        handkerchief_piperroot.pose.orientation.z = handkerchief_quat_in_piper.z();
        handkerchief_piperroot.pose.orientation.w = handkerchief_quat_in_piper.w();
        handkerchief_pub_->publish(handkerchief_piperroot);
        // 以 TF 形式发布 handkerchief_piperroot
        geometry_msgs::msg::TransformStamped handkerchief_tf;
        handkerchief_tf.header.stamp = this->now();
        handkerchief_tf.header.frame_id = "Piper_root";
        handkerchief_tf.child_frame_id = "Handkerchief_piperroot";
        handkerchief_tf.transform.translation.x = handkerchief_in_piper.x();
        handkerchief_tf.transform.translation.y = handkerchief_in_piper.y();
        handkerchief_tf.transform.translation.z = handkerchief_in_piper.z();
        handkerchief_tf.transform.rotation.x = handkerchief_quat_in_piper.x();
        handkerchief_tf.transform.rotation.y = handkerchief_quat_in_piper.y();
        handkerchief_tf.transform.rotation.z = handkerchief_quat_in_piper.z();
        handkerchief_tf.transform.rotation.w = handkerchief_quat_in_piper.w();
        tf_broadcaster_->sendTransform(handkerchief_tf);

        // 7. 计算手绢在 Piper_root 坐标系下的速度（带丢帧处理）
        rclcpp::Time now = this->now();
        double dt = prev_time_.nanoseconds() > 0 ? (now - prev_time_).seconds() : 0.0;
        
        Eigen::Vector3d velocity_vec(0, 0, 0);
        bool velocity_valid = false;
        
        if (!first_callback_ && dt > 0.001 && dt < 0.05) { // dt在1ms到50ms之间才计算速度（100Hz对应10ms，丢2-3帧为30ms左右）
            // 计算原始速度
            Eigen::Vector3d raw_velocity;
            raw_velocity.x() = (handkerchief_in_piper.x() - prev_relative_pos_[0]) / dt;
            raw_velocity.y() = (handkerchief_in_piper.y() - prev_relative_pos_[1]) / dt;
            raw_velocity.z() = (handkerchief_in_piper.z() - prev_relative_pos_[2]) / dt;
            
            // 速度限幅：限制最大速度为 3 m/s（针对100Hz数据调整）
            double max_velocity = 3.0;
            if (raw_velocity.norm() < max_velocity) {
                // 低通滤波：针对100Hz数据优化滤波系数
                double alpha = 0.4; // 滤波系数，针对高频数据适当增加响应速度
                velocity_vec.x() = alpha * raw_velocity.x() + (1 - alpha) * prev_velocity_[0];
                velocity_vec.y() = alpha * raw_velocity.y() + (1 - alpha) * prev_velocity_[1];
                velocity_vec.z() = alpha * raw_velocity.z() + (1 - alpha) * prev_velocity_[2];
                velocity_valid = true;
                
                // 更新上一次速度
                prev_velocity_[0] = velocity_vec.x();
                prev_velocity_[1] = velocity_vec.y();
                prev_velocity_[2] = velocity_vec.z();
            } else {
                // 速度异常大，使用上一次的速度值并逐渐衰减
                velocity_vec.x() = prev_velocity_[0] * 0.9;
                velocity_vec.y() = prev_velocity_[1] * 0.9;
                velocity_vec.z() = prev_velocity_[2] * 0.9;
                velocity_valid = true;
                
                // 更新上一次速度为衰减后的值
                prev_velocity_[0] = velocity_vec.x();
                prev_velocity_[1] = velocity_vec.y();
                prev_velocity_[2] = velocity_vec.z();
                
                RCLCPP_WARN(this->get_logger(), "速度异常: %.3f m/s, 使用衰减速度", raw_velocity.norm());
            }
        } else if (dt >= 0.05) {
            // 时间间隔过长（可能丢帧），逐渐衰减速度至零
            velocity_vec.x() = prev_velocity_[0] * 0.5;
            velocity_vec.y() = prev_velocity_[1] * 0.5;
            velocity_vec.z() = prev_velocity_[2] * 0.5;
            velocity_valid = true;
            
            prev_velocity_[0] = velocity_vec.x();
            prev_velocity_[1] = velocity_vec.y();
            prev_velocity_[2] = velocity_vec.z();
            
            RCLCPP_WARN(this->get_logger(), "检测到丢帧: dt=%.3f s (%.1f帧), 衰减速度", dt, dt * 100.0);
        }

        // 8. 发布 TwistStamped 速度消息（只有在速度有效时发布）
        if (velocity_valid) {
            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = now;
            twist_msg.header.frame_id = "Piper_root";
            twist_msg.twist.linear.x = velocity_vec.x();
            twist_msg.twist.linear.y = velocity_vec.y();
            twist_msg.twist.linear.z = velocity_vec.z();
            twist_msg.twist.angular.x = 0.0;
            twist_msg.twist.angular.y = 0.0;
            twist_msg.twist.angular.z = 0.0;
            vel_twist_pub_->publish(twist_msg);
        }

        // 9. 发布速度箭头可视化标记（修复可视化问题）
        double vel_magnitude = velocity_vec.norm();
        if (velocity_valid && vel_magnitude > 0.01) { // 只有当速度有效且足够大时才显示箭头
            visualization_msgs::msg::Marker velocity_marker;
            velocity_marker.header.stamp = now;
            velocity_marker.header.frame_id = "Piper_root";
            velocity_marker.ns = "handkerchief_velocity";
            velocity_marker.id = 0;
            velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
            velocity_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 使用 points 方式定义箭头的起点和终点
            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = handkerchief_in_piper.x();
            start_point.y = handkerchief_in_piper.y();
            start_point.z = handkerchief_in_piper.z();
            
            // 箭头终点：起点 + 缩放后的速度向量
            double scale = 0.2; // 箭头长度缩放因子
            end_point.x = start_point.x + velocity_vec.x() * scale;
            end_point.y = start_point.y + velocity_vec.y() * scale;
            end_point.z = start_point.z + velocity_vec.z() * scale;
            
            velocity_marker.points.push_back(start_point);
            velocity_marker.points.push_back(end_point);
            
            // 箭头尺寸
            velocity_marker.scale.x = 0.01; // 箭头轴直径
            velocity_marker.scale.y = 0.02; // 箭头头部直径
            velocity_marker.scale.z = 0.0;  // 未使用
            
            // 颜色：根据速度大小设置
            velocity_marker.color.a = 1.0;
            velocity_marker.color.r = std::min(vel_magnitude, 1.0);
            velocity_marker.color.g = std::max(1.0 - vel_magnitude, 0.0);
            velocity_marker.color.b = 0.0;
            
            velocity_marker.lifetime = rclcpp::Duration::from_nanoseconds(200000000); // 200ms
            vel_marker_pub_->publish(velocity_marker);
        }

        // 更新上一次的位置和时间
        prev_relative_pos_[0] = handkerchief_in_piper.x();
        prev_relative_pos_[1] = handkerchief_in_piper.y();
        prev_relative_pos_[2] = handkerchief_in_piper.z();
        prev_time_ = now;
        first_callback_ = false; // 标记已经不是第一次回调
        
        // 8. 处理机械臂末端位姿（如果有数据）
        if (end_effector_pose_) {
            RCLCPP_DEBUG(this->get_logger(), "Processing end effector pose data");
            
            // /end_pose 话题发布的数据已经是在机械臂基座坐标系下的位姿
            // 直接使用这些数据作为在 Piper_root 坐标系下的位置和姿态
            Eigen::Vector3d end_effector_in_piper(
                end_effector_pose_->position.x,
                end_effector_pose_->position.y,
                end_effector_pose_->position.z
            );
            Eigen::Quaterniond end_effector_quat_in_piper(
                end_effector_pose_->orientation.w,
                end_effector_pose_->orientation.x,
                end_effector_pose_->orientation.y,
                end_effector_pose_->orientation.z
            );
            
            // 发布机械臂末端在 Piper_root 坐标系下的位姿
            geometry_msgs::msg::PoseStamped end_effector_piperroot;
            end_effector_piperroot.header.stamp = this->now();
            end_effector_piperroot.header.frame_id = "Piper_root";
            end_effector_piperroot.pose.position.x = end_effector_in_piper.x();
            end_effector_piperroot.pose.position.y = end_effector_in_piper.y();
            end_effector_piperroot.pose.position.z = end_effector_in_piper.z();
            end_effector_piperroot.pose.orientation.x = end_effector_quat_in_piper.x();
            end_effector_piperroot.pose.orientation.y = end_effector_quat_in_piper.y();
            end_effector_piperroot.pose.orientation.z = end_effector_quat_in_piper.z();
            end_effector_piperroot.pose.orientation.w = end_effector_quat_in_piper.w();
            end_effector_pub_->publish(end_effector_piperroot);
            
            // 发布末端坐标系的 TF
            geometry_msgs::msg::TransformStamped end_effector_tf;
            end_effector_tf.header.stamp = this->now();
            end_effector_tf.header.frame_id = "Piper_root";
            end_effector_tf.child_frame_id = "EndEffector_piperroot";
            end_effector_tf.transform.translation.x = end_effector_in_piper.x();
            end_effector_tf.transform.translation.y = end_effector_in_piper.y();
            end_effector_tf.transform.translation.z = end_effector_in_piper.z();
            end_effector_tf.transform.rotation.x = end_effector_quat_in_piper.x();
            end_effector_tf.transform.rotation.y = end_effector_quat_in_piper.y();
            end_effector_tf.transform.rotation.z = end_effector_quat_in_piper.z();
            end_effector_tf.transform.rotation.w = end_effector_quat_in_piper.w();
            tf_broadcaster_->sendTransform(end_effector_tf);
            
            // 创建末端坐标系可视化标记（坐标轴）
            publishEndEffectorCoordinateMarker(end_effector_in_piper, end_effector_quat_in_piper);
        } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No end effector pose data available");
        }
    }
    
    void publishEndEffectorCoordinateMarker(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        // 发布坐标轴可视化（X轴红色，Y轴绿色，Z轴蓝色）
        
        // X轴（红色）
        visualization_msgs::msg::Marker x_axis;
        x_axis.header.frame_id = "Piper_root";
        x_axis.header.stamp = this->now();
        x_axis.ns = "end_effector_coordinate";
        x_axis.id = 0;
        x_axis.type = visualization_msgs::msg::Marker::ARROW;
        x_axis.action = visualization_msgs::msg::Marker::ADD;
        
        // X轴起点
        geometry_msgs::msg::Point start_x;
        start_x.x = position.x();
        start_x.y = position.y();
        start_x.z = position.z();
        
        // X轴终点（沿X轴方向0.05m）
        Eigen::Vector3d x_direction = orientation.toRotationMatrix().col(0);
        geometry_msgs::msg::Point end_x;
        end_x.x = position.x() + 0.05 * x_direction.x();
        end_x.y = position.y() + 0.05 * x_direction.y();
        end_x.z = position.z() + 0.05 * x_direction.z();
        
        x_axis.points.push_back(start_x);
        x_axis.points.push_back(end_x);
        x_axis.scale.x = 0.005; // 箭头轴直径
        x_axis.scale.y = 0.01;  // 箭头头部直径
        x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0; x_axis.color.a = 1.0;
        x_axis.lifetime = rclcpp::Duration::from_nanoseconds(100000000); // 100ms
        end_effector_marker_pub_->publish(x_axis);
        
        // Y轴（绿色）
        visualization_msgs::msg::Marker y_axis = x_axis;
        y_axis.id = 1;
        Eigen::Vector3d y_direction = orientation.toRotationMatrix().col(1);
        geometry_msgs::msg::Point end_y;
        end_y.x = position.x() + 0.05 * y_direction.x();
        end_y.y = position.y() + 0.05 * y_direction.y();
        end_y.z = position.z() + 0.05 * y_direction.z();
        y_axis.points.clear();
        y_axis.points.push_back(start_x);
        y_axis.points.push_back(end_y);
        y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0;
        end_effector_marker_pub_->publish(y_axis);
        
        // Z轴（蓝色）
        visualization_msgs::msg::Marker z_axis = x_axis;
        z_axis.id = 2;
        Eigen::Vector3d z_direction = orientation.toRotationMatrix().col(2);
        geometry_msgs::msg::Point end_z;
        end_z.x = position.x() + 0.05 * z_direction.x();
        end_z.y = position.y() + 0.05 * z_direction.y();
        end_z.z = position.z() + 0.05 * z_direction.z();
        z_axis.points.clear();
        z_axis.points.push_back(start_x);
        z_axis.points.push_back(end_z);
        z_axis.color.r = 0.0; z_axis.color.g = 0.0; z_axis.color.b = 1.0;
        end_effector_marker_pub_->publish(z_axis);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTestNode>());
    rclcpp::shutdown();
    return 0;
}