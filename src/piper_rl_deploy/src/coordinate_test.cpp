#include "piper_rl_deploy/coordinate_test.hpp"

// CoordinateTestNode implementation

CoordinateTestNode::CoordinateTestNode()
    : Node("coordinate_test_node")
{
    root_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vicon/root0908/root0908", 10,
        std::bind(&CoordinateTestNode::rootCallback, this, std::placeholders::_1)
    );
    handkerchief_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vicon/cloth0908/cloth0908", 10,
        std::bind(&CoordinateTestNode::handkerchiefCallback, this, std::placeholders::_1)
    );
    
    // 订阅机械臂末端位姿
    end_effector_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/end_pose", 10,
        std::bind(&CoordinateTestNode::endEffectorCallback, this, std::placeholders::_1)
    );

    // 去掉重复的 pos 和 vel 发布，只保留更完整的消息格式
    vel_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("handkerchief_velocity", 10);
    root_new_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Piper_root", 10);
    handkerchief_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("handkerchief_piperroot", 10);
    
    // 机械臂末端位姿发布者
    end_effector_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_piperroot", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CoordinateTestNode::publishData, this)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void CoordinateTestNode::rootCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    root_pose_ = msg;
}

void CoordinateTestNode::handkerchiefCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    handkerchief_pose_ = msg;
}

void CoordinateTestNode::endEffectorCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    end_effector_pose_ = msg;
}

void CoordinateTestNode::publishData() {
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

        // 构造旋转变换矩阵：先绕 y 轴逆时针转 90°，再绕 x 轴逆时针转 90° 
        Eigen::AngleAxisd rot_y(M_PI/2, Eigen::Vector3d::UnitY());  // TODO.check
        Eigen::AngleAxisd rot_x(M_PI, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond additional_rotation = rot_y * rot_x;
        // 计算 Piper_root 坐标系的姿态四元数：原姿态 * 附加旋转
        Eigen::Quaterniond piper_root_quat = root_quat * additional_rotation;
        Eigen::Matrix3d R = piper_root_quat.toRotationMatrix();

        // 计算 Piper_root 坐标系的位置：沿新坐标系 -z 轴平移 9mm
        // 新坐标系的 -z 轴在世界坐标系中的方向向量
        Eigen::Vector3d new_z_axis = R.col(2);  // 新坐标系的 z 轴
        Eigen::Vector3d piper_root_position = root_position - 0.009 * new_z_axis;  // 沿 -z 轴平移

        // 发布 Piper_root 的 PoseStamped 消息
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

        // 2. 计算手绢在 Piper_root 坐标系下的位置和姿态
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

        // 位置平移调整（根据需要修改偏移量）
        Eigen::Vector3d offset(0.02, 0.15, 0.0);  // x, y, z 方向的偏移量（单位：米）
        handkerchief_world += offset;

        // 手绢在 Piper_root 坐标系下的位置
        Eigen::Vector3d handkerchief_in_piper = R.transpose() * (handkerchief_world - piper_root_position);
        
        // 手绢在 Piper_root 坐标系下的姿态（相对于 Piper_root 的姿态变换）
        Eigen::Quaterniond handkerchief_quat_in_piper = piper_root_quat.inverse() * handkerchief_quat;

        // 发布 handkerchief_piperroot PoseStamped 消息
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

        // 3. 计算手绢在 Piper_root 坐标系下的速度（带丢帧处理）
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

        // 发布 TwistStamped 速度消息（只有在速度有效时发布）
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

        // 更新上一次的位置和时间
        prev_relative_pos_[0] = handkerchief_in_piper.x();
        prev_relative_pos_[1] = handkerchief_in_piper.y();
        prev_relative_pos_[2] = handkerchief_in_piper.z();
        prev_time_ = now;
        first_callback_ = false; // 标记已经不是第一次回调
        
        // 4. 处理机械臂末端位姿（如果有数据）
        if (end_effector_pose_) {
            // /end_pose 话题发布的数据已经是在机械臂基座坐标系下的位姿
            // 首先获取原始的末端执行器位置和姿态
            Eigen::Vector3d end_effector_base_pos(
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
            
            // 沿着末端执行器自身坐标系的 z 轴移动 17cm
            Eigen::Matrix3d end_effector_rotation = end_effector_quat_in_piper.toRotationMatrix();
            Eigen::Vector3d z_axis_direction = end_effector_rotation.col(2); // 末端执行器的 z 轴方向
            Eigen::Vector3d offset = 0.17 * z_axis_direction; // 17cm 偏移
            Eigen::Vector3d end_effector_in_piper = end_effector_base_pos + offset;
            
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
            

        } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No end effector pose data available");
        }
    }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTestNode>());
    rclcpp::shutdown();
    return 0;
}