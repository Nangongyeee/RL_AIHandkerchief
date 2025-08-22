#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
使用MIT模式的简化投掷控制器
直接通过ROS消息实现高力度投掷动作
"""

import rclpy
from rclpy.node import Node
from piper_msgs.msg import JointMitCmd
from sensor_msgs.msg import JointState
import time


class SimpleMITThrower(Node):
    """简化MIT投掷控制器"""
    
    def __init__(self):
        super().__init__('simple_mit_thrower')
        
        # 发布关节控制命令（标准模式）
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 发布MIT模式控制命令
        self.mit_cmd_pub = self.create_publisher(JointMitCmd, 'joint_mit_cmd', 10)
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states_single', self.joint_callback, 10
        )
        
        self.current_joints = None
        self.get_logger().info("MIT力控投掷控制器启动")
        self.get_logger().info("等待关节状态...")
        
        # 关键位姿定义
        self.initial_joints = [0.0, 0.3, -2.5, 0.0, 1.0, 0.0]  # 初始位姿
        self.prepare_joints = [0.0, 0.3, -2.8, 0.0, 1.3, 0.0]  # 蓄力位姿（向后拉）
        self.throw_joints = [0.0, 0.8, -1.8, 0.0, 0.5, 0.0]    # 投掷位姿（向前上方）
        
        # MIT力控参数
        self.mit_force_multiplier = 4.0  # 高力度倍数
        self.mit_speed = 80              # 高速度
        
        # 等待3秒获取初始状态后开始
        self.start_timer = self.create_timer(3.0, self.start_throwing_sequence)
    
    def joint_callback(self, msg):
        """获取当前关节状态"""
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])  # 只取前6个关节
            # 实时显示关节状态用于调试
            if hasattr(self, 'last_print_time'):
                current_time = time.time()
                if current_time - self.last_print_time > 1.0:  # 每秒打印一次
                    joint_degs = [j * 57.2958 for j in self.current_joints]  # 转换为度
                    self.get_logger().info(f"当前关节角度(度): [{joint_degs[0]:.1f}, {joint_degs[1]:.1f}, {joint_degs[2]:.1f}, {joint_degs[3]:.1f}, {joint_degs[4]:.1f}, {joint_degs[5]:.1f}]")
                    self.last_print_time = current_time
            else:
                self.last_print_time = time.time()
    
    def start_throwing_sequence(self):
        """开始投掷序列"""
        self.start_timer.cancel()
        
        if self.current_joints is None:
            self.get_logger().error("未获取到机械臂状态，无法开始投掷")
            return
            
        self.get_logger().info("开始MIT力控投掷序列")
        self.get_logger().info(f"初始位姿: {self.initial_joints}")
        
        # 步骤1: 移动到初始位置
        self.move_to_initial_position()
        
        # 步骤2: 2秒后移动到蓄力位置
        self.prepare_timer = self.create_timer(2.0, self.move_to_prepare_position)
    
    def move_to_initial_position(self):
        """移动到初始位置"""
        self.get_logger().info("移动到初始位置")
        self.publish_joint_command(self.initial_joints, velocity=30, use_mit=False)
    
    def move_to_prepare_position(self):
        """移动到蓄力位置"""
        self.prepare_timer.cancel()
        self.get_logger().info("移动到蓄力位置（后拉准备）")
        self.publish_joint_command(self.prepare_joints, velocity=40, use_mit=False)
        
        # 1.5秒后执行投掷
        self.throw_timer = self.create_timer(1.5, self.execute_mit_throw)
    
    def execute_mit_throw(self):
        """执行MIT模式高力度投掷"""
        self.throw_timer.cancel()
        self.get_logger().info(f"执行MIT高力投掷！力度倍数: {self.mit_force_multiplier}")
        
        # 使用MIT模式高力高速投掷
        self.publish_joint_command(
            self.throw_joints, 
            velocity=self.mit_speed, 
            use_mit=True
        )
        
        # 2秒后返回初始位置
        self.return_timer = self.create_timer(2.0, self.return_to_initial)
    
    def return_to_initial(self):
        """返回初始位置"""
        self.return_timer.cancel()
        self.get_logger().info("返回初始位置")
        self.publish_joint_command(self.initial_joints, velocity=40, use_mit=False)
        
        # 完成投掷序列
        self.complete_timer = self.create_timer(3.0, self.complete_sequence)
    
    def publish_joint_command(self, joint_positions, velocity=30, use_mit=False):
        """发布关节控制命令"""
        if use_mit:
            # MIT模式 - 发送单独的MIT控制命令到每个关节
            self.get_logger().info("使用MIT模式发送高力控制指令")
            
            # 记录执行前的关节状态
            if self.current_joints:
                before_degs = [j * 57.2958 for j in self.current_joints]
                self.get_logger().info(f"MIT执行前关节角度(度): [{before_degs[0]:.1f}, {before_degs[1]:.1f}, {before_degs[2]:.1f}, {before_degs[3]:.1f}, {before_degs[4]:.1f}, {before_degs[5]:.1f}]")
            
            # MIT控制参数
            mit_kp_values = [100.0, 120.0, 100.0, 80.0, 60.0, 40.0]  # 每个关节的刚度
            mit_kd_values = [5.0, 6.0, 5.0, 4.0, 3.0, 2.0]          # 每个关节的阻尼
            mit_torques = [25.0, 40.0, 35.0, 20.0, 15.0, 10.0]      # 每个关节的力矩
            
            # 为每个关节发送MIT控制命令
            for i in range(6):
                mit_cmd = JointMitCmd()
                mit_cmd.motor_num = i + 1
                mit_cmd.pos_ref = joint_positions[i]  # 目标位置（弧度）
                mit_cmd.vel_ref = 0.0  # 目标速度
                mit_cmd.kp = mit_kp_values[i]
                mit_cmd.kd = mit_kd_values[i] 
                mit_cmd.t_ref = mit_torques[i]  # 关键：高力矩输出
                
                self.mit_cmd_pub.publish(mit_cmd)
                self.get_logger().info(f"MIT控制关节{i+1}: pos={mit_cmd.pos_ref:.3f}, torque={mit_cmd.t_ref:.1f}")
                time.sleep(0.1)  # 短暂延迟确保命令发送
                
            # 等待一段时间然后检查结果
            self.get_logger().info("等待MIT控制执行...")
            time.sleep(3.0)
            
            # 记录执行后的关节状态
            if self.current_joints:
                after_degs = [j * 57.2958 for j in self.current_joints]
                self.get_logger().info(f"MIT执行后关节角度(度): [{after_degs[0]:.1f}, {after_degs[1]:.1f}, {after_degs[2]:.1f}, {after_degs[3]:.1f}, {after_degs[4]:.1f}, {after_degs[5]:.1f}]")
                
                # 计算运动差异
                if hasattr(self, 'current_joints'):
                    target_degs = [j * 57.2958 for j in joint_positions]
                    self.get_logger().info(f"MIT目标角度(度): [{target_degs[0]:.1f}, {target_degs[1]:.1f}, {target_degs[2]:.1f}, {target_degs[3]:.1f}, {target_degs[4]:.1f}, {target_degs[5]:.1f}]")
            
        else:
            # 标准模式 - 使用JointState控制
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            
            # 关节位置
            joint_cmd.position = joint_positions
            
            # 速度控制
            joint_cmd.velocity = [float(velocity)] * 6
            joint_cmd.effort = [0.0] * 6  # 标准模式不使用力控
            
            self.joint_cmd_pub.publish(joint_cmd)
    
    def complete_sequence(self):
        """完成投掷序列"""
        self.complete_timer.cancel()
        self.get_logger().info("MIT力控投掷序列完成!")
        self.get_logger().info("投掷动作已完成，如果感觉到明显的力度增强，说明MIT模式工作正常")
        self.get_logger().info("可以通过调整mit_force_multiplier和effort值来优化投掷力度")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        thrower = SimpleMITThrower()
        rclpy.spin(thrower)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
