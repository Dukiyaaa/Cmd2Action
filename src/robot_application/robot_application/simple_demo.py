#!/usr/bin/env python3
"""
最简单的 SCARA 机器人控制 Demo
目的: 学习如何通过 ROS2 action 控制关节
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState  # 用于接收关节状态
from robot_application.gazebo_box_display import BoxSpawner
import time

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        # 创建 action 客户端,用于控制手臂
        # action 名称: /scara_arm_controller/follow_joint_trajectory
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory,  # action 类型
            '/scara_arm_controller/follow_joint_trajectory'
        )
        
        # Gazebo 方块生成工具
        self.box = BoxSpawner(self)

        # 创建夹爪控制客户端
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scara_gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('控制器已初始化')
    
    def move_arm_simple(self, pos1, pos2, pos3, sec):
        """
        简单的手臂运动函数
        
        参数说明:
            pos1: rotation1 关节位置 (单位: 弧度 rad)
            pos2: rotation2 关节位置 (单位: 弧度 rad)
            pos3: gripper_joint 关节位置 (单位: 米 m)
        """
        
        # 1. 创建一个轨迹消息
        goal_msg = FollowJointTrajectory.Goal()
        
        # 2. 指定要控制的关节名称 这里指定三个关节，所以后续point.positions需要有三个值
        goal_msg.trajectory.joint_names = ['rotation1', 'rotation2', 'gripper_joint']
        
        # 3. 创建轨迹点 (定义目标位置)
        point = JointTrajectoryPoint()
        point.positions = [pos1, pos2, pos3]  # 三个关节的目标位置
        point.time_from_start = Duration(sec=sec)  
        
        # 4. 将轨迹点添加到轨迹消息
        goal_msg.trajectory.points = [point]
        
        # 5. 等待 action server 就绪
        self.arm_client.wait_for_server()
        
        # 6. 发送目标 异步发送
        print(f"发送命令: pos1={pos1}, pos2={pos2}, pos3={pos3}")
        return self.arm_client.send_goal_async(goal_msg)

    
    def control_gripper(self, finger1, finger2, finger3, finger4, sec=1):
        """
        控制夹爪
        
        参数说明:
            finger1: finger1_joint 位置 (0.0 ~ 0.02)
            finger2: finger2_joint 位置 (-0.02 ~ 0.0)
            finger3: finger3_joint 位置 (-0.02 ~ 0.0)
            finger4: finger4_joint 位置 (0.0 ~ 0.02)
            sec: 运动时间
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'finger1_joint', 'finger2_joint', 
            'finger3_joint', 'finger4_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [finger1, finger2, finger3, finger4]
        point.time_from_start = Duration(sec=sec)
        
        goal_msg.trajectory.points = [point]
        
        self.gripper_client.wait_for_server()
        print(f"发送夹爪命令: [{finger1:.3f}, {finger2:.3f}, {finger3:.3f}, {finger4:.3f}]")
        return self.gripper_client.send_goal_async(goal_msg)
    
    def open_gripper(self, sec=1):
        """打开夹爪"""
        print("打开夹爪...")
        return self.control_gripper(-0.02, 0.02, 0.02, -0.02, sec)
    
    def close_gripper(self, sec=1):
        """关闭夹爪 - 增大闭合量以产生足够夹持力"""
        print("关闭夹爪...")
        return self.control_gripper(0.02, -0.02, -0.02, 0.02, sec)

    def world_init(self):
        """初始化世界: 生成方块等"""
        future = self.move_arm_simple(np.pi/2, 0.0, 0.0, sec=3)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        time.sleep(5)
        print("完成!\n")
        future = self.box.spawn_box(
        name='test_box',
        x=1.8, y=0.0, z=0.05,
        yaw=0.0,
        sx=0.033, sy=0.033, sz=0.033,
        color_rgba=(0.2, 0.6, 0.9, 1.0),
        mass=0.01,
        reference_frame='world'
        )
        # 等待方块生成完成，再打开夹爪
        if future:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        g = self.open_gripper(sec=2)
        rclpy.spin_until_future_complete(self, g, timeout_sec=5)

        time.sleep(2)
        print("完成!\n")

def main():
    rclpy.init()
    controller = SimpleController()
    
    controller.world_init()

    # Demo 1: 移动手臂到抓取位置
    print("Demo 1: 移动手臂到抓取位置 (下降到方块高度)")
    future = controller.move_arm_simple(np.pi/2, 0.0, -0.255, sec=3)
    rclpy.spin_until_future_complete(controller, future, timeout_sec=5)
    
    print("到达抓取位置，等待稳定...")
    time.sleep(3)  # 减少不必要的等待
    print("位置稳定!\n")
    
    # 关闭夹爪并等待动作完成
    print("准备夹取方块...")
    g = controller.close_gripper(sec=3)  # 延长到 3 秒
    rclpy.spin_until_future_complete(controller, g, timeout_sec=6)
    
    print("等待物理引擎稳定接触...")
    time.sleep(3)  # 给足够时间让夹爪施加压力
    print("夹取完成!\n")

    # # 回到初始姿态（抬起方块）
    # print("抬起方块...")
    # future = controller.move_arm_simple(np.pi/2, 0.0, -0.20, sec=4)  # 慢速抬起
    # rclpy.spin_until_future_complete(controller, future, timeout_sec=6)
    # print("抬起完成！")
    time.sleep(3000)  # 给足够时间让夹爪施加压力
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
