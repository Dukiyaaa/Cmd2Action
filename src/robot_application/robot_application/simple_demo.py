#!/usr/bin/env python3
"""
最简单的 SCARA 机器人控制 Demo
目的: 学习如何通过 ROS2 action 控制关节
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState  # 用于接收关节状态


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


def main():
    rclpy.init()
    controller = SimpleController()
    
    print("\n=== 最简单的 SCARA 控制 Demo ===\n")
    
    # Demo 1: 移动手臂到位置1
    print("Demo 1: 移动手臂到位置1 (pi / 2, pi / 2, -0.0)")
    future = controller.move_arm_simple(3.14 / 2, 3.14 / 2, -0.0, sec=3)
    # future = controller.move_arm_simple(0.0, 0.0, -1.2)
    rclpy.spin_until_future_complete(controller, future, timeout_sec=5)
    
    # 等待2秒
    import time
    time.sleep(5)
    print("完成!\n")
    
    # Demo end:回来
    print("Demo end: 移动手臂回到初始位置 (0.0, 0.0, -0.0)")
    future = controller.move_arm_simple(0.0, 0.0, 0.0, sec=3)
    rclpy.spin_until_future_complete(controller, future, timeout_sec=5)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
