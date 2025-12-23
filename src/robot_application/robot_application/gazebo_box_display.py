#!/usr/bin/env python3
"""
Gazebo Box Display Utilities
在 Gazebo 中生成/删除指定尺寸与位置的方块。
用法: 在你的 ROS2 节点中实例化 BoxSpawner(node)，然后调用 spawn_box/delete_entity。
"""

from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import math


class BoxSpawner:
    def __init__(self, node):
        self.node = node
        self.spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = node.create_client(DeleteEntity, '/delete_entity')

    def _yaw_to_quat(self, yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _make_box_sdf(self, name: str, size_xyz, color_rgba=(0.8, 0.2, 0.2, 1.0), mass: float = 0.2) -> str:
        sx, sy, sz = size_xyz
        r, g, b, a = color_rgba
        ixx = (mass / 12.0) * (sy * sy + sz * sz)
        iyy = (mass / 12.0) * (sx * sx + sz * sz)
        izz = (mass / 12.0) * (sx * sx + sy * sy)
        return f"""
            <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="link">
                <inertial>
                    <mass>{mass}</mass>
                    <inertia>
                    <ixx>{ixx}</ixx>
                    <iyy>{iyy}</iyy>
                    <izz>{izz}</izz>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyz>0</iyz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                    <box>
                        <size>{sx} {sy} {sz}</size>
                    </box>
                    </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>20.0</mu>
                                    <mu2>20.0</mu2>
                                </ode>
                            </friction>
                            <contact>
                                <ode>
                                    <kp>100000000.0</kp>
                                    <kd>1.0</kd>
                                </ode>
                            </contact>
                        </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                    <box>
                        <size>{sx} {sy} {sz}</size>
                    </box>
                    </geometry>
                    <material>
                    <ambient>{r} {g} {b} {a}</ambient>
                    <diffuse>{r} {g} {b} {a}</diffuse>
                    </material>
                </visual>
                </link>
            </model>
            </sdf>
            """

    def spawn_box(self, name: str, x: float, y: float, z: float,
                  yaw: float,
                  sx: float, sy: float, sz: float,
                  color_rgba=(0.2, 0.6, 0.9, 1.0),
                  mass: float = 0.2,
                  reference_frame: str = 'world'):
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('等待 /spawn_entity 服务超时，请确认 gazebo_ros 已运行')
            return None

        xml = self._make_box_sdf(name, (sx, sy, sz), color_rgba, mass)
        qx, qy, qz, qw = self._yaw_to_quat(yaw)

        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw
        req.reference_frame = reference_frame

        self.node.get_logger().info(
            f'生成方块: name={name}, pos=({x:.3f},{y:.3f},{z:.3f}), size=({sx:.3f},{sy:.3f},{sz:.3f})')
        return self.spawn_client.call_async(req)

    def delete_entity(self, name: str):
        if not self.delete_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('等待 /delete_entity 服务超时，请确认 gazebo_ros 已运行')
            return None
        req = DeleteEntity.Request()
        req.name = name
        self.node.get_logger().info(f'删除实体: {name}')
        return self.delete_client.call_async(req)
