import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取路径
    robot_name_in_model = 'model'
    urdf_tutorial_path = get_package_share_directory('robot_description')
    default_model_path = urdf_tutorial_path + '/urdf/scara.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/default_world.world'
    # 为launch声明参数
    # 这一步实现的效果:
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    # 等价命令: ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=<urdf_content>
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 通过 PythonLaunchDescriptionSource包含另外一个launch文件
    # 等价命令: ros2 launch gazebo_ros gazebo.launch.py world:=<world_path> verbose:=true
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), 
            '/launch',
            '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    # 请求gazebo加载机器人
    # 等价命令: ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity model
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ]
    )
    # 加载并激活关节状态广播器
    # 等价命令: ros2 run controller_manager spawner joint_state_broadcaster
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 加载并激活手臂控制器
    # 等价命令: ros2 run controller_manager spawner scara_arm_controller
    load_arm_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_arm_controller'],
        output='screen'
    )

    # 加载并激活夹爪控制器
    # 等价命令: ros2 run controller_manager spawner scara_gripper_controller
    load_gripper_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_gripper_controller'],
        output='screen'
    )

    # 返回launch描述
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])