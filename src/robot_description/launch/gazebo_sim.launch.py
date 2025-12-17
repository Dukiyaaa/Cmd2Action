import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取路径
    robot_name_in_model = 'model'
    urdf_tutorial_path = get_package_share_directory('robot_description')
    default_model_path = urdf_tutorial_path + '/urdf/scara.urdf'
    default_world_path = urdf_tutorial_path + '/world/default_world.world'
    # 为launch声明参数
    # 这一步实现的效果:
    # ros2 launch robot_description display_robot.launch.py model:=/path/to/other.urdf
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 通过 PythonLaunchDescriptionSource包含另外一个launch文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), 
            '/launch',
            '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )

    # 请求gazebo加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ]
    )

    # 返回launch描述
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])