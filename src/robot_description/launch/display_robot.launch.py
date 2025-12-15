import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取路径
    urdf_tutorial_path = get_package_share_directory('robot_description')
    urdf_file_path = urdf_tutorial_path + '/urdf/scara.urdf'
    rviz_config_path = urdf_tutorial_path + '/config/display_model.rviz'

    print("urdf_file_path: ", urdf_file_path)
    # 为launch声明参数
    # 这一步实现的效果:
    # ros2 launch robot_description display_robot.launch.py model:=/path/to/other.urdf
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(urdf_file_path),
        description='Absolute path to robot urdf file'
    )
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # rviz2节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )
    # 返回launch描述
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])