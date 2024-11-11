import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    microstrain_inertial_driver_dir = get_package_share_directory('microstrain_inertial_driver')
    nav2_gq7_dir = get_package_share_directory("nav2_gq7_demo")
    params_dir = os.path.join(nav2_gq7_dir, "config")
    gq7_params = os.path.join(params_dir, "gq7.yml")
    nav2_params = os.path.join(params_dir, "nav2.yaml")
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )
    test_robot_urdf = os.path.join(nav2_gq7_dir, 'urdf', 'test_robot.urdf.xacro')

    microstrain_inertial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(microstrain_inertial_driver_dir, 'launch', 'microstrain_launch.py')
        ),
        launch_arguments={
          "params_file": gq7_params,
          "namespace": "gq7",
        }.items()
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_nav2_params,
            "autostart": "True",
        }.items()
    )

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            test_robot_urdf
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
        }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Robot Description launch
    ld.add_action(robot_description_command_arg)
    ld.add_action(robot_state_publisher_node)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # GQ7 launch
    ld.add_action(microstrain_inertial_driver_cmd)

    return ld
