import os
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from usv_py.gz_utils.otter_bridge_config import get_otter_bridge_config

PACKAGE_NAME = 'riteh_usv_sim'


def generate_launch_description():
    goal_lat_arg = DeclareLaunchArgument(
        'goal_lat', default_value='-33.721814',
        description='GPS-6 latitude (decimal degrees). Pass "nan" to let the BT service set the goal instead.')
    goal_lon_arg = DeclareLaunchArgument(
        'goal_lon', default_value='150.674820',
        description='GPS-6 longitude (decimal degrees). Pass "nan" to let the BT service set the goal instead.')
    gate_commit_dist_arg = DeclareLaunchArgument(
        'gate_commit_dist', default_value='20.0',
        description='Distance (m) at which to commit to a detected gate.')
    buoy_react_dist_arg = DeclareLaunchArgument(
        'buoy_react_dist', default_value='20.0',
        description='Distance (m) at which to react to a single buoy.')
    max_buoy_depth_arg = DeclareLaunchArgument(
        'max_buoy_depth', default_value='35.0',
        description='Maximum depth (m) at which to detect a buoy.')
    send_to_nav2_arg = DeclareLaunchArgument(
        'send_to_nav2', default_value='true',
        description='Send NavigateToPose goals to Nav2. Set false to only publish goals for inspection.')


    channel_navigator = Node(
        package=PACKAGE_NAME,
        executable='channel_navigator.py',
        name='channel_navigator',
        output='screen',
        parameters=[{
            'final_goal_lat': LaunchConfiguration('goal_lat'),
            'final_goal_lon': LaunchConfiguration('goal_lon'),
            'gate_commit_dist': LaunchConfiguration('gate_commit_dist'),
            'buoy_react_dist': LaunchConfiguration('buoy_react_dist'),
            'max_buoy_depth': LaunchConfiguration('max_buoy_depth'),
            'send_to_nav2': LaunchConfiguration('send_to_nav2'),
        }]
    )

    vessel_detector = Node(
        package=PACKAGE_NAME,
        executable='vessel_detector.py',
        name='vessel_detector',
        output='screen',
    )

    # Delay BT runner to give channel_navigator time to start and get GPS fix
    bt_runner = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', PACKAGE_NAME, 'channel_gate_runner',
                     '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen'
            )
        ]
    )

    otter_bridges = get_otter_bridge_config()
    otter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='otter_bridge',
        output='screen',
        arguments=[b.argument() for b in otter_bridges],
        remappings=[b.remapping() for b in otter_bridges],
    )

    # Change as needed, depending on the position of Otter
    otter_forward = TimerAction(
        period=4.5,
        actions=[
            Node(
                package=PACKAGE_NAME,
                executable='otter_forward.py',
                name='otter_forward',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        goal_lat_arg,
        goal_lon_arg,
        gate_commit_dist_arg,
        buoy_react_dist_arg,
        max_buoy_depth_arg,
        send_to_nav2_arg,
        channel_navigator,
        vessel_detector,
        bt_runner,
        otter_bridge,
        otter_forward,
    ])