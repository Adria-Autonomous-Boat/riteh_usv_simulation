from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable

from launch import LaunchDescription
import os

from usv_py.bridge_config import get_gz_bridge_config
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction

PACKAGE_NAME = 'riteh_usv_sim'


def launch_setup(context):
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_name = context.launch_configurations['world']
    model_name_base = context.launch_configurations['model']
    params_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", model_name_base)

    vessel_model_params = os.path.join(params_dir, "usv_control_params.yaml")

    # Load the SDF file from "description" package
    sdf_file = os.path.join(get_package_share_directory(PACKAGE_NAME), 'models', model_name_base, 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    model_name = model_name_base + '_0'

    world = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'worlds',
        world_name + '.sdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_desc,
                'publish_frequency': 30.0,
            }
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[sdf_file],
        parameters=[{'use_sim_time': True}]
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    # PX4 related processes
    micro_xrce_agent = ExecuteProcess(
        cmd=['gnome-terminal', '--tab', '--title=MicroXRCE Agent', '--', 'bash', '-c',
             'MicroXRCEAgent udp4 -p 8888; exec bash'],
        output='screen'
    )

    px4_sitl = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['gnome-terminal', '--tab', '--title=PX4 SITL', '--', 'bash', '-c',
                     'cd ~/PX4-Autopilot && '
                     'PX4_GZ_STANDALONE=1 '
                     'PX4_SYS_AUTOSTART=22002 '
                     'PX4_SIMULATOR=gz '
                     'PX4_GZ_MODEL_POSE=\'-455,172,0,0,0,1.57\' '
                     f'PX4_GZ_WORLD={world_name} PX4_SIM_MODEL={model_name_base} '
                     './build/px4_sitl_default/bin/px4; exec bash'],
                output='screen'
            )
        ]
    )

    # Spawn Maritime Robotics Otter model in the world
    # Config list:
    # HEAD-ON (Otter goes south, toward USV going north):
    #   '-x', '-454', '-y', '220', '-z', '0.5', '-Y', '-1.5708'
    #
    # CROSSING FROM STARBOARD (Otter goes west, crosses USV from right):
    #   '-x', '-420', '-y', '210', '-z', '0.5', '-Y', '3.1416'
    #   '-x', '-420', '-y', '195', '-z', '0.5', '-Y', '3.1416'
    #   '-x', '-430', '-y', '214', '-z', '0.5', '-Y', '-2.356'
    #   '-x', '-435', '-y', '204', '-z', '0.5', '-Y', '-2.5657'
    #
    # CROSSING FROM PORT (Otter crosses USV from left — stand-on):
    #   Simulating when it has enough space to pass safely:
    #   '-x', '-500', '-y', '210', '-z', '0.5', '-Y', '0.0'
    #   Simulating when it doesn't have enough space to pass safely:
    #   '-x', '-490', '-y', '210', '-z', '0.5', '-Y', '0.0'
    #
    #   '-x', '-467', '-y', '214', '-z', '0.5', '-Y', '-0.7854'

    #   Angled approaches — emergency arc expected:
    #   Due east, tight crossing:
    #   '-x', '-472', '-y', '210', '-z', '0.5', '-Y', '0.0'
    #   East-northeast (~22.5°), shallow angle:
    #   '-x', '-468', '-y', '205', '-z', '0.5', '-Y', '0.3927'
    #   East-southeast (~-22.5°), steep angle from north:
    #   '-x', '-468', '-y', '215', '-z', '0.5', '-Y', '-0.3927'
    #   Northeast (45°), angled from southwest:
    #   '-x', '-465', '-y', '198', '-z', '0.5', '-Y', '0.7854'
    #   Southeast (-45°), angled from northwest:
    #   '-x', '-465', '-y', '222', '-z', '0.5', '-Y', '-0.7854'


    spawn_otter = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'otter_usv_0',
                    '-file',
                    os.path.join(get_package_share_directory(PACKAGE_NAME), 'models', 'otter_usv', 'model.sdf'),
                    '-x', '-430', '-y', '190', '-z', '0.5', '-Y', '2.7075'
                ],
                output='screen'
            )
        ]
    )

    qground_control = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['gnome-terminal', '--tab', '--title=QGroundControl', '--', 'bash', '-c',
                     'cd ~/QGroundControl && ./QGroundControl.AppImage; exec bash'],
                output='screen'
            )
        ]
    )

    ######## ROS2 nodes ########
    engines_controller_sim = Node(
        package=PACKAGE_NAME,
        executable='engines_controller_sim.py',
        name='engines_controller_sim',
        parameters=[vessel_model_params]
    )

    differential_drive = Node(
        package=PACKAGE_NAME,
        executable='differential_drive',
        name='differential_drive'
    )

    differential_kinematics = Node(
        package=PACKAGE_NAME,
        executable='differential_kinematics',
        name='differential_kinematics'
    )

    differential_guidance = Node(
        package=PACKAGE_NAME,
        executable='differential_guidance',
        name='differential_guidance',
        parameters=[vessel_model_params]
    )

    differential_drive_kinematics = Node(
        package=PACKAGE_NAME,
        executable='differential_drive_kinematics',
        name='differential_drive_kinematics',
    )

    frame_publisher = Node(
        package=PACKAGE_NAME,
        executable='frame_publisher',
        name='frame_publisher',
        parameters=[{'use_sim_time': True}]
    )

    pid_qgc_server = Node(
        package=PACKAGE_NAME,
        executable='pid_qgc_server',
        name='pid_qgc_server'
    )

    nav2_qgc_server = Node(
        package=PACKAGE_NAME,
        executable='nav2_qgc_server',
        name='nav2_qgc_server'
    )

    # Ros2 - Gazebo bridging
    bridges = get_gz_bridge_config(
        world_name=world_name,
        model_name=model_name,
        enable_lidar=True,
        enable_camera=True,
        enable_wind=False
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    )

    delayed_pid_nodes = TimerAction(
        period=12.0,
        actions=[
            engines_controller_sim,
            differential_kinematics,
            differential_guidance,
            differential_drive,
            pid_qgc_server
        ]
    )

    ######### PID algorithm with QGC required nodes #########
    # return [
    #     gzserver,
    #     micro_xrce_agent,
    #     px4_sitl,
    #     qground_control,
    #     ros_gz_bridge,
    #     robot_state_publisher,
    #     joint_state_publisher_node,
    #     frame_publisher,
    #     delayed_pid_nodes
    # ]

    ######### Nav2 with Rviz waypoints required nodes #########
    return [
        gzserver,
        gzclient,
        micro_xrce_agent,
        px4_sitl,
        spawn_otter,
        # qground_control,
        ros_gz_bridge,
        robot_state_publisher,
        joint_state_publisher_node,
        engines_controller_sim,
        frame_publisher,
        differential_drive_kinematics
    ]

    ######### Nav2 with QGC waypoints required nodes #########
    # return [
    #     gzserver,
    #     #gzclient,
    #     micro_xrce_agent,
    #     px4_sitl,
    #     qground_control,
    #     ros_gz_bridge,
    #     robot_state_publisher,
    #     joint_state_publisher_node,
    #     engines_controller_sim,
    #     frame_publisher,
    #     differential_drive_kinematics,
    #     nav2_qgc_server
    # ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='sydney_regatta_colreg',
        description='Name of world'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='njord_usv',
        description='Name of SDF model'
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        OpaqueFunction(function=launch_setup)
    ])
