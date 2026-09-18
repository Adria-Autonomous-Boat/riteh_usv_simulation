from usv_py.gz_utils.bridge import Bridge, BridgeDirection


def get_otter_bridge_config(model_name: str = 'otter_usv_0') -> list[Bridge]:
    return [
        Bridge(
            gz_topic=f'/model/{model_name}/joint/left_engine_propeller_joint/cmd_thrust',
            ros_topic=f'/otter/thrusters/left/thrust',
            gz_type='gz.msgs.Double',
            ros_type='std_msgs/msg/Float64',
            direction=BridgeDirection.ROS_TO_GZ),
        Bridge(
            gz_topic=f'/model/{model_name}/joint/right_engine_propeller_joint/cmd_thrust',
            ros_topic=f'/otter/thrusters/right/thrust',
            gz_type='gz.msgs.Double',
            ros_type='std_msgs/msg/Float64',
            direction=BridgeDirection.ROS_TO_GZ),
        Bridge(
            gz_topic=f'/model/{model_name}/odometry',
            ros_topic=f'/otter/odom',
            gz_type='gz.msgs.Odometry',
            ros_type='nav_msgs/msg/Odometry',
            direction=BridgeDirection.GZ_TO_ROS),
    ]
