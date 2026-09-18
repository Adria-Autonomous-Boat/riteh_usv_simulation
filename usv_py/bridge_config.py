from usv_py.gz_utils.bridge import Bridge, BridgeDirection
from rclpy.logging import get_logger


logger = get_logger(__name__)

def get_gz_bridge_config(
        world_name: str,
        model_name: str,
        enable_lidar: bool = True,
        enable_camera: bool = True,
        enable_wind: bool = False,
) -> list[Bridge]:
    """
    Build Gazebo-to-ROS bridge configuration for the USV model.

    Args:
        world_name: Name of the Gazebo world.
        model_name: Name of the USV model (usv_model01_0, usv_model02_0, or wamv_model_0).
        enable_lidar: Whether to enable LiDAR sensor bridges. Defaults to True.
        enable_camera: Whether to enable camera sensor bridges. Defaults to True.
        enable_wind: Whether to enable wind sensor bridges. Defaults to False.

    Returns:
        A list of Bridge objects configured for the specified model and sensors.

    Raises:
        ValueError: If the model_name is not supported.
    """

    logger.info(f"Building bridge config: world={world_name} model={model_name}")
    logger.info(f"Enabling lidar: {enable_lidar}")
    logger.info(f"Enabling camera: {enable_camera}")
    logger.info(f"Enabling wind: {enable_wind}")

    # Base topics that are always enabled
    bridges = [
        Bridge(
            gz_topic=f'/{model_name}/command/motor_speed',
            ros_topic=f'/usv/motor_speed',
            gz_type='gz.msgs.Actuators',
            ros_type='actuator_msgs/msg/Actuators',
            direction=BridgeDirection.GZ_TO_ROS),
        Bridge(
            gz_topic=f'/model/{model_name}/joint/left_engine_propeller_joint/cmd_thrust',
            ros_topic=f'/usv/thrusters/left/thrust',
            gz_type='gz.msgs.Double',
            ros_type='std_msgs/msg/Float64',
            direction=BridgeDirection.ROS_TO_GZ),
        Bridge(
            gz_topic=f'/model/{model_name}/joint/right_engine_propeller_joint/cmd_thrust',
            ros_topic=f'/usv/thrusters/right/thrust',
            gz_type='gz.msgs.Double',
            ros_type='std_msgs/msg/Float64',
            direction=BridgeDirection.ROS_TO_GZ),
        Bridge(
            gz_topic=f'/clock',
            ros_topic=f'/clock',
            gz_type='gz.msgs.Clock',
            ros_type='rosgraph_msgs/msg/Clock',
            direction=BridgeDirection.GZ_TO_ROS),
        Bridge(
            gz_topic=f'/cmd_vel',
            ros_topic=f'/cmd_vel',
            gz_type='gz.msgs.Twist',
            ros_type='geometry_msgs/msg/Twist',
            direction=BridgeDirection.GZ_TO_ROS),
        Bridge(
            gz_topic=f'/world/{world_name}/model/{model_name}/link/navsat_link/sensor/navsat_sensor/navsat',
            ros_topic=f'/navsat',
            gz_type='gz.msgs.NavSat',
            ros_type='sensor_msgs/msg/NavSatFix',
            direction=BridgeDirection.GZ_TO_ROS)
    ]

    if enable_wind:
        wind_speed = Bridge(
            gz_topic=f'/vrx/debug/wind/speed',
            ros_topic=f'/wind/speed',
            gz_type='gz.msgs.Float',
            ros_type='std_msgs/msg/Float32',
            direction=BridgeDirection.GZ_TO_ROS)

        wind_direction = Bridge(
            gz_topic=f'/vrx/debug/wind/direction',
            ros_topic=f'/wind/direction',
            gz_type='gz.msgs.Float',
            ros_type='std_msgs/msg/Float32',
            direction=BridgeDirection.GZ_TO_ROS)

        set_wind_params = Bridge(
            gz_topic=f'/vrx/debug/set_wind_params',
            ros_topic=f'/wind/set_wind_params',
            gz_type='gz.msgs.Vector3d',
            ros_type='geometry_msgs/msg/Vector3',
            direction=BridgeDirection.ROS_TO_GZ)

        bridges.append(wind_speed)
        bridges.append(wind_direction)
        bridges.append(set_wind_params)

    if model_name == 'usv_model02_0' or model_name == 'wamv_model_0':
        if enable_camera:
            left_camera_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/left_camera_link/sensor/zed_camera/image',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/image_raw',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            right_camera_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/right_camera_link/sensor/zed_camera/image',
                ros_topic=f'/sensors/cameras/zed2i_right_camera/image_raw',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_depth_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/left_camera_link/sensor/zed_depth_camera/depth_image',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/depth_image',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            right_camera_depth_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/right_camera_link/sensor/zed_depth_camera/depth_image',
                ros_topic=f'/sensors/cameras/zed2i_right_camera/depth_image',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_pointcloud = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/left_camera_link/sensor/zed_depth_camera/depth_image/points',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/pointcloud',
                gz_type='gz.msgs.PointCloudPacked',
                ros_type='sensor_msgs/msg/PointCloud2',
                direction=BridgeDirection.GZ_TO_ROS)

            right_camera_pointcloud = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/right_camera_link/sensor/zed_depth_camera/depth_image/points',
                ros_topic=f'/sensors/cameras/zed2i_right_camera/pointcloud',
                gz_type='gz.msgs.PointCloudPacked',
                ros_type='sensor_msgs/msg/PointCloud2',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_info = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/left_camera_link/sensor/zed_camera/camera_info',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/camera_info',
                gz_type='gz.msgs.CameraInfo',
                ros_type='sensor_msgs/msg/CameraInfo',
                direction=BridgeDirection.GZ_TO_ROS)

            right_camera_info = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/right_camera_link/sensor/zed_camera/camera_info',
                ros_topic=f'/sensors/cameras/zed2i_right_camera/camera_info',
                gz_type='gz.msgs.CameraInfo',
                ros_type='sensor_msgs/msg/CameraInfo',
                direction=BridgeDirection.GZ_TO_ROS)

            bridges.extend([
                left_camera_image,
                right_camera_image,
                left_camera_depth_image,
                right_camera_depth_image,
                left_camera_pointcloud,
                right_camera_pointcloud,
                left_camera_info,
                right_camera_info
            ])

        if enable_lidar:
            lidar_scan = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/lidar_link/sensor/rplidar/scan',
                ros_topic=f'/sensors/rplidar/scan',
                gz_type='gz.msgs.LaserScan',
                ros_type='sensor_msgs/msg/LaserScan',
                direction=BridgeDirection.GZ_TO_ROS)
            bridges.append(lidar_scan)

    elif model_name == 'usv_model01_0':
        if enable_camera:
            left_camera_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/camera_link/sensor/zed2i_left_camera/image',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/image_raw',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_info = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/camera_link/sensor/zed2i_left_camera/camera_info',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/camera_info',
                gz_type='gz.msgs.CameraInfo',
                ros_type='sensor_msgs/msg/CameraInfo',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_depth_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/camera_link/sensor/zed2i_depth_camera/depth_image',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/depth_image',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            left_camera_pointcloud = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/camera_link/sensor/zed2i_depth_camera/depth_image/points',
                ros_topic=f'/sensors/cameras/zed2i_left_camera/pointcloud',
                gz_type='gz.msgs.PointCloudPacked',
                ros_type='sensor_msgs/msg/PointCloud2',
                direction=BridgeDirection.GZ_TO_ROS)

            bridges.extend([
                left_camera_image,
                left_camera_info,
                left_camera_depth_image,
                left_camera_pointcloud
            ])

        if enable_lidar:
            lidar_scan = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/lidar_link/sensor/rplidar/scan',
                ros_topic=f'/sensors/rplidar/scan',
                gz_type='gz.msgs.LaserScan',
                ros_type='sensor_msgs/msg/LaserScan',
                direction=BridgeDirection.GZ_TO_ROS)

            bridges.append(lidar_scan)

    elif model_name == 'njord_usv_0':
        if enable_camera:
            zed_camera_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/zed_camera_link/sensor/zed_camera/image',
                ros_topic=f'/sensors/cameras/zed_camera/image_raw',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            zed_camera_depth_image = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/zed_camera_link/sensor/zed_depth_camera/depth_image',
                ros_topic=f'/sensors/cameras/zed_camera/depth_image',
                gz_type='gz.msgs.Image',
                ros_type='sensor_msgs/msg/Image',
                direction=BridgeDirection.GZ_TO_ROS)

            zed_camera_pointcloud = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/zed_camera_link/sensor/zed_depth_camera/depth_image/points',
                ros_topic=f'/sensors/cameras/zed_camera/pointcloud',
                gz_type='gz.msgs.PointCloudPacked',
                ros_type='sensor_msgs/msg/PointCloud2',
                direction=BridgeDirection.GZ_TO_ROS)

            zed_camera_info = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/zed_camera_link/sensor/zed_camera/camera_info',
                ros_topic=f'/sensors/cameras/zed_camera/camera_info',
                gz_type='gz.msgs.CameraInfo',
                ros_type='sensor_msgs/msg/CameraInfo',
                direction=BridgeDirection.GZ_TO_ROS)

            bridges.extend([
                zed_camera_image,
                zed_camera_depth_image,
                zed_camera_pointcloud,
                zed_camera_info
            ])

        if enable_lidar:
            lidar_scan = Bridge(
                gz_topic=f'/world/{world_name}/model/{model_name}/link/lidar_link/sensor/livox_lidar/scan/points',
                ros_topic=f'/sensors/livox_lidar/scan',
                gz_type='gz.msgs.PointCloudPacked',
                ros_type='sensor_msgs/msg/PointCloud2',
                direction=BridgeDirection.GZ_TO_ROS)

            bridges.append(lidar_scan)

    else:
        logger.error(f'Model {model_name} not supported.')
        raise ValueError(f'Model {model_name} not supported.')

    return bridges