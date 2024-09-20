from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar2cam',
            executable='lidar2cam_node',
            name='lidar2cam_node',
            output='screen',
            parameters=[{'lidar_frame': 'ego_vehicle/LIDAR_TOP',
                         'camera_frame': 'ego_vehicle/CAM_BACK'}],
            remappings=[
                ('/lidar_points', '/carla/ego_vehicle/LIDAR_TOP'),
                ('/camera_image', '/carla/ego_vehicle/CAM_BACK'),
                ('/camera_info', '/carla/ego_vehicle/CAM_BACK/camera_info')
            ]
        ),
    ])
