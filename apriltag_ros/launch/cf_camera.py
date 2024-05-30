import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from yaml.loader import SafeLoader

def generate_launch_description():
    
    # load ip and port from crazyflies.yaml
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    with open(crazyflies_yaml, 'r') as ymlfile:
        cf = yaml.safe_load(ymlfile)

    robots_list = cf['robots']

    ld = LaunchDescription()

    for x in robots_list.keys():
        # load calibration
        calibration_yaml = os.path.join(
            get_package_share_directory('apriltag_ros'),
            'calibration',
            x + '.yaml')
        
        with open(calibration_yaml, 'r') as ymlfile:
            calibration = yaml.safe_load(ymlfile)
        
        calib_params = [calibration] + [cf]
        
        camera_node = Node(
                package="apriltag_ros",
                executable="cf_opencv_publisher.py",
                name='cf_streamer_' + x,
                output="screen",
                parameters=calib_params
            )
        ld.add_action(camera_node)

    return ld