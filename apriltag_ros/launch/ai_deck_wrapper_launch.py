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

    # load apriltag
    cfg_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg', 'tags_36h11.yaml')

    with open(cfg_yaml, 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    with open(crazyflies_yaml, 'r') as ymlfile:
        cf = yaml.safe_load(ymlfile)

    robots_list = cf['robots']

    ld = LaunchDescription()

    for x in robots_list.keys():
        # Camera node
        camera_node_params = [cf] + [{'camera_name': x}]

        camera_node = Node(
            package="apriltag_ros",
            executable="ai_deck_wrapper.py",
            name='cf_streamer_' + x,
            output="screen",
            remappings=[
                ('image', x + '/image'),
                ('camera_info', x + '/camera_info')
            ],
            parameters=camera_node_params
        )

        # AprilTag node
        tag_node_params = [cfg]

        tag_node = Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name='apriltag_node' + x,
            output="screen",
            remappings=[
                ('image_rect', x + '/image'),
                ('camera_info', x + '/camera_info'),
                ('detections', x + '/tag'),
            ],
            parameters=tag_node_params
        )

        ld.add_action(camera_node)
        ld.add_action(tag_node)

    return ld