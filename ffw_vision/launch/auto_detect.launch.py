import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# YOLO weights trained outside the repo (2026-10-29); FFW_VISION_MODEL or model_path:= overrides
DEFAULT_MODEL = '/workspace/vision_ws/runs/dyn_seg_n/weights/best.pt'


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('ffw_vision'), 'config')
    camera_id = LaunchConfiguration('camera_id')
    params_file = LaunchConfiguration('params_file')
    task_file = LaunchConfiguration('task_file')
    namespace = ['ffw_vision/', camera_id]
    overrides = {'camera_id': camera_id}

    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='right_wrist',
                              description='head, left_wrist or right_wrist'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(config_dir, 'cameras.yaml')),
        DeclareLaunchArgument('task_file',
                              default_value=os.path.join(config_dir, 'dynamixel_basket_seg.yaml')),
        DeclareLaunchArgument('basket_file',
                              default_value=os.path.join(config_dir, 'blue_basket.yaml')),
        DeclareLaunchArgument('model_path',
                              default_value=EnvironmentVariable('FFW_VISION_MODEL',
                                                                default_value=DEFAULT_MODEL),
                              description='YOLO weights for detector yolo_seg'),
        DeclareLaunchArgument('port', default_value='8765'),
        Node(
            package='ffw_vision', executable='object_detector', name='object_detector',
            namespace=namespace,
            parameters=[params_file, LaunchConfiguration('basket_file'), task_file,
                        {**overrides, 'model_path': LaunchConfiguration('model_path')}],
            output='screen'),
        Node(
            package='ffw_vision', executable='pixel_observer', name='pixel_observer',
            namespace=namespace, parameters=[params_file, overrides], output='screen'),
        Node(
            package='ffw_vision', executable='web_select', name='web_select',
            namespace=namespace, output='screen',
            parameters=[params_file, overrides, {
                'display_topic': 'debug_image',
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}]),
    ])
