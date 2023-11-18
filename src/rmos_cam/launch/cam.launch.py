import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition
import yaml

def generate_launch_description():
    # Launch Argument
    camera_type = LaunchConfiguration('camera_type')

    debug_launch_argument_ = DeclareLaunchArgument(name='debug',
                                                default_value='False',
                                                description='True or False')
    camera_launch_argument_ = DeclareLaunchArgument(name='camera_type',
                                                    default_value='daheng',
                                                    description='usb or daheng')

    # Load Param 
    Daheng_Param_file = os.path.join(get_package_share_directory('rmos_bringup'), 'config', 'daheng_node.yaml')
    with open(Daheng_Param_file, 'r') as f:
        Daheng_Param = yaml.safe_load(f)['/daheng_node']['ros__parameters']

    # Set Node
    daheng_node_ = ComposableNode(
                package='rmos_cam',
                plugin='rm::DahengCamNode',
                name='daheng_node',
                parameters=[Daheng_Param],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
    usb_node_ = ComposableNode(
                package='rmos_cam',
                plugin='rm::USBCamNode',
                name='usb_node',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
    cam_sub_node_ = ComposableNode(
                package='rmos_cam',
                plugin='rm::CamSubNode',
                name='cam_sub_node',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}]
            )

    usb_auto_aim_container_ = ComposableNodeContainer(
        name='rmos_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(PythonExpression(["'", camera_type, "'=='usb'"])),
        composable_node_descriptions=[
            usb_node_,
            cam_sub_node_,
        ],
        output='screen',
        emulate_tty=True
    )

    daheng_auto_aim_container_ = ComposableNodeContainer(
        name='rmos_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(PythonExpression(["'", camera_type, "'=='daheng'"])),
        composable_node_descriptions=[
            daheng_node_,
            cam_sub_node_,
        ],
        output='screen',
        emulate_tty=True
    )


    # Done
    return LaunchDescription([
        camera_launch_argument_,
        debug_launch_argument_,

        usb_auto_aim_container_,
        daheng_auto_aim_container_,
    ])
