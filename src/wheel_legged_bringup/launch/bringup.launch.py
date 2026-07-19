from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    camera_index = LaunchConfiguration("camera_index")
    dry_run = LaunchConfiguration("dry_run")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="true=只打印USB帧；实机联调 false",
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "camera_index",
                default_value="4",
                description="OpenCV VideoCapture index；D435i 彩色为 /dev/video4",
            ),
            Node(
                package="wheel_legged_bridge",
                executable="h7_bridge",
                name="h7_bridge",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "serial_port": serial_port,
                        "baudrate": 115200,
                    }
                ],
            ),
            Node(
                package="wheel_legged_camera",
                executable="mjpeg_server",
                name="mjpeg_server",
                output="screen",
                parameters=[
                    {
                        "device_index": ParameterValue(camera_index, value_type=int),
                        "host": "0.0.0.0",
                        "port": 8081,
                        "path": "/stream",
                        "fps": 30,
                        "width": 640,
                        "height": 480,
                        "jpeg_quality": 60,
                    }
                ],
            ),
            Node(
                package="wheel_legged_teleop",
                executable="net_gateway",
                name="net_gateway",
                output="screen",
                parameters=[
                    {
                        "status_port": 8082,
                        "command_port": 8083,
                    }
                ],
            ),
        ]
    )
