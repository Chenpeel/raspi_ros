from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    xml_path = LaunchConfiguration("xml_path")
    print_rate_hz = LaunchConfiguration("print_rate_hz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    real_time_factor = LaunchConfiguration("real_time_factor")
    keyframe = LaunchConfiguration("keyframe")
    headless = LaunchConfiguration("headless")

    return LaunchDescription(
        [
            DeclareLaunchArgument("xml_path", default_value="gaoda_jiyuan/scene.xml"),
            DeclareLaunchArgument("print_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="60.0"),
            DeclareLaunchArgument("real_time_factor", default_value="1.0"),
            DeclareLaunchArgument("keyframe", default_value="home"),
            DeclareLaunchArgument("headless", default_value="false"),
            Node(
                package="mjc_viewer",
                executable="mjc_viewer_py",
                name="mjc_viewer",
                output="screen",
                parameters=[
                    {
                        "xml_path": xml_path,
                        "print_rate_hz": print_rate_hz,
                        "publish_rate_hz": publish_rate_hz,
                        "real_time_factor": real_time_factor,
                        "keyframe": keyframe,
                        "headless": headless,
                    }
                ],
            ),
        ]
    )
