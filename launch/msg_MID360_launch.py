import os
import time
import subprocess
import threading



from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    GroupAction,
    TimerAction,
)

# For respawing component container
from launch.actions import RegisterEventHandler
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_started import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart


def is_container_name_in_process_cmd(container_name: str, process_cmd: list) -> bool:
    """! Function to check whether a component name is in a process cmd
    @param container_name (str) the name of the container to search for
    @param process_cmd (str) the process' cmd
    @return bool Whether the container name is in the process cmd
    """
    for element in process_cmd:
        if container_name in element:
            return True
    return False


################### user configure parameters for ros2 start ###################
xfer_format = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0  # 0-lidar, others-Invalid data src
publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = "livox_link"
lvx_file_path = "/home/livox/livox_test.lvx"
cmdline_bd_code = "livox0000000001"

cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
cur_config_path = cur_path + "../config"
user_config_path = os.path.join(cur_config_path, "MID360_config.json")

container_name = "livox_360_container"

################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
]

# Lidar 3D filter Params
lidar_3d_filter_params_file = None
try:
    lidar_3d_filter_params_file = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "3d_lidar_filter_params.yaml",
    )
except Exception as e:
    print(f"3D Lidar filter failed reading params file. Got: {e}")


def generate_launch_description():
    # livox_driver = Node(
    #     package="livox_ros_driver2",
    #     executable="livox_ros_driver2_node",
    #     name="livox_lidar_publisher",
    #     output="screen",
    #     respawn=True,
    #     respawn_delay=5.0,
    #     parameters=livox_ros2_params,
    # )
    def livox360_composed_launch():
        return GroupAction(
            actions=[
                Node(
                    name=container_name,
                    package="rclcpp_components",
                    executable="component_container_isolated",
                    output="both",
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        LoadComposableNodes(
                            target_container=container_name,
                            composable_node_descriptions=[
                                ComposableNode(
                                    package="pcl_ros",
                                    plugin="pcl_ros::CropBox",
                                    name="pcl_box_filter",
                                    remappings=[
                                        ("input", "/livox/lidar"),
                                        ("output", "/livox/filtered"),
                                    ],
                                    parameters=[lidar_3d_filter_params_file],
                                ),
                                ComposableNode(
                                    package="livox_ros_driver2",
                                    plugin="livox_ros::DriverNode",
                                    name="livox_lidar_publisher",
                                    parameters=livox_ros2_params,
                                ),
                            ],
                        ),
                    ],
                ),
            ],
        )

    def relaunch_livox360_component(event: ProcessExited, context: LaunchContext):
        if (
            event.returncode != 0
            and "component_container_isolated" in event.action.name
            and container_name in event.cmd[-1]
        ):
            print(
                "\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
                    event.action.name, event.pid, event.returncode
                )
            )
            print(f"respawning: {event.cmd[-1].split('=')[-1]}...")
            return livox360_composed_launch()  # respawn node action

    def reniceness_execute():
        time.sleep(10)
        print("Renicing LIVOX Component")
        cmd = "ps -eLf | grep 'livox_360' | grep -v grep | awk '{print $4}' | xargs -r -n1 renice -20 -p 1> /dev/null"
        subprocess.call(cmd, shell=True)

    def reniceness_livox360_component(event: ProcessStarted, context: LaunchContext):
        # Start a new thread to run the command
        if (
                "component_container_isolated" in event.action.name
                and is_container_name_in_process_cmd(
                    container_name=container_name, process_cmd=event.cmd
            )
        ):
            threading.Thread(target=reniceness_execute).start()


    respawn_livox360_composition_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(on_exit=relaunch_livox360_component)
    )

    reniceness_livox360_composition_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(on_start=reniceness_livox360_component)
    )

    return LaunchDescription(
        [
            # livox_driver,
            livox360_composed_launch(),
            respawn_livox360_composition_event_handler,
            reniceness_livox360_composition_event_handler,
            # launch.actions.RegisterEventHandler(
            #     event_handler=launch.event_handlers.OnProcessExit(
            #         target_action=livox_rviz,
            #         on_exit=[
            #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
            #         ]
            #     )
            # )
        ]
    )
