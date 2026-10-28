"""Start the NXP Cup simulation: Gazebo Jetty world, car, ROS <-> Gazebo bridge.

    ros2 launch nxp_cup_gazebo sim.launch.py                  # sim only, drive it yourself
    ros2 launch nxp_cup_gazebo sim.launch.py follower:=true   # plus the line follower
    ros2 launch nxp_cup_gazebo sim.launch.py obstacle:=true   # with the "end of race" cube
    ros2 launch nxp_cup_gazebo sim.launch.py gui:=false       # headless (camera still renders)
    ros2 launch nxp_cup_gazebo sim.launch.py camera:=linescan follower:=true   # TSL1401

camera:=area (320x240) uses nxp_cup_control/config/line_follower.yaml,
camera:=linescan (TSL1401, 128x1) uses line_follower_linescan.yaml. The camera
mount (height, pitch, field of view) on the simulated car is read from the
camera: section of that same file, so the car and the tracker always agree.

The lap monitor (on by default) prints lap times and, when the car goes off the
track, saves a report with a video of the last seconds to ~/nxp_cup_runs/.

Spawn pose and obstacle pose default to the values in config/track_info.yaml,
which scripts/generate_track.py writes for the current layout.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("nxp_cup_gazebo")
    desc_pkg = get_package_share_directory("nxp_cup_description")

    with open(os.path.join(pkg, "config", "track_info.yaml")) as f:
        track = yaml.safe_load(f)
    spawn, obst = track["spawn"], track["obstacle"]

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    use_sim_time = {"use_sim_time": True}

    args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "nxp_cup.sdf")),
        DeclareLaunchArgument("follower", default_value="false",
                              description="start nxp_cup_control line_follower"),
        DeclareLaunchArgument("monitor", default_value="true",
                              description="lap times + failure reports in ~/nxp_cup_runs"),
        DeclareLaunchArgument("obstacle", default_value="false",
                              description="spawn the 20 cm 'end of race' cube"),
        DeclareLaunchArgument("x", default_value=str(spawn["x"])),
        DeclareLaunchArgument("y", default_value=str(spawn["y"])),
        DeclareLaunchArgument("yaw", default_value=str(spawn["yaw"])),
        DeclareLaunchArgument("camera", default_value="area", choices=["area", "linescan"],
                              description="area: 320x240 camera; linescan: TSL1401 128x1"),
    ]

    # Let Gazebo find model://nxp_cup_track and model://nxp_cup_stop_obstacle
    resource_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg, "models"))

    # -r: start running; -s: server only; --headless-rendering: cameras without a display
    gz_args = PythonExpression([
        "'-r -v 3 ' + ('' if '", gui, "' == 'true' else '-s --headless-rendering ') + '", world, "'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    spawn_car = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-world", "nxp_cup", "-topic", "robot_description", "-name", "nxp_car",
                   "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                   "-z", "0.01", "-Y", LaunchConfiguration("yaw")],
    )

    spawn_obstacle = Node(
        package="ros_gz_sim", executable="create", output="screen",
        condition=IfCondition(LaunchConfiguration("obstacle")),
        arguments=["-world", "nxp_cup", "-name", "stop_obstacle",
                   "-file", os.path.join(pkg, "models", "nxp_cup_stop_obstacle", "model.sdf"),
                   "-x", str(obst["x"]), "-y", str(obst["y"]), "-z", "0.0",
                   "-Y", str(obst["yaw"])],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml")}, use_sim_time],
    )

    monitor = Node(
        package="nxp_cup_control", executable="lap_monitor", output="screen",
        condition=IfCondition(LaunchConfiguration("monitor")),
        parameters=[use_sim_time],
    )

    return LaunchDescription(args + [resource_path, gazebo, OpaqueFunction(function=_car_nodes),
                                     spawn_car, spawn_obstacle, bridge, monitor])


def _car_nodes(context):
    """robot_state_publisher + line follower, set up for the chosen camera."""
    camera = LaunchConfiguration("camera").perform(context)
    ctrl_cfg = os.path.join(get_package_share_directory("nxp_cup_control"), "config",
                            "line_follower_linescan.yaml" if camera == "linescan"
                            else "line_follower.yaml")
    with open(ctrl_cfg) as f:
        mount = yaml.safe_load(f)["line_follower"]["ros__parameters"]["camera"]

    xacro_file = os.path.join(get_package_share_directory("nxp_cup_description"),
                              "urdf", "nxp_cup_car.urdf.xacro")
    robot_description = ParameterValue(Command([
        "xacro ", xacro_file,
        f" camera_type:={camera} camera_height:={mount['height']}"
        f" camera_pitch:={mount['pitch']} camera_hfov:={mount['hfov']}",
    ]), value_type=str)
    use_sim_time = {"use_sim_time": True}

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, use_sim_time],
        output="screen",
    )
    follower = Node(
        package="nxp_cup_control", executable="line_follower", output="screen",
        condition=IfCondition(LaunchConfiguration("follower")),
        parameters=[ctrl_cfg, use_sim_time],
    )
    return [rsp, follower]
