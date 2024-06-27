from launch import LaunchDescription


import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Constants for paths to different files and folders
    gazebo_models_path = "models"
    package_name = "limo_description"
    robot_name_in_model = "limo_four_diff"
    rviz_config_file_path = "rviz/urdf.rviz"
    urdf_file_path = "urdf/limo_four_diff.xacro"
    world_path = "/usr/share/gazebo-11/worlds/empty.world"

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    ###################################
    #   Common Launch Arguments (Used by multiple nodes)
    ###################################

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    ###################################
    #   Common Launch Configs (Used by multiple nodes)
    ###################################
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    ###################################
    #   Gazebo
    ###################################

    gazebo_process = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    ###################################
    #   Robot State publisher
    ###################################

    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    declare_urdf_model_path_arg = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_urdf_model_path,
        description="Absolute path to robot urdf file",
    )

    urdf_model = LaunchConfiguration("urdf_model")

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_model]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    ###################################
    #   Robot Joint State publisher
    ###################################

    declare_use_joint_state_publisher_arg = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )

    gui = LaunchConfiguration("gui")

    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ###################################
    #   Rviz
    ###################################

    rviz_config_file = LaunchConfiguration("rviz_config_file")

    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )

    # Launch RViz
    start_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    ###################################
    #   Spawn entity
    ###################################

    # Pose where we want to spawn the robot
    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "0.00"

    # Launch the robot
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            robot_name_in_model,
            "-topic",
            "robot_description",
            "-x",
            spawn_x_val,
            "-y",
            spawn_y_val,
            "-z",
            spawn_z_val,
            "-Y",
            spawn_yaw_val,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time_arg,
            declare_urdf_model_path_arg,
            declare_use_joint_state_publisher_arg,
            declare_rviz_config_file_arg,
            gazebo_process,
            start_robot_state_publisher_node,
            start_joint_state_publisher_gui_node,
            start_rviz_node,
            spawn_entity_cmd,
        ]
    )
