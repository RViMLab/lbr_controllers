from ast import arguments
from distutils.util import execute
import controller_manager
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    

    # load robot description
    model = "med7"
    robot_description = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("lbr_description"), "urdf/{}/{}.urdf.xacro".format(model, model)]
            ), " ",
            "sim:=false"
        ]
    )

    robot_description = {"robot_description": robot_description}



    # launch controller manager, robot_description & controller configurations
    controller_configurations = PathJoinSubstitution([
        FindPackageShare("rvim_position_controllers"),
        "config/sample_config.yml"
    ])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_configurations],
        output="screen"
    )

    controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["hand_guide_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[robot_description]
    )

    return LaunchDescription([
        controller_manager,
        controller
    ])
