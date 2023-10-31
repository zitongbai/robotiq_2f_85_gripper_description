from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# TODO: Namespace or tf_prefix

def generate_launch_description():
    
    description_package_launch_arg = DeclareLaunchArgument(
        "description_package",
        default_value="robotiq_2f_85_gripper_description",
        description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
    )
    description_file_launch_arg = DeclareLaunchArgument(
        "description_file",
        default_value="robotiq_arg2f_85_model.xacro",
        description="URDF/XACRO description file with the robot.",
    )
    tf_prefix_launch_arg = DeclareLaunchArgument(
        "tf_prefix",
        default_value='""',
        description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    #TODO: rviz config file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_gripper.rviz"]
    )

    # ==========================================================
    # Nodes
    # ==========================================================
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )


    return LaunchDescription(
        [
            description_package_launch_arg,
            description_file_launch_arg,
            tf_prefix_launch_arg,
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )