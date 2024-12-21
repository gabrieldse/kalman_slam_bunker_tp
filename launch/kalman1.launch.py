import os
import launch
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

os.environ["GAZEBO_MODEL_PATH"] = os.path.join(
    "/root/ros_ws/src/pol_bunker/", "models"
)

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = True
    xacro_file = os.path.join(
        get_package_share_directory("pol_bunker"), "urdf/rob_perception.xacro"
    )

    robot_desc = xacro.process_file(xacro_file).toxml()


    rviz_config_bunker = os.path.join(
        get_package_share_directory("pol_bunker"), "urdf/bunker.rviz"
    )

    world_file = 'perception2-1.world'
    # world_file = 'perception2-2.world'
    # world_file = 'perception2-3.world'

    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_sim_time}],
    )


    spawn_entity1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "DT1",
            "-z 0.5",
        ],
    )

    node_tf1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0", "0", "0", "0", "world", "odom"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_bunker],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=[
            os.path.join(get_package_share_directory("pol_bunker"), "urdf", world_file),
            "",
        ],
        description="SDF world file",
    )


    apriltag=Node(
        package="pol_bunker",
        executable="apriltag_detect"
    )

    Kalman=Node(
        package="pol_bunker",
        executable="kalman1"
    )
    
    # Delay to only start the movement command once gazebo is fully launched.
    Kalman_delayed = TimerAction(
        period=5.0,  
        actions=[Kalman]
    )   

    ld.add_action(apriltag)
    ld.add_action(Kalman_delayed)
    ld.add_action(world_arg)
    ld.add_action(robot_state_node)
    ld.add_action(spawn_entity1)
    ld.add_action(node_tf1)
    ld.add_action(gazebo)
 
    return ld
