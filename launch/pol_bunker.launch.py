import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    ld=LaunchDescription()
    #valeus dans le script d'origine, valeures par défaut OK
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
    #                                          description='Use simulation clock if true')

    # port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
    #                                      description='CAN bus name, e.g. can0')
    # odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
    #                                        description='Odometry frame id')
    # base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
    #                                             description='Base link frame id')
    # odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
    #                                        description='Odometry topic name')
    # is_bunker_mini_arg = DeclareLaunchArgument('is_bunker_mini', default_value='false',
    #                                       description='Scout mini model')
    # simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
    #                                                description='Whether running with simulator')
    # sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
    #                                              description='Simulation control loop update rate')



    bunker_base=Node(
        package='bunker_base',
        executable='bunker_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
             }])

    gpsd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory(gpsd_client), 'launch'), '/gpsd_client-launch.py']),
             )

    # teleop=Node(
    #         package='teleop_twist_joy', executable='teleop_node',
    #         name='teleop_twist_joy_node', parameters=[config_filepath],
    #         #remappings=[("cmd_vel","PS_cmd_vel")]
    #         remappings=[("joy","DT2/joy")]
    #        )

    bno=Node(
            package='pol_bunker', executable='node_bno055',
            parameters=[{'device': '/dev/ttyUSB0',}]
            )

    tf2_bno=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0","0","0","0","0","0","base_link","imubno055"]
            )
    #attention : UTM :horizontal coordinate system (x= easting, y=northing, z=altitude)
    
    tf2_gps=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0","0","0","0","0","0","base_link","gps"]
            )

    # gotogps=Node(
    #     package='pol_bunker', executable='gotogps_bunker'
    #     )

    ld.add_action(bunker_base)
    # ld.add_action(gpsd)
    ld.add_action(bno)
    ld.add_action(tf2_bno)
    ld.add_action(tf2_gps)
    # ld.add_action(gotogps)
    

    




    return ld
