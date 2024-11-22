import os
import launch
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
#pour ajouter l'acces au modeles du package
os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory('pol_bunker'),'models')


def generate_launch_description():
    ld=LaunchDescription()
    # xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/bunker.xacro')
    # xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/my_2roues.urdf.xacro')
    xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/my_bunker.xacro')
    
    robot_desc=xacro.process_file(xacro_file).toxml()
    
    rviz_config_bunker=os.path.join(get_package_share_directory('pol_bunker'),'urdf/bunker.rviz')

    # urdf = os.path.join(get_package_share_directory('pol_bunker'),'urdf/test.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

#     world_file = 'cohoma.world'
#     world_file = 'underwater.world'
    world_file = 'test.world'
    # world_file ='/opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_gps_sensor_demo.world'

    robot_state_node=Node(
          package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
    
    )
    
    joint_state_node=Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                parameters=[{'robot_description': robot_desc}]
    )


    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_config_bunker]

    )

    spawn_entity=Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "bunker", "-z 0.5"]
    )



    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    world_arg = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(
            get_package_share_directory('pol_bunker'),
             'urdf', world_file), ''],
          description='SDF world file')   

    ld.add_action(world_arg)
    ld.add_action(rviz_node)    
    ld.add_action(robot_state_node)
    ld.add_action(joint_state_node)     
    ld.add_action(spawn_entity)
    ld.add_action(gazebo)


    return ld
