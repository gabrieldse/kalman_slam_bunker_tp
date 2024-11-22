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

### MULTI ROBOTS
# https://www.theconstruct.ai/spawning-multiple-robots-in-gazebo-with-ros2/
# https://robotics.snowcron.com/robotics_ros2/multi_bot_03_intro.htm



def generate_launch_description():
    ld=LaunchDescription()
    use_sim_time = True
    # xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/bunker.xacro')
    # xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/my_2roues.urdf.xacro')
    xacro_file=os.path.join(get_package_share_directory('pol_bunker'),'urdf/my_bunker.xacro')
    xacro_file2=os.path.join(get_package_share_directory('pol_bunker'),'urdf/my_bunker2.xacro')
    
    robot_desc=xacro.process_file(xacro_file,mappings={"namespace":"/DT1"}).toxml()
    robot_desc2=xacro.process_file(xacro_file,mappings={"namespace":"/DT2"}).toxml()
    # robot_desc=xacro.process_file(xacro_file).toxml()
    # robot_desc2=xacro.process_file(xacro_file).toxml()
    
    rviz_config_bunker=os.path.join(get_package_share_directory('pol_bunker'),'urdf/bunker.rviz')

    # urdf = os.path.join(get_package_share_directory('pol_bunker'),'urdf/test.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

#     world_file = 'cohoma.world'
    world_file = 'underwater.world'
    # world_file = 'test.world'
    

#  Pour le premier bunker
    robot_state_node=Node(
          package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='DT1',
            parameters=[{'robot_description': robot_desc,
                'use_sim_time': use_sim_time}]
    
    )
    robot_state_node2=Node(
          package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='DT2',
            parameters=[{'robot_description': robot_desc2,
                'use_sim_time':use_sim_time}]
    
    )
    
    joint_state_node=Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace='DT1',
                parameters=[{'robot_description': robot_desc,
                'use_sim_time': use_sim_time}]
    )
    joint_state_node2=Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace='DT2',
                parameters=[{'robot_description': robot_desc2,
                'use_sim_time': use_sim_time}]
    )



    spawn_entity1=Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            namespace='/DT1',
            arguments=["-topic", "/DT1/robot_description", 
                        "-entity", "DT1",
                        '-robot_namespace',"DT1", 
                        "-z 0.5"],
    )
    spawn_entity2=Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            namespace='/DT2',
            arguments=["-topic", "/DT2/robot_description",
                        "-entity", "DT2", 
                        '-robot_namespace', "DT2", 
                        "-y -5.0", "-z 0.5"]                
    )

    node_tf1=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0','0.0','0','0','0','0','world','DT1/odom']
    )
    node_tf2=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0','0.0','0','0','0','0','world','DT2/odom']
    )

# BUNKER 2
    

    

#        RVIZ
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_config_bunker]

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



    # gazebo_exit_event_handler = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #     target_action=gazebo_node,
    #     on_exit=EmitEvent(event=Shutdown(reason='gazebo exited'))))
                        
    # rviz_exit_event_handler = RegisterEventHandler(
    # event_handler=OnProcessExit(
    #     target_action=rviz_node,
    #     on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))


    ld.add_action(world_arg)
    ld.add_action(rviz_node)    

    ld.add_action(robot_state_node)
    ld.add_action(joint_state_node)     
    ld.add_action(spawn_entity1)
    ld.add_action(node_tf1)

    ld.add_action(robot_state_node2)
    ld.add_action(joint_state_node2)    
    ld.add_action(spawn_entity2)
    ld.add_action(node_tf2)
 
    ld.add_action(gazebo)

    # ld.add_action(gazebo_exit_event_handler)
    # ld.add_action(rviz_exit_event_handler)


    return ld
