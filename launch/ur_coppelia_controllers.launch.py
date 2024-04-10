from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import xacro,os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os,time

# Not elegant but works
os.system("ros2 topic pub /stopSimulation std_msgs/msg/Bool '{data: true}' --once")
time.sleep(0.5)
os.system("ros2 topic pub /startSimulation std_msgs/msg/Bool '{data: true}' --once")
time.sleep(0.5)

distro = os.environ['ROS_DISTRO']
if distro == 'humble' or distro == 'galactic':
    spawner = "spawner"
else:  # foxy
    spawner = "spawner.py"


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    use_sim_time = True
    description_package = get_package_share_directory('ur_coppeliasim')
    xacro_path = os.path.join(description_package,"urdf","ur.urdf.xacro")
    robot_controllers = os.path.join(description_package,"config", "test_controllers.yaml")
    # initial_joint_controllers = os.path.join(description_package,"config", "ur_controllers_coppelia.yaml")


    ur_type="ur3e"
    robot_description_content = xacro.process_file(xacro_path, mappings={"safety_limits":"true","safety_pos_margin":"0.15",
                                                                        "safety_k_position":"20",
                                                                        "name":"ur","ur_type":ur_type,
                                                                        "prefix":'1_',"sim_ignition":"false","sim_gazebo":"false",
                                                                        "simulation_controllers":robot_controllers})
    robot_description_content = robot_description_content.toprettyxml(indent=' ')

    robot_description = {"robot_description": robot_description_content}


    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers,{"use_sim_time": use_sim_time}],
        output="both",
        # remappings=[
        #     ('motion_control_handle/target_frame', 'target_frame'),
        #     ('cartesian_motion_controller/target_frame', 'target_frame'),
        #     ]
    )

    # Joint states broadcaster for RViz
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_motion_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    
    # This controller is the one to move manually the robot in Rviz2 through the 3D cursor.
    # Remove the --stopped argument to start it automatically. You can always start it manually
    # using the RQT plugin or the ROS2 control tool from terminal.
    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable=spawner,
        # arguments=["motion_control_handle", "-c"," --stopped " "/controller_manager"],
        arguments=["motion_control_handle", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    
    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": use_sim_time}],
    )

    # Visualization
    rviz_config = os.path.join(get_package_share_directory('ur_coppeliasim'), 'rviz',
                             'robot.rviz')
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Nodes to start
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        # cartesian_motion_controller_spawner,
        # motion_control_handle_spawner,
        robot_state_publisher,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
