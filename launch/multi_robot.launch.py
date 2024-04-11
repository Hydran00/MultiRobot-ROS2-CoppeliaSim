from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

# prefix to distinguish between the two robots
PREFIX_LIST = ['1_', '2_']

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    use_sim_time = True
    description_package = get_package_share_directory('ur_coppeliasim')
    xacro_path = os.path.join(description_package,"urdf","ur.urdf.xacro")
    robot_controllers = os.path.join(description_package,"config", "multi_robot_controllers.yaml")
    # initial_joint_controllers = os.path.join(description_package,"config", "ur_controllers_coppelia.yaml")


    ur_type="ur3e"
    robot_description_content1 = xacro.process_file(xacro_path, mappings={"safety_limits":"true","safety_pos_margin":"0.15",
                                                                        "safety_k_position":"20",
                                                                        "name":"ur","ur_type":ur_type,
                                                                        "prefix":PREFIX_LIST[0],"sim_ignition":"false","sim_gazebo":"false",
                                                                        "simulation_controllers":robot_controllers})
    robot_description_content2 = xacro.process_file(xacro_path, mappings={"safety_limits":"true","safety_pos_margin":"0.15",
                                                                        "safety_k_position":"20",
                                                                        "name":"ur","ur_type":ur_type,
                                                                        "prefix":PREFIX_LIST[1],"sim_ignition":"false","sim_gazebo":"false",
                                                                        "simulation_controllers":robot_controllers})

    robot_description_content1 = robot_description_content1.toprettyxml(indent=' ')
    robot_description_content2 = robot_description_content2.toprettyxml(indent=' ')

    robot_description1 = {"robot_description": robot_description_content1}
    robot_description2 = {"robot_description": robot_description_content2}


    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.

    
    control_node1 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description1, robot_controllers,{"use_sim_time": use_sim_time}],
        namespace="robot1",
        output="both",
    )
    # Joint states broadcaster for RViz
    joint_state_broadcaster_spawner1 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot1",
        arguments=["joint_state_broadcaster", "-c", "/robot1/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )


    # TF tree
    robot_state_publisher1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot1",
        parameters=[robot_description1,{"use_sim_time": use_sim_time}],
    )

    static_broadcaster1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y", "-1.0", "--frame-id", "world", "--child-frame-id", PREFIX_LIST[0]+"base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="log"
    )

    static_broadcaster2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y","1.0","--yaw", "-3.1416", "--frame-id", "world", "--child-frame-id", PREFIX_LIST[1]+"base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="log"
    )
    joint_state_broadcaster_spawner2 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot2",
        arguments=["joint_state_broadcaster", "-c", "/robot2/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    control_node2 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description2, robot_controllers,{"use_sim_time": True}],
        namespace="robot2",
        output="both",
    )

    robot_state_publisher2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot2",
        parameters=[robot_description2,{"use_sim_time": use_sim_time}],
    )

    # include the launch file for point clouds
    point_clouds_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pc2_coppeliasim'), 'launch', 'pc2_coppeliasim.launch.py')
        ),
        launch_arguments=[('prefixes', PREFIX_LIST)]
    )

    # Visualization
    rviz_config = os.path.join(get_package_share_directory('ur_coppeliasim'), 'rviz', 'robot.rviz')
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
        robot_state_publisher1,
        joint_state_broadcaster_spawner1,
        control_node1,

        control_node2,
        joint_state_broadcaster_spawner2,
        robot_state_publisher2,

        rviz,

        # static_broadcaster1,
        # static_broadcaster2
    ]
    transform_broadcasters1 = TimerAction(period=1.0,
            actions=[static_broadcaster1])
    transform_broadcasters2 = TimerAction(period=1.0,
            actions=[static_broadcaster2])

    return LaunchDescription(nodes + [transform_broadcasters1, transform_broadcasters2] + [point_clouds_converter])

