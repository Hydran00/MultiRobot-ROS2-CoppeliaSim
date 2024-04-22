from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os,time

# Not elegant but works
# os.system("ros2 topic pub /stopSimulation std_msgs/msg/Bool '{data: true}' --once")
# time.sleep(0.5)
# os.system("ros2 topic pub /startSimulation std_msgs/msg/Bool '{data: true}' --once")
# time.sleep(0.5)

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
        namespace="robot1_",
        output="both",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ]
    )
    # Joint states broadcaster for RViz
    joint_state_broadcaster_spawner1 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot1_",
        arguments=["joint_state_broadcaster", "-c", "/robot1_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )


    # TF tree
    robot_state_publisher1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot1_",
        parameters=[robot_description1,{"use_sim_time": use_sim_time, "publish_frequency": 100.0}],
    )

    static_broadcaster1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y", "-0.75", "--frame-id", "world", "--child-frame-id", PREFIX_LIST[0]+"base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="log"
    )

    static_broadcaster2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y","0.75","--yaw", "-3.1416", "--frame-id", "world", "--child-frame-id", PREFIX_LIST[1]+"base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="log"
    )
    joint_state_broadcaster_spawner2 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot2_",
        arguments=["joint_state_broadcaster", "-c", "/robot2_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    control_node2 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description2, robot_controllers,{"use_sim_time": True}],
        namespace="robot2_",
        output="both",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ]
    )

    robot_state_publisher2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot2_",
        parameters=[robot_description2,{"use_sim_time": use_sim_time, "publish_frequency": 100.0}],
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


    ## CONTROLLERS
    cartesian_motion_controller_spawner1 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot1_",
        arguments=["cartesian_motion_controller", "-c", "/robot1_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    motion_control_handle_spawner1 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot1_",
        arguments=["motion_control_handle", "-c", "/robot1_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    cartesian_motion_controller_spawner2 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot2_",
        arguments=["cartesian_motion_controller", "-c", "/robot2_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    motion_control_handle_spawner2 = Node(
        package="controller_manager",
        executable=spawner,
        namespace="robot2_",
        arguments=["motion_control_handle", "-c", "/robot2_/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Static broadcasters
    static_broadcaster1_t = TimerAction(period=2.0,
            actions=[static_broadcaster1, static_broadcaster2])
    # static_broadcaster2_t = TimerAction(period=0.0,
    #         actions=[static_broadcaster2])
    
    # robot nodes
    nodes = TimerAction(period=0.0,
        actions=[
        robot_state_publisher1,
        joint_state_broadcaster_spawner1,
        control_node1,

        control_node2,
        joint_state_broadcaster_spawner2,
        robot_state_publisher2,

    ])

    
    rviz_t = TimerAction(period=0.0,
            actions=[rviz])

    # controllers are launched only when robot's base_link(s) are in the correct position
    controllers = TimerAction(period=4.0,
                              actions=[
                                    cartesian_motion_controller_spawner1,
                                    # motion_control_handle_spawner1,
                                    cartesian_motion_controller_spawner2,
                                    # motion_control_handle_spawner2,
                                ])

    # include the launch file for point clouds
    # point_clouds_converter = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ur_coppeliasim'), 'launch', 'pc2_coppeliasim.launch.py')
    #     ),
    #     launch_arguments=[('prefixes', PREFIX_LIST)]
    # )

    # return LaunchDescription([nodes] + [rviz_t] + [controllers] + [point_clouds_converter])
    return LaunchDescription([static_broadcaster1_t] + [nodes] + [rviz_t] + [controllers])# + [point_clouds_converter])




