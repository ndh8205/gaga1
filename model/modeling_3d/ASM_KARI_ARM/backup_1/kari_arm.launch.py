import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_orbit_sim = get_package_share_directory('orbit_sim')
    
    world_file = os.path.join(pkg_orbit_sim, 'worlds', 'kari_arm.sdf')

    # Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Bridge for ROS2 communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/kari_arm_world/model/kari_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/kari_arm_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/kari_arm/joint/joint1/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint2/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint3/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint4/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint5/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint6/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kari_arm/joint/joint7/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
    ])