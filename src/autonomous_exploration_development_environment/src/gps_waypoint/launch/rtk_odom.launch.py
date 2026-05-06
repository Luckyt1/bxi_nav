from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fix_topic',
            default_value='/fix',
            description='RTK GPS NavSatFix 话题名'
        ),
        DeclareLaunchArgument(
            'src_odom_topic',
            default_value='/aft_mapped_to_init',
            description='提供朝向的源里程计话题名（如 Point-LIO 输出）'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/state_estimation',
            description='融合后发布的里程计话题名（供导航框架使用）'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_init',
            description='里程计父坐标系'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='aft_mapped',
            description='里程计子坐标系（机器人本体）'
        ),

        Node(
            package='gps_waypoint',
            executable='rtk_odom',
            name='rtk_odometry_node',
            output='screen',
            parameters=[{
                'fix_topic':      LaunchConfiguration('fix_topic'),
                'src_odom_topic': LaunchConfiguration('src_odom_topic'),
                'odom_topic':     LaunchConfiguration('odom_topic'),
                'frame_id':       LaunchConfiguration('frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
            }]
        ),
    ])
