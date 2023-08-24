from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd_convert_pcl_ros2',
            namespace='pcd_convert_pcl_ros2',
            executable='pcd_to_pcl_node',
            parameters=[{'map_path':'/home/haru/bags/ros2bag_tsudanuma_gaisyu/0811/range120/map.pcd',
                        'frame_id':'map'
                        }]
        )
    ])