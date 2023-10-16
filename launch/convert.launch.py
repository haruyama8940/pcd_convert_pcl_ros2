from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd_convert_pcl_ros2',
            namespace='pcd_convert_pcl_ros2',
            executable='pcd_to_pcl_node',
            parameters=[{
                        # 'map_path':'/home/haru/bags/ros2bag_tsudanuma_gaisyu/0811/range120/trans_map.pcd',
                        'map_path':'/home/haru/bags/tsukuba/2023/0817/0817_tsukuba.pcd',
                        'frame_id':'map',
                        'use_downsampling':True,
                        'voxel_leafsize':0.5
                        }]
        )
    ])