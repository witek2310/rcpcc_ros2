from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    folder_path_arg = DeclareLaunchArgument(
        'folder_path',
        default_value='.',
        description='Folder path for rcpcc nodes data, must exist'
    )

    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/velodyne/velodyne_points',
        description='Point cloud topic to be subscribed'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/rcpcc_decompressed',
        description='Topic name for decompressed point cloud output'
    )

    compression_mode_arg = DeclareLaunchArgument(
        'compression_ratio',
        default_value='2',
        description='Compression ratio for rcpcc_compress node'
    )


    return LaunchDescription([
        folder_path_arg,
        point_cloud_topic_arg,
        output_topic_arg,
        compression_mode_arg,

        # For compression metrics
        Node(
            package='point_cloud_metrics',
            executable='pointcloud_metrics_node',
            name='pointcloud_metrics_node',
            parameters=[{
                'metrics_csv_path': LaunchConfiguration('folder_path'),
                'cloud1_topic': LaunchConfiguration('point_cloud_topic'),   # Original pointcloud
                'cloud2_topic': LaunchConfiguration('output_topic'),        # Decompressed pointcloud
            }],
            output='screen'
        ),

        # rcpcc_compress node
        Node(
            package='rcpcc',
            executable='rcpcc_compress',
            name='rcpcc_compress',
            parameters=[ {'csv_folder_path': LaunchConfiguration('folder_path')},
                         {'pointcloud_topic': LaunchConfiguration('point_cloud_topic')},
                         {'q_level': LaunchConfiguration('compression_ratio')}],
        ),

        # rcpcc_decompress node
        Node(
            package='rcpcc',
            executable='rcpcc_decompress',
            name='rcpcc_decompress',
            parameters=[{'csv_folder_path': LaunchConfiguration('folder_path'),
                        'output_topic': LaunchConfiguration('output_topic')}]
        ),
    ])
