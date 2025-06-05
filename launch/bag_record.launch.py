from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_bag_record_cmd(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag_path').perform(context)
    output_topic = LaunchConfiguration('output_topic')
    additional_topics_str = LaunchConfiguration('additional_topics').perform(context)
    
    # Split space-seperated additional topics string into a list
    additional_topics = additional_topics_str.split()
    
    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path, output_topic] + additional_topics,
            output='screen'
        )
    ]

def generate_launch_description():
    # Declare parameters for bag path and folder path
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='./my_bag',
        description='Path to save the ROS 2 bag file'
    )

    folder_path_arg = DeclareLaunchArgument(
        'folder_path',
        default_value='.',
        description='Folder path for rcpcc nodes data, folder must exist'
    )

    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/velodyne/velodyne_points',
        description='Point cloud topic to be subscribed'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/rcpcc_decompressed',
        description='Output decompressed point cloud topic'
    )

    # add parameter for compression ratio
    compression_mode_arg = DeclareLaunchArgument(
        'compression_ratio',
        default_value="2",
        description='Compression mode for rcpcc_compress node'
    )
    
    # add additional topics from original to include in output rosbag
    additional_topics_arg = DeclareLaunchArgument(
        'additional_topics',
        default_value='',
        description='Space-seperated list of additional topics to record into output bag'
    )

    return LaunchDescription([
        bag_path_arg,
        folder_path_arg,
        point_cloud_topic_arg,
        output_topic_arg,
        compression_mode_arg,
        additional_topics_arg,

        # Dynamically generate ros2 bag record command
        OpaqueFunction(function=generate_bag_record_cmd),

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
