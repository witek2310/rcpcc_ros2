from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    # add parameter for compression ratio
    compression_mode_arg = DeclareLaunchArgument(
        'compression_ratio',
        default_value="2",
        description='Compression mode for rcpcc_compress node'
    )

    return LaunchDescription([
        bag_path_arg,
        folder_path_arg,
        point_cloud_topic_arg,
        compression_mode_arg,

        # ros2 bag record process
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_path'), '/rcpcc_decompressed',
            ],
            output='screen'
        ),

        Node(
            package='snnrmse',
            executable='snnrmse',
            name='snnrmse',
            output='screen',
            parameters=[
                {'original_topic': LaunchConfiguration('point_cloud_topic')},
                {'decompressed_topic': '/rcpcc_decompressed'},
                {'output_folder': LaunchConfiguration("folder_path")}  # or any desired path
            ]
        ),

        # rcpcc_compress node
        Node(
            package='rcpcc',
            executable='rcpcc_compress',
            name='rcpcc_compress',
            parameters=[{'csv_folder_path': LaunchConfiguration('folder_path')},
                         {'pointcloud_topic': LaunchConfiguration('point_cloud_topic')},
                         {'q_level': LaunchConfiguration('compression_ratio')}],
        ),

        # rcpcc_decompress node
        Node(
            package='rcpcc',
            executable='rcpcc_decompress',
            name='rcpcc_decompress',
            parameters=[{'csv_folder_path': LaunchConfiguration('folder_path')}]
        ),


    ])