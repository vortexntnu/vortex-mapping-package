from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Launch configuration for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    
    # Package paths
    pkg = get_package_share_directory("blueye_nav")
    params_file = os.path.join(pkg, "config", "my_nav2_params.yaml")

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')


    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'params', 'my_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all nodes'
    )

    # Node for nav2 bringing up the navigation stack
    nav2_bringup_node = Node(
        package='nav2_bringup',
        executable='bringup',
        output='screen',
        parameters=[{'configured_params': params_file}],
        remappings=[
            ('/scan', '/point_cloud'),
            ('odom', 'odom'),
        ],
    )


    # Include launch description for navigation
    nav2_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    # 'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    # 'autostart': autostart,
                    'params_file': params_file,
                    # 'use_composition': use_composition,
                    # 'use_respawn': use_respawn,
                    # 'container_name': 'nav2_container',
                }.items()
            )
    

    slamtoolbox_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    # 'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    # 'autostart': autostart,
                    'params_file': params_file,
                    # 'use_composition': use_composition,
                    # 'use_respawn': use_respawn,
                    # 'container_name': 'nav2_container',
                }.items()
            )

    # Add actions to the LaunchDescription
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    # ld.add_action(nav2_bringup_node)
    # ld.add_action(nav2_include_launch)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_launch)
    

    return ld
