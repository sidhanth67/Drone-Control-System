import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    ament_prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
        prefix for prefix in ament_prefixes
        if not os.path.basename(prefix).startswith('sjtu_drone_')
    )

    pkg_share = get_package_share_directory('drone_control_pkg')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.world')
    drone_sdf = os.path.join(pkg_share, 'models', 'mini_drone', 'drone.sdf')
    models_path = os.path.join(pkg_share, 'models')
    
    
    set_gazebo_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
)

    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mini_drone',
            '-file', drone_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.05'
        ],
        output='screen'
    )

    def start_after_spawn(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Drone spawn failed; controller will not start.')]

        return [
            Node(
                package='drone_control_pkg',
                executable='aruco_tracker_node',
                name='aruco_tracker_node',
                output='screen'
            ),
            Node(
                package='drone_control_pkg',
                executable='controller_node',
                name='controller_node',
                parameters=[{'wait_for_goal': True}],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'sim_control.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
        ]

    return LaunchDescription([
        set_gazebo_model_path,
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),
        
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_drone,
                on_exit=start_after_spawn,
            )
        ),
        spawn_drone,
    ])
