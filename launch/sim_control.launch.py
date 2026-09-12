import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_control_pkg')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.world')
    drone_sdf = os.path.join(pkg_share, 'models', 'mini_drone', 'drone.sdf')
    models_path = os.path.join(pkg_share, 'models')
    
    # Crucial: Tells Gazebo where to find the mini_drone model and its textures
    set_gazebo_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
)
    
    return LaunchDescription([
        set_gazebo_model_path,
        
        # 1. Launch Gazebo with your custom world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),
        
        # 2. Spawn the Mini Drone model into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'mini_drone',
                '-file', drone_sdf,
                '-x', '0.0', '-y', '0.0', '-z', '0.5'
            ],
            output='screen'
        ),
        
        # 3. Launch the ArUco Tracker / Perception Node
        Node(
            package='drone_control_pkg',
            executable='aruco_tracker_node',
            name='aruco_tracker_node',
            output='screen'
        ),
        
        # 4. Launch your tuned PID Controller Node
        Node(
            package='drone_control_pkg',
            executable='controller_node',
            name='controller_node',
            output='screen'
        )
    ])