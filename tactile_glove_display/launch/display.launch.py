from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    tactile_glove_display_path = FindPackageShare('tactile_glove_display')
    default_model_path = PathJoinSubstitution(['urdf', 'puthand-standalone.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([tactile_glove_display_path, 'rviz', 'hand.rviz'])

    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    ld.add_action(gui_arg)

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    ld.add_action(rviz_arg)

    ld.add_action(DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Path to robot urdf file relative to urdf_tutorial package'
    ))
    
    ld.add_action(Node(
        package='tactile_glove_markers',
        executable='markers_sphere_publisher', 
        output='screen',
    ))
    
    ld.add_action(Node(
        package='tactile_glove_markers',
        executable='markers_text_publisher', 
        output='screen',
    ))
    
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'tactile_glove_display',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')
        }.items()
    ))

    return ld

