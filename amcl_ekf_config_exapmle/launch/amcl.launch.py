import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Get package directories
package_dir = get_package_share_directory('tf_switch_node')
map_yaml_file = os.path.join(package_dir, 'amcl_ekf_config_exapmle', 'map', 'map12', 'map_1751196450.yaml')
params_file = os.path.join(package_dir, 'amcl_ekf_config_exapmle', 'parameter', 'amcl_params.yaml')

def generate_launch_description():   
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    amcl_launch = GroupAction(
        actions=[
        SetRemap(src='/tf', dst='/tf'),   
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,  'localization_launch.py')),
            launch_arguments={'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file
                              }.items()),
                              
    ])

# Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(amcl_launch)

    return ld