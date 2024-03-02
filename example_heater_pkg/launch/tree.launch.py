
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create an ArgumentParser to handle command-line arguments
    pkg_name = LaunchConfiguration('pkg_name')

    
    ld = LaunchDescription()
    
    monitor_node = Node(
        package=pkg_name,
        executable='monitor',
        name='monitor_node', 
        output='screen'
    )

    py_bt_node = Node(
        package=pkg_name,
        executable='bt_node.py',
        name='bt_node',
        output='screen'
    )
    
    ld.add_action(monitor_node)
    ld.add_action(py_bt_node)
    
    
    return ld