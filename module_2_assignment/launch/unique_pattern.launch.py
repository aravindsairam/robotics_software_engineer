from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    
    draw_logspiral_node = Node(
            package='module_2_assignment',
            executable='draw_logspiral',
            name='draw_logspiral')
    

    return LaunchDescription([
        turtlesim_node,
        draw_logspiral_node
    ])