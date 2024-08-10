from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    
    turtlesim_1 = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim1'
        )  
    
    spawn_turtle_2 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}\""],
            name='spawn_turtle2',
            shell=True
        )

    change_turtle_speed_1 = Node(
            package='module_2_assignment',
            executable='change_speed',
            name='speed_change_1',
            parameters=[{"linear_speed": 0.0,
                          "angular_speed": 0.0}]
        )
    
    change_turtle_speed_2 = Node(
            package='module_2_assignment',
            executable='change_speed',
            name='speed_change_2',
            parameters=[{"cmd_vel_topic": "/turtle2/cmd_vel",
                         "linear_speed": 0.0,
                          "angular_speed": 0.0}]
        )

    return LaunchDescription([
        turtlesim_1,
        spawn_turtle_2,
        change_turtle_speed_1,
        change_turtle_speed_2,
    ])