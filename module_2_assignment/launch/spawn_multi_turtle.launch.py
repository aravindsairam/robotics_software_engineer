from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    
    turtlesim_1 = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim1'
        )  

    kill_turtle_1 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', "\"{name: 'turtle1'}\""],
            name='kill_turtle1',
            shell=True
        ) 
    
    spawn_turtle_1 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle1'}\""],
            name='spawn_turtle1',
            shell=True
        )
    
    spawn_turtle_2 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 4.0, y: 4.0, theta: 0.0, name: 'turtle2'}\""],
            name='spawn_turtle2',
            shell=True
        )
    
    spawn_turtle_3 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 6.0, y: 6.0, theta: 0.0, name: 'turtle3'}\""],
            name='spawn_turtle3',
            shell=True
        )
    
    spawn_turtle_4 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 8.0, y: 8.0, theta: 0.0, name: 'turtle4'}\""],
            name='spawn_turtle4',
            shell=True
        )
    
    spawn_turtle_5 = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 10.0, y: 10.0, theta: 0.0, name: 'turtle5'}\""],
            name='spawn_turtle5',
            shell=True
        )

    turtle_driver_2 = Node(
            package='module_2_assignment',
            executable='drive_robot',
            name='drive_robot_2',
            parameters=[{"cmd_vel_topic": "/turtle2/cmd_vel"}]
        )
    
    turtle_driver_3 = Node(
            package='module_2_assignment',
            executable='drive_robot',
            name='drive_robot_3',
            parameters=[{"cmd_vel_topic": "/turtle3/cmd_vel"}]
        )
    
    turtle_driver_4 = Node(
            package='module_2_assignment',
            executable='drive_robot',
            name='drive_robot_4',
            parameters=[{"cmd_vel_topic": "/turtle4/cmd_vel"}]
        )


    return LaunchDescription([
        turtlesim_1,
        kill_turtle_1,
        spawn_turtle_2,
        spawn_turtle_3,
        spawn_turtle_4,
        spawn_turtle_5,
        spawn_turtle_1,
        turtle_driver_2,
        turtle_driver_3,
        turtle_driver_4
    ])