import launch
import launch_ros

def generate_launch_description():
    # 产生 launch description 对象

    # 创建 action 动作
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        
    )
    # 不太对
    action_node_turtle_teleop_key = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='turtle_teleop_key',
        output='log'
        
    )
    # action_node_face_detect_node = launch_ros.actions.Node(
    #     package='demo_python_service',
    #     executable='face_detect_node',
    #     name='face_detect_node',
        
    # )

    return launch.LaunchDescription([
        # action 动作
        action_node_turtlesim_node,
        action_node_turtle_teleop_key,
        # action_node_face_detect_node    
    ])

