from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定義命令行參數 'robot_name'
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    # 使用 LaunchConfiguration 獲取 robot_name 參數
    robot_name = LaunchConfiguration('robot_name')

    # 創建brain_module節點
    brain_node = Node(
        package='brain_module',
        executable='brain_node',
        name='brain_node',
        output='screen',
        parameters=[{'robot_name': robot_name}],  # 使用 parameters 傳遞參數
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    # 創建vision_module節點
    vision_node = Node(
        package='vision_module',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{'robot_name': robot_name}],  # 使用 parameters 傳遞參數
        arguments=['--ros-args', '--log-level', 'ERROR']
        
    )

    # 創建memory_module節點
    memory_node = Node(
        package='memory_module',
        executable='memory_node',
        name='memory_node',
        output='screen',
        parameters=[{'robot_name': robot_name}],  # 使用 parameters 傳遞參數
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    # 創建robot_module節點
    robot_node = Node(
        package='robot_module',
        executable='robot_node',
        name='robot_node',
        output='screen',
        parameters=[{'robot_name': robot_name}],  # 使用 parameters 傳遞參數
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    # 創建world_module節點
    world_node = Node(
        package='world_module',
        executable='desktop_world_node',
        name='world_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    visualizer_node = Node(
        package='visualizer_module',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen', 
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    object_detection_node = Node(
        package='vision_module',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    object_detection_visualizer_node = Node(
        package='visualizer_module',
        executable='object_detection_visualizer_node',
        name='object_detection_visualizer_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    semantic_segmentation_node = Node(
        package='vision_module',
        executable='semantic_segmentation_node',
        name='semantic_segmentation_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    semantic_segmentation_visualizer_node = Node(
        package='visualizer_module',
        executable='semantic_segmentation_visualizer_node',
        name='semantic_segmentation_visualizer_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    # 將所有節點添加到LaunchDescription中
    return LaunchDescription([
        robot_name_arg,
        brain_node,
        vision_node,
        memory_node,
        robot_node,
        world_node,
        visualizer_node,
        object_detection_node,
        object_detection_visualizer_node,
        semantic_segmentation_node,
        semantic_segmentation_visualizer_node
    ])