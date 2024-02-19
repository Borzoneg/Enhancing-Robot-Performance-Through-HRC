from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node

tree_node_desc = LaunchDescription([Node(package='enhancing_robot_performance_through_hrc', executable='tree', name='tree_node')])
gui_node_desc = LaunchDescription([Node(package='enhancing_robot_performance_through_hrc', executable='gui', name='gui_node')])

launch_service = LaunchService()
launch_service.include_launch_description(tree_node_desc)
launch_service.include_launch_description(gui_node_desc)
launch_service.run()