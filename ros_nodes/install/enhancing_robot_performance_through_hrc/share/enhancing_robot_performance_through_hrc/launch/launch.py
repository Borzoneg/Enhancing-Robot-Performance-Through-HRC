from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes_desc = LaunchDescription()

    tree_node = Node(package='enhancing_robot_performance_through_hrc', executable='tree')
    gui_node = Node(package='enhancing_robot_performance_through_hrc', executable='gui')
    srvs_node = Node(package='enhancing_robot_performance_through_hrc', executable='srvs')

    nodes_desc.add_action(tree_node)
    nodes_desc.add_action(gui_node)
    nodes_desc.add_action(srvs_node)

    return nodes_desc