from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes_desc = LaunchDescription()
    log_file = {'log_file', '../../../../logs/thesis.log'}
    tree_node = Node(package='enhancing_robot_performance_through_hrc', executable='tree', arguments=["asd"])
    gui_node = Node(package='enhancing_robot_performance_through_hrc', executable='gui', arguments=["qwe"])
    srvs_node = Node(package='enhancing_robot_performance_through_hrc', executable='srvs')

    nodes_desc.add_action(tree_node)
    nodes_desc.add_action(gui_node)
    nodes_desc.add_action(srvs_node)

    return nodes_desc

def on_exit_callback(event):
    if event.process_return_code != 0:
        print(f"Node exited with error code {event.process_return_code}")
    else:
        print("Node exited gracefully")