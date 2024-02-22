from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node

def main(args=None):
    nodes_desc = LaunchDescription()

    tree_node = Node(package='enhancing_robot_performance_through_hrc', executable='tree', name='tree_node')
    gui_node = Node(package='enhancing_robot_performance_through_hrc', executable='gui', name='gui_node')

    nodes_desc.add_action(tree_node)
    nodes_desc.add_action(gui_node)


    launch_service = LaunchService()
    launch_service.include_launch_description(nodes_desc)
    launch_service.run()

if __name__ == "__main__":
    main()