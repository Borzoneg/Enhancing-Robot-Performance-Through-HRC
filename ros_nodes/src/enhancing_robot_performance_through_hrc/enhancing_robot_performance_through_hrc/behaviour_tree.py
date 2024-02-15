import py_trees as pt
import rclpy
from .behaviours import *
from custom_interfaces.srv import String

class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self):
        print("Initializing behavior tree")
        self.updating_bb_node = rclpy.create_node("updating_bb_node")
        self.updating_bb_node.create_service(String, "send_button_code", self.update_bb)
        
        self.behaviours_names = ["hold_left_sim", "hold_left_real", "reset_left", "place_left", 
                                 "hold_right_sim", "hold_right_real", "reset_right", "place_right",
                                 "hold_joint_sim", "hold_joint_real", "complete_task", "reset_task"]
        
        self.blackboard = py_trees.blackboard.Client(name="Blackboard_client")
        for behaviour in self.behaviours_names:
            self.blackboard.register_key(key=behaviour, access=py_trees.common.Access.WRITE)
            self.blackboard.set(behaviour, "not_done")
        print(self.blackboard)

        self.behaviours = []
        for behaviour in self.behaviours_names:
            self.behaviours.append(GenericBehaviour(behaviour, self.blackboard))

        self.left_placed = False
        self.right_placed = False

        root = pt.composites.Sequence(name = "Main sequence", memory=False)       
        root.add_children(self.behaviours)
        super(BehaviourTree, self).__init__(root)

    def update_bb(self, request, response):
        # possible request string: 
        #                         hold_left_sim
        #                         hold_left_real
        #                         place_left
        #                         reset_left
        #                         hold_right_sim
        #                         hold_right_real
        #                         place_right
        #                         reset_right
        #                         hold_joint_sim
        #                         hold_joint_real
        #                         reset_task
        #                         complete_task
        self.blackboard.set(request.data, "requested")
        response.ans = ""
        return response

def main(args=None):
    rclpy.init(args=args)    
    tree = BehaviourTree()
    ### render dot tree (.dot, .png, and .svg files) ###
    # pt.display.render_dot_tree(tree.root)
    print("\n"+pt.display.unicode_tree(root=tree.root,show_status=True))
    
    done = False
    while not done:
        rclpy.spin_once(tree.updating_bb_node)
        tree.tick()
        # print("\n"+pt.display.unicode_tree(root=tree.root,show_status=True))
        # print(tree.tip())
        # print(tree.root.status)
    rclpy.shutdown()

if __name__ == "__main__":
    main()