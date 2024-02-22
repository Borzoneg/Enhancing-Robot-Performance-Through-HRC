import py_trees as pt
import rclpy
from .behaviours import *
from custom_interfaces.srv import String
from .send_str_clt import SendStrClient
from .log_manager import CustomLogger
import os
import re

class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self):
        self.logger = CustomLogger("Tree", os.path.expanduser('~') + 
                                   "/.local/share/ov/pkg/isaac_sim-2023.1.1/Enhancing-Robot-Performance-Through-HRC/logs/tree.log",
                                   overwrite=True)
        self.logger.info("="*10 + "Starting init behavior tree" + "="*10)
        
        self.updating_bb_node = rclpy.create_node("updating_bb_node")
        self.updating_bb_node.create_service(String, "send_button_code", self.update_bb)

        self.logger.info("Created service send_button_code")
        
        self.generic_behaviours_names = ["place_left", "place_right", "place_joint"]
        self.hold_behaviours_names = ["hold_left_sim",   "hold_left_real",
                                      "hold_right_sim",  "hold_right_real",
                                      "hold_joint_sim",  "hold_joint_real"]
        self.reset_behaviours_name = {"reset_left": ["hold_left_sim", "hold_left_real",],
                                      "reset_right":["hold_right_sim", "hold_right_real"],
                                      "reset_task": ["hold_left_sim", "hold_left_real", "place_left", 
                                                     "hold_right_sim", "hold_right_real", "place_left", 
                                                     "hold_joint_sim", "hold_joint_real"]}
        self.blackboard = pt.blackboard.Client(name="Blackboard_client")
        self.logger.info("Creating blackboard")
        for behaviour_name in self.generic_behaviours_names + self.hold_behaviours_names + list(self.reset_behaviours_name.keys()):
            self.blackboard.register_key(key=behaviour_name, access=pt.common.Access.WRITE)
            self.blackboard.set(behaviour_name, "not_done")

        self.behaviours = {}
        for behaviour_name in self.generic_behaviours_names:
            self.behaviours[behaviour_name] = GenericBehaviour(behaviour_name, self.blackboard, self.logger)
        
        for behaviour_name in self.hold_behaviours_names:
            self.behaviours[behaviour_name] = HoldBehaviour(behaviour_name, self.blackboard, self.logger)

        for behaviour_name in self.reset_behaviours_name:
            self.behaviours[behaviour_name] = ResetBehaviour(behaviour_name, self.blackboard, self.logger, self.gen_list_behaviours(self.reset_behaviours_name[behaviour_name]))
        
        hold_left = pt.composites.Parallel(name="hold_left",
                                           policy=pt.common.ParallelPolicy.SuccessOnSelected([self.behaviours['hold_left_real']]),
                                           children=[self.behaviours['hold_left_sim'], self.behaviours['hold_left_real']])
        placing_left = pt.composites.Sequence(name="placing_left", memory=True, 
                                              children=[hold_left, self.behaviours['place_left']])
        hold_right = pt.composites.Parallel(name="hold_right",
                                            policy=pt.common.ParallelPolicy.SuccessOnSelected([self.behaviours['hold_right_real']]),
                                            children=[self.behaviours['hold_right_sim'], self.behaviours['hold_right_real']])
        placing_right = pt.composites.Sequence(name="placing_right", memory=True, 
                                              children=[hold_right, self.behaviours['place_right']])
        placing_parts = pt.composites.Parallel(name="placing_parts", 
                                               policy=pt.common.ParallelPolicy.SuccessOnSelected([placing_left, placing_right]),
                                               children=[placing_left, self.behaviours['reset_left'], placing_right, self.behaviours['reset_right']])
        hold_joint = pt.composites.Parallel(name="hold_joint",
                                            policy=pt.common.ParallelPolicy.SuccessOnSelected([self.behaviours['hold_joint_real']]),
                                            children=[self.behaviours['hold_joint_sim'], self.behaviours['hold_joint_real']])
        placing_joint = pt.composites.Sequence(name="placing_joint", memory=True,
                                               children=[hold_joint, self.behaviours['place_joint']])
        task = pt.composites.Sequence(name="task", memory=True,
                                      children=[placing_parts, placing_joint])
        root = pt.composites.Parallel(name = "main", 
                                      policy=pt.common.ParallelPolicy.SuccessOnSelected([task]),
                                      children=[task, self.behaviours['reset_task']])       
        self.logger.info("Created tree")
        
        self.quit_request = False
        super(BehaviourTree, self).__init__(root)
        self.logger.info("="*10 + "Ending tree init" + "="*10)

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
        self.logger.info("Received new request from gui: " + request.data)
        if request.data == "quit":
            self.quit_request = True
            response.ans = ""
            self.logger.info("Quit request: " + str(self.quit_request))
            return response
        self.blackboard.set(request.data, "requested")
        self.logger.info(request.data + " satus changed to request")
        response.ans = ""
        return response

    def gen_list_behaviours(self, behaviour_names):
        """
        generate a list of behvaiours from a list of names, the function assume the behaviours is contained in the 
        self.behaviours dictionary
        """
        behaviour_list = []
        for behaviour_name in behaviour_names:
            behaviour_list.append(self.behaviours[behaviour_name])
        return behaviour_list
    
    def get_clean_tree_string(self):
        unicode_tree = pt.display.unicode_tree(root=self.root,show_status=True)
        clean_tree = re.sub(r'.\[\dm', '', unicode_tree)
        clean_tree = re.sub(r'.\[\d\dm', '', clean_tree)        
        return clean_tree

    def get_clean_blackboard_string(self):
        black_board_str = str(self.blackboard)
        clean_bb = re.sub(r'.\[0m', '', black_board_str)
        clean_bb = re.sub(r'.\[\d\dm', '', clean_bb)
        return clean_bb


def main(args=None):
    rclpy.init(args=args)    
    tree = BehaviourTree()
    tree.logger.info("Behaviour tree:\n" + tree.get_clean_tree_string())
    tree.logger.info("Behaviour tree blackboard:\n" + tree.get_clean_blackboard_string())
    tree.logger.info("Entering main loop main tree")
    while not tree.quit_request:
        tree.logger.info("Before spinning ros node")
        rclpy.spin_once(tree.updating_bb_node)
        tree.logger.info("Before tree tick")
        tree.tick()    
        tree.logger.info("Tree tip: " + str(tree.tip()))
        tree.logger.info("Behaviour tree:\n" + tree.get_clean_tree_string())
        tree.logger.info("Behaviour tree blackboard:\n" + tree.get_clean_blackboard_string())
    tree.logger.info("Come out of main loop")
    rclpy.shutdown()
    tree.logger.info("Turning off")

if __name__ == "__main__":
    main()