import py_trees
import rclpy 
from .send_str_clt import SendStrClient

class GenericBehaviour(py_trees.behaviour.Behaviour):
    """
    All the behaviors are doing the same thing: they send a service requesta, wait for answer and change the blackboard 
    accordingly the name of the behavior is also the name of the service 
    """
    def __init__(self, name, blackboard):
        super(GenericBehaviour, self).__init__(name)
        self.blackboard = blackboard
        self.ros_client = SendStrClient(name)

    def update(self):
        response = self.ros_client.send_request("_")
        print(self.name, response)
        if response.ans == "success":
            return py_trees.common.Status.SUCCESS
        elif response.ans == "failure":
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.blackboard.hold_first_sim = "success" 
        elif new_status == py_trees.common.Status.FAILURE:
            self.blackboard.hold_first_sim = "failure" 