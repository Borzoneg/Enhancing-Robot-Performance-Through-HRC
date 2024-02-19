import py_trees as pt
import rclpy 
from .send_str_clt import SendStrClient

"""
=======================BEHAVIOURS FOR THE TREE=======================
hold_left_sim     hold_left_real      reset_left      place_left    |
hold_right_sim    hold_right_real     reset_right     place_right   |
hold_joint_sim    hold_joint_real     complete_task   reset_task    |
=====================================================================
"""
class GenericBehaviour(pt.behaviour.Behaviour):
    """
    Most of the behaviors are doing the same thing: they send a service requests, wait for answer and change the 
    blackboard accordingly the name of the behavior is also the name of the service 
    """
    def __init__(self, name, blackboard):
        super(GenericBehaviour, self).__init__(name)
        self.blackboard = blackboard
        self.ros_client = SendStrClient(name)

    def update(self):
        print("Inside", self.name)
        if self.blackboard.get(self.name) != "requested":
            print("Not supposed to run")
            return pt.common.Status.INVALID
        response = self.ros_client.send_request("")
        print(self.name, response)
        if response.ans == "success":
            print("inside success")
            return pt.common.Status.SUCCESS
        elif response.ans == "failure":
            return pt.common.Status.FAILURE
        
    def terminate(self, new_status):
        if new_status == pt.common.Status.SUCCESS:
            self.blackboard.set(self.name, "success")
        elif new_status == pt.common.Status.FAILURE:
            self.blackboard.set(self.name, "failure")


class ResetBehaviour(GenericBehaviour):
    def __init__(self, name, blackboard, reset_behaviours):
        super().__init__(name, blackboard)
        self.reset_behaviours = reset_behaviours
    def update(self):
        new_status = super().update()
        if new_status == pt.common.Status.SUCCESS:
            for behaviour in self.reset_behaviours:
                self.blackboard.set(behaviour.name, 'not_done')
        return pt.common.Status.INVALID
    

class SkippingBehaviour(GenericBehaviour):
    def __init__(self, name, blackboard, skip_behaviours):
        super().__init__(name, blackboard)
        self.skip_behaviours = skip_behaviours
    
    def update(self):
        new_status = super().update()
        if new_status == pt.common.Status.SUCCESS:
            for behaviour in self.skip_behaviours:
                behaviour.status = pt.common.Status.SUCCESS
        return new_status