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
    def __init__(self, name, blackboard, logger):
        super(GenericBehaviour, self).__init__(name)
        self.logger = logger
        self.logger.info("Init behaviour: " + self.name)
        self.blackboard = blackboard
        self.ros_client = SendStrClient(name)

    def update(self):
        self.logger.info("Update step for behaviour: " + self.name)
        if self.blackboard.get(self.name) == "requested":    
            response = self.ros_client.send_request("")
            self.logger.info(self.name + " response: " + response.ans)
            if response.ans == "success":
                return pt.common.Status.SUCCESS
            elif response.ans == "failure":
                return pt.common.Status.FAILURE
        else:
            pass
            # self.logger.debug("Not supposed to run " + self.name)
            # return self.status
        
    def terminate(self, new_status):
        self.logger.info("Terminating: " + self.name + " with status: " + str(new_status))
        if new_status == pt.common.Status.SUCCESS:
            self.status = new_status
            self.blackboard.set(self.name, "success")
        elif new_status == pt.common.Status.FAILURE:
            self.status = new_status
            self.blackboard.set(self.name, "failure")
        elif new_status == pt.common.Status.RUNNING:
            self.status = new_status
            self.blackboard.set(self.name, "running")


class ResetBehaviour(GenericBehaviour):
    def __init__(self, name, blackboard, logger, reset_behaviours):
        super().__init__(name, blackboard, logger)
        self.logger.info("This behaviour reset: " + str([behaviour.name for behaviour in reset_behaviours]))
        self.reset_behaviours = reset_behaviours
    def update(self):
        new_status = super().update()
        if new_status == pt.common.Status.SUCCESS:
            for behaviour in self.reset_behaviours:
                self.logger.info("Reseting: " + behaviour.name)
                self.blackboard.set(behaviour.name, 'not_done')
        return pt.common.Status.INVALID
    

class HoldBehaviour(GenericBehaviour):
    def update(self):
        self.logger.info("Update step for behaviour: " + self.name)
        if self.blackboard.get(self.name) == "requested":    
            response = self.ros_client.send_request("")
            self.logger.info(self.name + " response: " + response.ans)
            if response.ans == "success":
                return pt.common.Status.RUNNING
            elif response.ans == "failure":
                return pt.common.Status.FAILURE
        elif self.blackboard.get(self.name) == "running":
            part_position_str = self.name.split('_')[1] # from the name we can find witch object are we using left or right
            # if placing has been requested, the user does not want the robot to hold onto the part no more, the holdin action is complete
            if self.blackboard.get("place_" + part_position_str) == "requested": 
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.RUNNING
        else:
            pass
            # self.logger.debug("Not supposed to run " + self.name)
            # return self.status