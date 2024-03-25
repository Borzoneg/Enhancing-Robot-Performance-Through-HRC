import py_trees as pt
class Behaviour(pt.behaviour.Behaviour):
    """
    Most of the behaviors are doing the same thing: they send a service requests, wait for answer and change the 
    blackboard accordingly the name of the behavior is also the name of the service 
    """
    def __init__(self, name, success=True):
        super(Behaviour, self).__init__(name)
        self.succ = success

    def update(self):
        if self.succ:
            new_status = pt.common.Status.SUCCESS
        else:
            new_status = pt.common.Status.FAILURE
        print(self.name, ": update step", self.status)
        return new_status

    def terminate(self, new_status):
        self.status = new_status


class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self):
        
        behaviour_1 = Behaviour("behaviour_1")
        behaviour_2 = Behaviour("behaviour_2")
        behaviour_3 = Behaviour("behaviour_3", False)

        root = pt.composites.Parallel(name = "main", 
                                      policy=pt.common.ParallelPolicy.SuccessOnSelected([behaviour_1]),
                                      children=[behaviour_1, behaviour_2, behaviour_3])
        # root = pt.composites.Sequence(name="placing_joint", memory=True,
        #                                        children=[behaviour_1, behaviour_2, behaviour_3])
        super(BehaviourTree, self).__init__(root)

def main(args=None):
    tree = BehaviourTree()
    while True:
        print("tree tick")
        tree.tick()
        if tree.root.status is pt.common.Status.SUCCESS:
            break

if __name__ == "__main__":
    main()