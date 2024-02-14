import py_trees as pt
import rclpy
from behaviours import *

class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self):
        print("Initializing behavior tree")

        self.hold_first = HoldFirst()
        
        root = pt.composites.Sequence(name = "Main sequence", memory=False)
        
        root.add_children([self.hold_first, 
                           ])  
        super(BehaviourTree, self).__init__(root)

def main(args=None):
    rclpy.init(args=args)    
    
    tree = BehaviourTree()

    ### render dot tree (.dot, .png, and .svg files) ###
    pt.display.render_dot_tree(tree.root)
    
    print("\n"+pt.display.unicode_tree(root=tree.root,show_status=True))
    
    done = False
    while not done:

        tree.tick()
        print("\n"+pt.display.unicode_tree(root=tree.root,show_status=True))
        print(tree.tip())
        print(tree.root.status)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()