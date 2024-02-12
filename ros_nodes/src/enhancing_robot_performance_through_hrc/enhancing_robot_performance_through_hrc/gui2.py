import tkinter as tk
import rclpy
from rclpy.node import Node
from send_int_clt import SendIntClient 

class Gui(Node):
    def __init__(self):
        super().__init__('dmp_node')

        # self.send_button_clt = SendIntClient("send_button_code")
        self.buttons_dict = {'hold_left': 0, 'hold_right': 1}

        root = tk.Tk()
        tk.Grid.rowconfigure(root, 0, weight=1)
        tk.Grid.columnconfigure(root, 0, weight=1)

        #Create & Configure frame 
        frame=tk.Frame(root)
        frame.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        #Create a 5x10 (rows x columns) grid of buttons inside the frame
        for row_index in range(3):
            tk.Grid.rowconfigure(frame, row_index, weight=1)
            for col_index in range(5):
                tk.Grid.columnconfigure(frame, col_index, weight=1)
                _ = tk.Frame(frame, bg='grey', width=100, height=100) #create a button inside frame 
                _.grid(row=row_index, column=col_index, sticky=tk.N+tk.S+tk.E+tk.W)  
        hold_left_btn = tk.Button(frame, text="Hold left", command=lambda: self.on_click('hold_left'),font=("Times New Roman", 20), state="active")
        hold_left_btn.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        hold_right_btn = tk.Button(frame, text="Hold right", command=lambda: self.on_click('hold_right'),font=("Times New Roman", 20), state="active")
        hold_right_btn.grid(row=1, column=2, sticky=tk.N+tk.S+tk.E+tk.W)
        hold_mid_btn = tk.Button(frame, text="Hold plaque", command=lambda: self.on_click('hold_plaque'),font=("Times New Roman", 20), state="active")
        hold_mid_btn.grid(row=1, column=3, sticky=tk.N+tk.S+tk.E+tk.W)

        root.mainloop()

    def on_click(self, button_label):
        pass

def main(args=None):
    rclpy.init(args=args)
    gui = Gui()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    self.up = tk.Button(self.left_window, text="U", 
                                     #width=6,
                                     #height=3,
                                     
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        self.down = tk.Button(self.left_window, text="D", 
                                     #width=6,
                                     #height=3,
                                     
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        self.left = tk.Button(self.left_window, text="L", 
                                     #width=6,
                                     #height=3,
                                     
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        self.right = tk.Button(self.left_window, text="R", 
                                     #width=6,
                                     #height=3,
                                     
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        self.teach = tk.Button(self.left_window, text="TEACH", 
                                     #width=6,
                                     #height=3,
                                     
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        