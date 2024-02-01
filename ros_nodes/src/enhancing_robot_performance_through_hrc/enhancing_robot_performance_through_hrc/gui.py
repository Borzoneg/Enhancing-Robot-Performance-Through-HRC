import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from send_int_clt import SendIntClient

class Gui(Node):
    def __init__(self):
        super().__init__('dmp_node')

        # self.send_button_clt = SendIntClient("send_button_code")
        self.buttons_dict = {'hold_left': 0, 'hold_right': 1}

        self.window = tk.Tk()
        
        # width, height = self.window.winfo_screenwidth(), self.window.winfo_screenheight()
        width, height = 1850, 1136
        x, y = 1000, 0
        n_rows, n_cols = 5, 5   
        # self.window.geometry(f"{width}x{height}+{x}+{y}")
        buttons_window = tk.Frame(self.window, bg='grey', height=100, width=200)
        hold_left_button = tk.Button(self.window, text="Hold left", 
                                     width=20,
                                     height=20,
                                     highlightbackground="green", highlightthickness=5,
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        hold_right_button = tk.Button(self.window, text="Hold right", 
                                      width=20,
                                      height=20,
                                      highlightbackground="green", highlightthickness=5,
                                      command=lambda: self.on_click('hold_right'),
                                      font=("Times New Roman", 20), state="active")
        
        # label = tk.Label(text="ASD")
        
        # label.grid(column=2, row=1)
        buttons_window.place((10,10))
        # buttons_window.grid(column=0, row=0)
        hold_left_button.grid(column=1, row=2)
        hold_right_button.grid(column=3, row=2)
        
        # hold_left_button.grid(column=3, row=1)
        # hold_left_button.grid(column=4, row=1)
        # hold_left_button.grid(column=5, row=1)
        # hold_right_button.grid(column=2, row=2)
        # rows_size, cols_size = self.window.winfo_height()//n_rows, self.window.winfo_width()//n_cols
        # print(self.window.winfo_height(), self.window.winfo_width())
        # for i in range(n_cols):
        #     self.window.columnconfigure(i, weight=1, minsize=200)
        # for i in range(n_rows):
        #     self.window.rowconfigure(i, weight=1, minsize=200)
        
        self.window.mainloop()

    def on_click(self, button_label):
        print(self.window.winfo_width(), self.window.winfo_height())
        # self.send_button_clt.send_request(self.buttons_dict[button_label])

def main(args=None):
    rclpy.init(args=args)
    gui = Gui()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
