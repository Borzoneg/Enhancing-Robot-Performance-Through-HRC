import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from send_int_clt import SendIntClient

class Gui(Node):
    def __init__(self):
        super().__init__('dmp_node')

        # self.send_button_clt = SendIntClient("send_button_code")
        self.buttons_dict = {'hold_left': 0, 'hold_right': 1, 'hold_joint': 2}
        self.x, self.y, self.z = 0, 0, 0
        self.roll , self.pitch, self.yaw = 0, 0, 0
        self.left_placed, self.right_placed = False, False

        self.window = tk.Tk()
        self.window.winfo_toplevel().title("CIM use case")
        self.configure_window()
        self.configure_frames()
        self.configure_labels()
        self.configure_buttons()
    
        self.window.mainloop()

    def configure_window(self):
        self.window.columnconfigure(0, weight=1, minsize=200)
        self.window.columnconfigure(1, weight=1, minsize=200)
        self.window.columnconfigure(2, weight=1, minsize=200)
        self.window.rowconfigure(0, weight=1)
        self.window.rowconfigure(1, weight=10, minsize=300)

    def configure_frames(self):
        self.left_window = tk.Frame(self.window, bg='grey')
        self.mid_window = tk.Frame(self.window, bg='grey')
        self.right_window = tk.Frame(self.window, bg='grey')

        self.left_window.grid(row=1, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
        self.mid_window.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        self.right_window.grid(row=1, column=2, sticky=tk.N+tk.S+tk.E+tk.W)

        self.left_window.rowconfigure(0, weight=1, minsize=100)
        self.left_window.rowconfigure(1, weight=3, minsize=100)
        self.left_window.rowconfigure(2, weight=1, minsize=100)
        self.left_window.columnconfigure(0, weight=1)
        self.left_window.columnconfigure(1, weight=3)

        self.right_window.rowconfigure(0, weight=1, minsize=100)
        self.right_window.rowconfigure(1, weight=3, minsize=100)
        self.right_window.rowconfigure(2, weight=1, minsize=100)
        self.right_window.columnconfigure(0, weight=3)
        self.right_window.columnconfigure(1, weight=1)

        self.mid_window.rowconfigure(0, weight=1, minsize=100)
        self.mid_window.rowconfigure(1, weight=3, minsize=100)
        self.mid_window.rowconfigure(2, weight=1, minsize=100)
        # self.mid_window.columnconfigure(0, weight=1)
        self.mid_window.columnconfigure(1, weight=3)
        # self.mid_window.columnconfigure(2, weight=1)

    def configure_buttons(self):
        self.hold_left_button = tk.Button(self.left_window, text="Hold left",
                                     command=lambda: self.on_click('hold_left'),
                                     font=("Times New Roman", 20), state="active")
        self.hold_joint_button = tk.Button(self.mid_window, text="Hold joint", 
                                      command=lambda: self.on_click('hold_joint'),
                                      font=("Times New Roman", 20), state="disabled")
        self.hold_right_button = tk.Button(self.right_window, text="Hold right",
                                      command=lambda: self.on_click('hold_right'),
                                      font=("Times New Roman", 20), state="active")
        
        self.hold_left_button.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        self.hold_joint_button.grid(sticky=tk.N+tk.S+tk.E+tk.W, row=1, column=1)
        self.hold_right_button.grid(row=1, column=0, sticky=tk.N+tk.S+tk.E+tk.W) 

    def configure_labels(self):
        label_str = f"X: {self.x:05.2f}, Y: {self.x:05.2f}, Z: {self.x:05.2f}, Roll: {self.x:05.2f}, Pitch: {self.x:05.2f}, Yaw: {self.x:05.2f}"
        self.label = tk.Label(self.window, text=label_str, font=("Times New Roman", 15))
        self.label.grid(row=0, column=0, columnspan=3)

    def on_click(self, button_label):
        # self.send_button_clt.send_request(self.buttons_dict[button_label])
        if button_label == 'hold_left':
            self.left_window.configure(bg="light blue")
            self.hold_left_button.configure(text="Piece placed", command=lambda: self.on_click('left_placed'))
        elif button_label == 'hold_right':
            self.right_window.configure(bg="light blue")
            self.hold_right_button.configure(text="Piece placed", command=lambda: self.on_click('right_placed'))
        
        elif button_label == 'left_placed':
            self.left_window.configure(bg="light green")
            self.hold_left_button.configure(text="Reset", command=lambda: self.on_click('left_reset'))
            self.left_placed = True
            if self.right_placed and self.left_placed:
                self.hold_joint_button.configure(state='active')
                self.hold_right_button.configure(state='disabled')
                self.hold_left_button.configure(state='disabled')
        elif button_label == 'right_placed':
            self.right_window.configure(bg="light green")
            self.hold_right_button.configure(text="Reset", command=lambda: self.on_click('right_reset'))
            self.right_placed = True
            if self.right_placed and self.left_placed:
                self.hold_joint_button.configure(state='active')
                self.hold_right_button.configure(state='disabled')
                self.hold_left_button.configure(state='disabled')

        elif button_label == 'left_reset':
            self.left_window.configure(bg="grey")
            self.hold_left_button.configure(text="Hold left", command=lambda: self.on_click('hold_left'))
            self.left_placed = False
        elif button_label == 'right_reset':
            self.right_window.configure(bg="grey")
            self.hold_right_button.configure(text="Hold right", command=lambda: self.on_click('hold_right'))
            self.right_placed = False

        elif button_label == 'hold_joint':
            self.mid_window.configure(bg="light blue")
            self.hold_joint_button.configure(text="Complete task", command=lambda: self.on_click('task_completed'))

        elif button_label == 'task_completed':
            self.left_window.configure(bg="grey")
            self.right_window.configure(bg="grey")
            self.mid_window.configure(bg="grey")
            self.hold_left_button.configure(text="Hold left", command=lambda: self.on_click('hold_left'), state="active")
            self.hold_right_button.configure(text="Hold right", command=lambda: self.on_click('hold_right'), state="active")
            self.hold_joint_button.configure(text="Hold joint", command=lambda: self.on_click('hold_joint'), state="disabled")
            self.right_placed = False
            self.left_placed = False


def main(args=None):
    rclpy.init(args=args)
    gui = Gui()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
