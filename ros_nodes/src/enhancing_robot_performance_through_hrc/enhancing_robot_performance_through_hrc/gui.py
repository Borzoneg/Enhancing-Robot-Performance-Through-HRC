import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
# from .send_str_clt import SendStrClient

class Gui(Node):
    def __init__(self):
        super().__init__('dmp_node')
        # self.send_button_clt = SendStrClient("send_button_code")
        self.x, self.y, self.z = 0, 0, 0
        self.roll , self.pitch, self.yaw = 0, 0, 0
        self.left_placed, self.right_placed = False, False

        self.window = tk.Tk()
        self.window.winfo_toplevel().title("CIM use case")
        self.configure_window()
        self.configure_frames()
        self.configure_labels()
        self.configure_buttons()

        self.to_close = False
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()

    def configure_window(self):
        self.window.columnconfigure(0, weight=4, minsize=400)
        self.window.columnconfigure(1, weight=3, minsize=300)
        self.window.columnconfigure(2, weight=4, minsize=400)
        self.window.rowconfigure(0, weight=1, minsize=20)
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
        self.mid_window.columnconfigure(1, weight=1)

        self.left_hold_frame = tk.Frame(self.left_window, background="yellow")
        self.right_hold_frame = tk.Frame(self.right_window, background="yellow")
        self.left_hold_frame.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W, padx=10)
        self.right_hold_frame.grid(row=1, column=0, sticky=tk.N+tk.S+tk.E+tk.W, padx=10)
        self.left_hold_frame.columnconfigure(0, weight=1, minsize=200)
        self.left_hold_frame.columnconfigure(1, weight=1, minsize=200)
        self.left_hold_frame.rowconfigure(0, weight=1, minsize=20)
        self.right_hold_frame.columnconfigure(0, weight=1, minsize=200)
        self.right_hold_frame.columnconfigure(1, weight=1, minsize=200)
        self.right_hold_frame.rowconfigure(0, weight=1, minsize=20)
    

    def configure_buttons(self):
        self.reset_button = tk.Button(self.window, text="Reset task", font=("Times New Roman", 15), bg="light yellow", 
                                      command=lambda: self.reset_task()) 
        self.reset_button.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        self.hold_left_real_btn = tk.Button(self.left_hold_frame, text="Hold left real",
                                     command=lambda: self.hlr(),
                                     font=("Times New Roman", 20), state="active")
        self.hold_left_sim_btn = tk.Button(self.left_hold_frame, text="Hold left sim",
                                     command=lambda: self.hls(),
                                     font=("Times New Roman", 20), state="active")
        self.place_left_btn = tk.Button(self.left_hold_frame, text="Place left",
                                     command=lambda: self.pl(),
                                     font=("Times New Roman", 20), state="active")
        
        self.hold_right_sim_btn = tk.Button(self.right_hold_frame, text="Hold right sim",
                                     command=lambda: self.hrs(),
                                     font=("Times New Roman", 20), state="active")
        self.hold_right_real_btn = tk.Button(self.right_hold_frame, text="Hold right real",
                                     command=lambda: self.hrr(),
                                     font=("Times New Roman", 20), state="active")
        self.place_right_btn = tk.Button(self.right_hold_frame, text="Place right",
                                     command=lambda: self.pr(),
                                     font=("Times New Roman", 20), state="active")
        #TODO: joint has to be sim and real as well
        self.joint_button = tk.Button(self.mid_window, text="Hold joint", 
                                      command=lambda: self.hold_joint(),
                                      font=("Times New Roman", 20), state="disabled")
        
        
        self.hold_left_sim_btn.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
        self.hold_left_real_btn.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        self.hold_right_sim_btn.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
        self.hold_right_real_btn.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        self.place_left_btn.grid(row=0, column=0, columnspan=2, sticky=tk.N+tk.S+tk.E+tk.W)
        self.place_right_btn.grid(row=0, column=0, columnspan=2, sticky=tk.N+tk.S+tk.E+tk.W)
        self.place_left_btn.grid_remove()
        self.place_right_btn.grid_remove()
        self.joint_button.grid(sticky=tk.N+tk.S+tk.E+tk.W, row=1, column=1)

    def configure_labels(self):
        label_str = f"X: {self.x:05.2f}, Y: {self.x:05.2f}, Z: {self.x:05.2f}\nRoll: {self.x:05.2f}, Pitch: {self.x:05.2f}, Yaw: {self.x:05.2f}"
        self.label = tk.Label(self.window, text=label_str, font=("Times New Roman", 15), bg="light yellow")
        self.label.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)
        
        self.label = tk.Label(self.window, text="", font=("Times New Roman", 15), bg="light yellow")
        self.label.grid(row=0, column=2, sticky=tk.N+tk.S+tk.E+tk.W)

    def on_click(self, button_label):
        # self.send_button_clt.send_request(button_label)
        if button_label == 'hold_left_real':
            self.left_window.configure(bg="light blue")
            self.left_button.configure(text="Piece placed", command=lambda: self.on_click('place_left'))
            self.right_button.configure(state='disabled')
        elif button_label == 'hold_right_real':
            self.right_window.configure(bg="light blue")
            self.right_button.configure(text="Piece placed", command=lambda: self.on_click('place_right'))
            self.left_button.configure(state='disabled')
        
        
        elif button_label == 'place_left':
            self.left_window.configure(bg="light green")
            self.left_button.configure(text="Reset", command=lambda: self.on_click('reset_left'))
            self.right_button.configure(state='active')
            self.left_placed = True
            if self.right_placed and self.left_placed:
                self.joint_button.configure(state='active')
                self.right_button.configure(state='disabled')
                self.left_button.configure(state='disabled')
        elif button_label == 'place_right':
            self.right_window.configure(bg="light green")
            self.right_button.configure(text="Reset", command=lambda: self.on_click('reset_right'))
            self.left_button.configure(state='active')
            self.right_placed = True
            if self.right_placed and self.left_placed:
                self.joint_button.configure(state='active')
                self.right_button.configure(state='disabled')
                self.left_button.configure(state='disabled')

        elif button_label == 'reset_left':
            self.left_window.configure(bg="grey")
            self.left_button.configure(text="Hold left", command=lambda: self.on_click('hold_left_real'))
            self.left_placed = False
        elif button_label == 'reset_right':
            self.right_window.configure(bg="grey")
            self.right_button.configure(text="Hold right", command=lambda: self.on_click('hold_right_real'))
            self.right_placed = False

        elif button_label == 'hold_joint_real':
            self.mid_window.configure(bg="light blue")
            self.joint_button.configure(text="Complete task", command=lambda: self.on_click('complete_task'))

        elif button_label == 'complete_task':
            self.left_window.configure(bg="grey")
            self.right_window.configure(bg="grey")
            self.mid_window.configure(bg="grey")
            self.left_button.configure(text="Hold left", command=lambda: self.on_click('hold_left_real'), state="active")
            self.right_button.configure(text="Hold right", command=lambda: self.on_click('hold_right_real'), state="active")
            self.joint_button.configure(text="Hold joint", command=lambda: self.on_click('hold_joint_real'), state="disabled")
            self.right_placed = False
            self.left_placed = False
        
        elif button_label == 'reset_task':
            self.left_window.configure(bg="grey")
            self.right_window.configure(bg="grey")
            self.mid_window.configure(bg="grey")
            self.left_button.configure(text="Hold left", command=lambda: self.on_click('hold_left_real'), state="active")
            self.right_button.configure(text="Hold right", command=lambda: self.on_click('hold_right_real'), state="active")
            self.joint_button.configure(text="Hold joint", command=lambda: self.on_click('hold_joint_real'), state="disabled")
            self.right_placed = False
            self.left_placed = False

    def send_ros_request(self, ros_string):
        try:
            self.send_button_clt.send_request(ros_string)
        except AttributeError:
            print("ERROR: Client not initialized, can't send request")
        
    def hls(self):
        self.send_ros_request("hold_left_sim")
        self.left_window.configure(bg="light blue")
        self.hold_left_sim_btn.configure(state='disabled')
        self.hold_right_sim_btn.configure(state='disabled')
        self.hold_right_real_btn.configure(state='disabled')

    def hlr(self):
        self.send_ros_request("hold_left_real")
        self.left_window.configure(bg="blue")
        self.hold_left_real_btn.grid_remove()
        self.hold_left_sim_btn.grid_remove()
        self.hold_right_sim_btn.configure(state='disabled')
        self.hold_right_real_btn.configure(state='disabled')
        self.place_left_btn.grid()

    def pl(self):
        self.send_ros_request("place_left")
        self.left_window.configure(bg="light green")
        self.place_left_btn.configure(text="Reset", command=lambda: self.resetl())
        self.hold_right_sim_btn.configure(state='active')
        self.hold_right_real_btn.configure(state='active')
    
    def resetl(self, only_reset_gui=False):
        if not only_reset_gui:
            self.send_ros_request("reset_left")
        self.left_window.configure(bg="grey")
        self.place_left_btn.configure(text="Place left", command=lambda: self.pl())
        self.place_left_btn.grid_remove()
        self.hold_left_real_btn.grid()
        self.hold_left_sim_btn.grid()
        self.hold_left_sim_btn.configure(state='active')
            
    def hrs(self):
        self.send_ros_request("hold_right_sim")
        self.right_window.configure(bg="light blue")
        self.hold_right_sim_btn.configure(state='disabled')
        self.hold_left_sim_btn.configure(state='disabled')
        self.hold_left_real_btn.configure(state='disabled')

    def hrr(self):
        self.send_ros_request("hold_right_real")
        self.right_window.configure(bg="blue")
        self.hold_left_sim_btn.configure(state='disabled')
        self.hold_left_real_btn.configure(state='disabled')
        self.hold_right_real_btn.grid_remove()
        self.hold_right_sim_btn.grid_remove()
        self.place_right_btn.grid()
    
    def pr(self):
        self.send_ros_request("place_right")
        self.right_window.configure(bg="light green")
        self.place_right_btn.configure(text="Reset", command=lambda: self.resetr())
        self.hold_left_sim_btn.configure(state='active')
        self.hold_left_real_btn.configure(state='active')
    
    def resetr(self, only_reset_gui=False):
        if not only_reset_gui:
            self.send_ros_request("reset_right")
        self.right_window.configure(bg="grey")
        self.place_right_btn.configure(text="Place right", command=lambda: self.pr())
        self.place_right_btn.grid_remove()
        self.hold_right_real_btn.grid()
        self.hold_right_sim_btn.grid()
        self.hold_right_sim_btn.configure(state='active')

    def reset_task(self):
        self.send_ros_request("reset_task")
        self.resetl(only_reset_gui=True)
        self.resetr(only_reset_gui=True)
        self.hold_left_sim_btn.configure(state='active')
        self.hold_left_real_btn.configure(state='active')
        self.hold_right_sim_btn.configure(state='active')
        self.hold_right_real_btn.configure(state='active')
        
    def on_closing(self):
        self.send_ros_request("quit")
        self.window.destroy()
        self.to_close = True
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui = Gui()
    # while not gui.to_close:
    #     rclpy.spin_once(gui)
    #     gui.window.mainloop()
    # gui.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
