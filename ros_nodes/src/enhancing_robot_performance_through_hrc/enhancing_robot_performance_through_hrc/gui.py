import tkinter as tk
from tkinter import ttk

window = tk.Tk()

width = 700
height= window.winfo_screenheight()
x = window.winfo_screenwidth()-width
y = 0
#setting tkinter window size
window.geometry("%dx%d+%d+%d" % (width, height,x,y))
buttons_window = tk.Frame(window, bg='skyblue')
button1 = tk.Button(buttons_window, height= 4, width=20, highlightbackground="green", highlightthickness=5, text="New User", command=lambda: button_click(1),font=("Times New Roman",20),state="active")
label = tk.Label(text="ASD")
entry = tk.Entry()

label.pack()
entry.pack()
button1.pack()
buttons_window.pack()
window.mainloop()