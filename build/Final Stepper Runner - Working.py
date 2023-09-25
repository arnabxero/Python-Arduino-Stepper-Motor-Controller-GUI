###################################
# Stepper Motor Pin Configuration:
# CLK+ = 8
# CW+ = 9
###################################
from pathlib import Path
from PIL import Image, ImageTk
import os
import pyfirmata
import re
import tkinter as tk
from tkinter import END, ttk
import platform
import threading
import time
import csv
import datetime
import math

from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage

system = platform.system()

code_path = os.path.abspath(__file__)
assets_relative_path = os.path.join(
    os.path.dirname(code_path), "assets/frame0")
ASSETS_PATH = Path(assets_relative_path)


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)


window = Tk()
window.geometry("650x400")
window.configure(bg="#FFFFFF")
canvas = Canvas(
    window,
    bg="#FFFFFF",
    height=400,
    width=650,
    bd=0,
    highlightthickness=0,
    relief="ridge"
)
canvas.place(x=0, y=0)


def exit_gui():
    os._exit(0)


def update_display_string(new_string):

    global display_string, entry_4
    display_string = str(new_string)
    entry_4.delete(0, tk.END)
    entry_4.insert(0, display_string)

# Auto Arduino board and port selection


def boardSetup():
    global board, pin_step_active, pin_step_direction
    ######################## Arduino Setup ########################
    lines = []

    if system == "Darwin":
        print("Mac OS Detected, finding ports")
        update_display_string("Mac OS detected, finding ports")
        output = os.popen('ls /dev/tty.*').read()
    elif system == "Windows":
        print("Windows Detected, finding ports")
        update_display_string("Windows detected, finding ports")
        output = os.popen('wmic path Win32_SerialPort').read()
    else:
        update_display_string("Unknown operating system")
        print("Unknown operating system")

    output = os.popen('ls /dev/tty.*').read()

    for substr in output.split('\n'):
        substr2 = re.sub('\x1b\[[0-9;]*m', '', substr)
        lines.append(substr2)
        print(substr2)
        update_display_string(substr2)
    lines.pop()

    def_index = 0
    for i, line in enumerate(lines):
        if "usb" in line:
            def_index = i
            break

    default_port_name = lines[def_index]
    print("Default Port: " + default_port_name)
    update_display_string(default_port_name)

    try:
        board = pyfirmata.Arduino(default_port_name)
        it = pyfirmata.util.Iterator(board)
        it.start()
        board.digital[pin_step_direction].mode = pyfirmata.OUTPUT
        board.digital[pin_step_active].mode = pyfirmata.OUTPUT
        print(pin_step_direction)
        print(pin_step_active)
    except:
        print("Arduino Not Found")
        update_display_string("Arduino Not Found")
    ######################## Arduino Setup ########################


# function to set values
def setGlobals(cur_speed, cur_run_time):
    global speed, run_time
    speed = float(cur_speed)
    run_time = float(cur_run_time)
    print("New Speed: " + str(speed))
    print("New Run Time: " + str(run_time))
    update_display_string("Speed: " + str(speed) +
                          " | Run Time: " + str(run_time)+"Seconds")


def setMotorActive(stat):
    global motor_active

    if stat:
        motor_active = True
    else:
        motor_active = False

    print("Setting motor active: " + str(motor_active))
    update_display_string("Setting motor active: " + str(motor_active))


def calculate_custom_delay(speed):
    if speed < 1:
        custom_delay = 1.0
    else:
        custom_delay = 1.0 / speed
    return custom_delay


def moveMotor():

    global board, pin_step_active, pin_step_direction, direction

    try:
        if (direction):  # If direction is true, set the stepper's direction to 1 or forward
            board.digital[pin_step_direction].write(int(0))
        else:  # If direction is false, set the stepper's direction to 0 or backwards
            board.digital[pin_step_direction].write(int(1))

        # Make the stepper motor activation pin active for 0.0001 seconds
        board.digital[pin_step_active].write(int(1))
        time.sleep(.0001)

        # Make the stepper motor activation pin inactive again
        board.digital[pin_step_active].write(int(0))
    except:
        print("Unable to write to Arduino board")
        # update_display_string("Unable to write to Arduino board")


def runMainThread():
    global speed, run_time, direction, active_run_time, motor_active

    while True:
        custom_delay = calculate_custom_delay(speed)

        if (motor_active):
            print("Stepper is active")
            active_run_time = active_run_time + custom_delay

            if (active_run_time > run_time):
                update_display_string("Runtime Reached")
                motor_active = False
                active_run_time = 0
                continue

            update_display_string(f"Stepper is active: {active_run_time:.3f}")
            moveMotor()
        else:
            continue

        time.sleep(custom_delay)


# global variables
board = None

speed = 1
run_time = 10
direction = True
active_run_time = 0
motor_active = False

pin_step_direction = 9
pin_step_active = 8

display_string = "Ready to run stepper motor"


entry_1 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_1.insert(0, speed)
entry_1.place(
    x=151.0,
    y=139+30.0,
    width=136.0,
    height=44.0
)


entry_2 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_2.insert(0, run_time)
entry_2.place(
    x=363.0,
    y=139+30.0,
    width=136.0,
    height=44.0
)

canvas.create_text(
    380.0,
    127+30.0,
    anchor="nw",
    text="RUN TIME (Seconds)",
    fill="#3B3D70",
    font=("Inter SemiBold", 10 * -1)
)

entry_3 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_3.insert(0, active_run_time)
entry_3.place(
    x=257.0,
    y=59+35.0,
    width=136.0,
    height=44.0
)

entry_4 = Entry(
    bd=0,
    bg="#dbdbdb",
    fg="#4f4f4f",
    highlightthickness=0,
    justify="center",

)
entry_4.insert(0, display_string)
entry_4.place(
    x=150.0,
    y=40.0,
    width=350.0,
    height=35.0
)

canvas.create_text(
    265.0,
    47+35.0,
    anchor="nw",
    text="RUNNING FOR (Seconds)",
    fill="#3B3D70",
    font=("Inter SemiBold", 10 * -1)
)

button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setMotorActive(True),
    relief="flat"
)
button_1.place(
    x=151.0,
    y=211+30.0,
    width=136.0,
    height=46.0
)

button_image_2 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_2 = Button(
    image=button_image_2,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: exit_gui(),
    relief="flat"
)
button_2.place(
    x=554.0,
    y=354.0,
    width=96.12933349609375,
    height=46.0
)

button_image_3 = PhotoImage(
    file=relative_to_assets("button_3.png"))
button_3 = Button(
    image=button_image_3,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_3.place(
    x=277.0,
    y=282+30.0,
    width=96.0,
    height=32.0
)


def button_pressed():
    setMotorActive(True)


def button_released():
    setMotorActive(False)


button_3.bind("<ButtonPress-1>", lambda event: button_pressed())
button_3.bind("<ButtonRelease-1>", lambda event: button_released())

button_image_4 = PhotoImage(
    file=relative_to_assets("button_4.png"))
button_4 = Button(
    image=button_image_4,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setGlobals(entry_1.get(), entry_2.get()),
    relief="flat",
)
button_4.place(
    x=299.0,
    y=146+30.0,
    width=53.0,
    height=32.0
)


def update_button_image():
    if direction:
        button_image_6.configure(file=relative_to_assets("button_for.png"))
    else:
        button_image_6.configure(file=relative_to_assets("button_bac.png"))


def setDirection(dir):
    global direction
    direction = not dir
    print("Direction is set to " + str(direction))
    update_display_string("Direction is set to " + str(direction))
    update_button_image()


button_image_6 = PhotoImage(
    file=relative_to_assets("button_for.png"))
button_6 = Button(
    image=button_image_6,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setDirection(direction),
    relief="flat",
)
button_6.place(
    x=299.0,
    y=218+30.0,
    width=53.0,
    height=32.0
)

button_image_5 = PhotoImage(
    file=relative_to_assets("button_5.png"))
button_5 = Button(
    image=button_image_5,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setMotorActive(False),
    relief="flat"
)
button_5.place(
    x=363.0,
    y=211+30.0,
    width=136.0,
    height=46.0
)

canvas.create_text(
    207.0,
    3.0,
    anchor="nw",
    text="SINGLE STEPPER CONTROL UI",
    fill="#727272",
    font=("Inter SemiBold", 16 * -1)
)

canvas.create_rectangle(
    -1.0,
    24.0,
    650.0,
    25.0,
    fill="#D6D6D6",
    outline="")

canvas.create_text(
    203.0,
    127+30.0,
    anchor="nw",
    text="SPEED",
    fill="#3B3D70",
    font=("Inter SemiBold", 10 * -1)
)

boardSetup()


main_thread = threading.Thread(target=runMainThread)
main_thread.daemon = True
main_thread.start()

window.resizable(False, False)
window.mainloop()
