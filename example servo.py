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

# from tkinter import *
# Explicit imports to satisfy Flake8
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage

system = platform.system()

current_datetime = datetime.datetime.now()
formatted_datetime = current_datetime.strftime("%d-%m-%Y | %I_%M_%S %p")


class AnimatedGifViewer:
    def __init__(self, parent):
        self.parent = parent
        self.canvas = tk.Canvas(self.parent)
        self.canvas.pack()
        self.gif_frames = []
        self.gif_index = 0

    def place(self, x, y):
        self.canvas.place(x=x, y=y)

    def set_size(self, width, height):
        self.canvas.config(width=width, height=height)

    def load_gif(self, gif_path):
        self.gif_frames = []
        self.gif_index = 0
        self.gif_object = Image.open(gif_path)
        self.gif_frames = self.get_gif_frames()
        self.update_image()

    def get_gif_frames(self):
        frames = []
        try:
            while True:
                gif_frame = self.gif_object.copy()
                frames.append(gif_frame)
                self.gif_object.seek(len(frames))
        except EOFError:
            pass
        return frames

    def update_image(self):
        self.canvas.delete("all")
        gif_frame = self.gif_frames[self.gif_index]
        self.gif_image = ImageTk.PhotoImage(gif_frame)
        self.canvas.create_image(0, 0, anchor="nw", image=self.gif_image)
        self.gif_index += 1
        if self.gif_index >= len(self.gif_frames):
            self.gif_index = 0
        # Update every 100ms (adjust as needed)
        self.parent.after(100, self.update_image)

    def change_source(self, gif_path):
        self.load_gif(gif_path)


consoleBox = None


code_path = os.path.abspath(__file__)
assets_relative_path = os.path.join(
    os.path.dirname(code_path), "assets/frame0")
ASSETS_PATH = Path(assets_relative_path)


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)


def updateConsole(value):
    global consoleBox

    try:
        consoleBox.insert(tk.END, value + "\n")
        consoleBox.see(tk.END)
    except:
        print("Minor Warning-Unable to update console")

###########################################


# def calcPosition(x):
#     # giving in mm
#     # position = .9526*(x*-1.0893 + 61.922)
#     position = x*degree_to_mm_factor
#     position = round(position, 3)
#     return position


# def calcPressure(x):
#     # giving in kpa
#     # pressure = x*1023*.058
#     pressure = x*1023*sensor_to_pressure_factor
#     pressure = round(pressure, 3)
#     return pressure


def append_to_csv(file_path, column1_data, column2_data, column3_data):
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([column1_data, column2_data, column3_data])


def export_data(array1, array2, array3, file_name):

    for i in range(len(array1)):
        element1 = array1[i]
        element2 = array2[i]
        element3 = array3[i]
        print(element1, element2, element3)
        append_to_csv(file_name, element1, element2, element3)


############################################


def moveToPos_smooth(pin_num, pos):
    global board

    deltime = 0.1
    try:
        cur_pos = int(board.digital[pin_num].read())
    except:
        print("Not able to read from pin")
        return

    if (cur_pos > pos):
        for i in range(cur_pos, pos, -1):
            try:
                board.digital[pin_num].write(i)
            except:
                print("Not able to write to pin")
            # if (pin_num == 9):
            #     board.digital[7].write(180-i)
            print("Moving standard position, please wait...")
            updateConsole("Moving standard position, please wait...")
            time.sleep(deltime)

    elif (cur_pos < pos):
        for i in range(cur_pos, pos, 1):
            try:
                board.digital[pin_num].write(i)
            except:
                print("Not able to write to pin")
            # if (pin_num == 11):
            #     board.digital[7].write(180-i)
            print("Moving standard position, please wait...")
            updateConsole("Moving standard position, please wait...")
            time.sleep(deltime)

    print("Robot now in standard position.")
    updateConsole("Robot now in standard position.")


def close_action():

    global exit_flag

    exit_flag = True

    moveToPos_smooth(8, 52)
    moveToPos_smooth(9, 43)
    # moveToPos_smooth(7, 180-43)
    moveToPos_smooth(10, 0)
    moveToPos_smooth(11, 4)
    moveToPos_smooth(12, 0)

    moveToPos_smooth(13, 20)

    print("Exiting...")

    os._exit(0)


def refreshSpeedEntries():
    global servo_speed

    try:
        entry_1.delete(0, tk.END)
        entry_1.insert(0, servo_speed[0])
        entry_2.delete(0, tk.END)
        entry_2.insert(0, servo_speed[1])
        entry_3.delete(0, tk.END)
        entry_3.insert(0, servo_speed[2])
        entry_4.delete(0, tk.END)
        entry_4.insert(0, servo_speed[3])
        entry_5.delete(0, tk.END)
        entry_5.insert(0, servo_speed[4])
    except:
        print("Minor Warning-Unable to refresh entries")
        updateConsole("Minor Warning-Unable to refresh entries")


def handleButton(servo_num, direction):
    global servo_direction
    servo_direction[servo_num] = direction
    print("Servo " + str(servo_num) + " is now moving " + str(direction))
    updateConsole("Servo " + str(servo_num) +
                  " is now moving " + str(direction))


def setSpeed(servo_num, speed):
    global servo_speed

    speed = float(speed)

    if (servo_num != 99):
        servo_speed[servo_num] = speed
        print("Servo " + str(servo_num) + " is now moving " +
              str(speed) + " degrees faster")
        updateConsole("Servo " + str(servo_num) +
                      " is now moving " + str(speed) + " degrees faster")

    else:
        for i in range(5):
            servo_speed[i] = speed
        refreshSpeedEntries()
        print("All Servos are now moving " + str(speed) + " degrees faster")
        updateConsole("All Servos are now moving " +
                      str(speed) + " degrees faster")


def boardSetup():
    global board, servo_pins, gripper_pin, sensor
    ######################## Arduino Setup ########################
    lines = []

    if system == "Darwin":
        print("Mac OS Detected, finding ports")
        updateConsole("Mac OS Detected, finding ports")
        output = os.popen('ls /dev/tty.*').read()
    elif system == "Windows":
        print("Windows Detected, finding ports")
        updateConsole("Windows Detected, finding ports")
        output = os.popen('wmic path Win32_SerialPort').read()
    else:
        print("Unknown operating system")
        updateConsole("Unknown operating system")

    output = os.popen('ls /dev/tty.*').read()

    for substr in output.split('\n'):
        substr2 = re.sub('\x1b\[[0-9;]*m', '', substr)
        lines.append(substr2)
        print(substr2)
        updateConsole(substr2)
    lines.pop()

    def_index = 0
    for i, line in enumerate(lines):
        if "usb" in line:
            def_index = i
            break

    default_port_name = lines[def_index]
    print("Default Port: " + default_port_name)
    updateConsole("Default Port: " + default_port_name)

    try:
        board = pyfirmata.Arduino(default_port_name)
        it = pyfirmata.util.Iterator(board)
        it.start()
        sensor = board.get_pin('a:2:i')
    except:
        # tk.messagebox.showerror('Arduino Selection error', 'Arduino not found')
        print("Arduino Not Found")

    try:
        board.digital[gripper_pin].mode = pyfirmata.SERVO
        board.digital[gripper_pin].write(20)
    except:
        print("Unable to set digital pins for output")

    try:
        board.digital[8].mode = pyfirmata.SERVO
        # moveToPos_smooth(8, 52)
        board.digital[8].write(52)

        board.digital[9].mode = pyfirmata.SERVO
        # moveToPos_smooth(9, 43)
        board.digital[9].write(43)
        # board.digital[7].mode = pyfirmata.SERVO
        # # moveToPos_smooth(7, 180-43)
        # board.digital[7].write(180-43)

        board.digital[10].mode = pyfirmata.SERVO
        # moveToPos_smooth(10, 0)
        board.digital[10].write(0)

        board.digital[11].mode = pyfirmata.SERVO
        # moveToPos_smooth(11, 4)
        board.digital[11].write(4)

        board.digital[12].mode = pyfirmata.SERVO
        # moveToPos_smooth(12, 0)
        board.digital[12].write(0)
    except:
        print("Cant get arduino digital pins")
    ######################## Arduino Setup ########################


def servoRunLoop():
    global servo_direction, board, servo_pins, exit_flag
    while True:
        if (exit_flag):
            print("Servo thread stopped")
            break

        for i in range(5):
            cur_loc = board.digital[servo_pins[i]].read()

            if cur_loc is not None:
                cur_loc = int(cur_loc)
            else:
                cur_loc = 0
                # print("Error Reading Servo " + str(i) + " Location")
                continue

            if servo_direction[i] == 1:
                cur_loc += servo_speed[i]
                print("Servo " + str(i) + " is now moving " +
                      str(cur_loc) + " degrees clockwise")
                updateConsole("Servo " + str(i) + " is now moving " +
                              str(cur_loc) + " degrees clockwise")

            elif servo_direction[i] == 2:
                cur_loc -= servo_speed[i]
                print("Servo " + str(i) + " is now moving " +
                      str(cur_loc) + " degrees counterclockwise")
                updateConsole("Servo " + str(i) + " is now moving " +
                              str(cur_loc) + " degrees counterclockwise")

            cur_loc = max(0, min(cur_loc, 180))
            board.digital[servo_pins[i]].write(cur_loc)

            # if (i == 1):
            #     board.digital[7].write(180-cur_loc)

            time.sleep(0.01)

########################### Gripper Functions ###########################


def setGripperAngle(minmax, val):
    global gripper_angle_min, gripper_angle_max
    if (minmax == 0):
        gripper_angle_min = int(val)
        print("Gripper is now moving between " + str(val) +
              " and " + str(gripper_angle_max) + " degrees")
        updateConsole("Gripper is now moving between " + str(val) +
                      " and " + str(gripper_angle_max) + " degrees")
    else:
        gripper_angle_max = int(val)
        print("Gripper is now moving between " +
              str(gripper_angle_min) + " and " + str(val) + " degrees")
        updateConsole("Gripper is now moving between " +
                      str(gripper_angle_min) + " and " + str(val) + " degrees")

    cur_loc = board.digital[gripper_pin].read()

    if (cur_loc < gripper_angle_min):
        board.digital[gripper_pin].write(gripper_angle_min)
    elif (cur_loc > gripper_angle_max):
        board.digital[gripper_pin].write(gripper_angle_max)


def setGripperSpeed(speed):
    global gripper_speed
    gripper_speed = float(speed)
    print("Gripper is now moving " + str(speed) + " degrees faster")
    updateConsole("Gripper is now moving " + str(speed) + " degrees faster")


def setGripperPrecision(precision):
    global gripper_precision
    gripper_precision = float(precision)
    print("Gripper is now moving every " + str(precision) + " seconds")
    updateConsole("Gripper is now moving every " + str(precision) + " seconds")


def handleGripperButton(direction):
    global gripper_direction
    gripper_direction = direction
    print("Gripper is now moving " + str(direction))
    updateConsole("Gripper is now moving " + str(direction))


def getTableparams():
    global get_table_params_flag, table_index, table_diameter, table_estar, table_delta_pressure, table_deformation_mm
    global deformation_to_mm, sensor_value, gripper_degree

    table_diameter = calcDiameter(gripper_degree)
    table_deformation_mm = deformation_to_mm
    table_delta_pressure = calcDeltaPressure(sensor_value)
    table_estar = calcEstar(table_delta_pressure,
                            table_diameter/2, table_deformation_mm)

    updateConsole("Table: " + str(table_index))
    updateConsole("Diameter: " + str(table_diameter))
    updateConsole("Deformation: " + str(table_deformation_mm))
    updateConsole("Pressure: " + str(table_delta_pressure))
    updateConsole("Estar: " + str(table_estar))

    # get_table_params_flag = False


def runGripperLoop():
    global board, gripper_pin, gripper_direction, gripper_speed, gripper_precision, sensor_value, sensor_max, gripper_angle_max, gripper_angle_min

    global touched_flag, touched_angle, deformation

    global sensor, sensor_touch_value, sensor_min, get_table_params_flag

    board.digital[gripper_pin].write(gripper_angle_min)

    while True:
        if (gripper_direction != 0):

            cur_loc = board.digital[gripper_pin].read()

            #######################

            temp_read_sensor = sensor.read()

            if (temp_read_sensor >= sensor_min+sensor_touch_value and touched_flag == False):
                touched_flag = True
                get_table_params_flag = True
                touched_angle = cur_loc
                print("------------Touched")
                updateConsole("-----------Touched")

            #######################

            if (gripper_direction == 1):
                cur_loc += gripper_speed
                print("Gripper is now moving " +
                      str("{:.3f}".format(cur_loc)) + " degrees")
                updateConsole("Gripper is now moving " +
                              str("{:.3f}".format(cur_loc)) + " degrees")

            elif (gripper_direction == 2):
                touched_flag = False
                touched_angle = 0.0
                gripper_direction = 0

                while (cur_loc > gripper_angle_min):
                    cur_loc -= gripper_speed
                    board.digital[gripper_pin].write(cur_loc)
                    time.sleep(gripper_precision)
                continue

            if (touched_flag == True):
                gripper_direction = 0
                # touched_flag = False
                print("Gripper has grabbed the object")
                updateConsole("Gripper has grabbed the object")

                deformed_cur_loc = cur_loc + deformation

                board.digital[gripper_pin].write(deformed_cur_loc)

                getTableparams()

                # while (cur_loc < (touched_angle+deformation)):
                #     cur_loc += gripper_speed
                #     board.digital[gripper_pin].write(cur_loc)
                #     time.sleep(gripper_precision)
                continue

            # need to add sensor read value here to stop gripper
            elif ((gripper_direction != 2 and sensor_value >= sensor_max) or cur_loc >= gripper_angle_max or cur_loc <= gripper_angle_min):
                gripper_direction = 0
                print("Gripper has reached its limit")
                updateConsole("Gripper has reached its limit")
                continue

            if (gripper_direction != 0):
                board.digital[gripper_pin].write(cur_loc)

        time.sleep(gripper_precision)


########################### Gripper Functions ###########################

########################### Sensor Functions ###########################


def setSensorRange(minmax, val):
    global sensor_min, sensor_max
    if (minmax == 0):
        sensor_min = float(val)
        print("Sensor is now reading between " + str(val) +
              " and " + str(sensor_max) + " mm")
        updateConsole("Sensor is now reading between " + str(val) +
                      " and " + str(sensor_max) + " mm")
    else:
        sensor_max = float(val)
        print("Sensor is now reading between " +
              str(sensor_min) + " and " + str(val) + " mm")
        updateConsole("Sensor is now reading between " +
                      str(sensor_min) + " and " + str(val) + " mm")


def calcEstar(del_p, rad, defrm):

    # E* = 2.355*DP*(R/d)^0.5

    global EStar, touched_flag

    if (touched_flag == True):
        try:
            temp1 = math.sqrt(rad/defrm)

            EStar = 2.355*del_p*temp1
        except:
            EStar = 0.00
    else:
        EStar = 0.00

    return EStar


def calcDiameter(angle):
    global gripper_angle_max, diameter, deformation, gripper_angle_min

    # radius = ((gripper_angle_max-angle) * degree_to_mm_factor)/2

    # radius = (angle - deformation - gripper_angle_min)**2 * 0.0149 + \
    # 0.0323 * (angle - deformation - gripper_angle_min)

    x = (angle - deformation - gripper_angle_min)

    a = (x**2)*0.0179
    b = x*0.241

    diameter = (54.345 - a - b)*1.3

    return diameter


def calcDeltaPressure(sensorV):
    global sensor_to_pressure_factor, delta_pressure, sensor_min

    delta_pressure = (sensorV-sensor_min) * sensor_to_pressure_factor

    return delta_pressure


def runSensorLoop():
    global sensor, sensor_value, board, sample_rate, gripper_pin, gripper_degree, runSystem_flag, entry_livesensor, entry_16, entry_15, entry_livegripper, EStar, delta_pressure, diameter, sensor_touch_value, entry_14, deformation
    global export_delta_pressure, export_radius, export_estar, touched_flag, touched_angle, sensor_min, sensor_max, deformation_to_mm

    while True:
        if (runSystem_flag == True):

            try:
                sensor_value = sensor.read()
                gripper_degree = board.digital[gripper_pin].read()
            except:
                # give random values
                sensor_value = 1234
                gripper_degree = 123.9292

            # if (sensor_value >= sensor_min+sensor_touch_value and touched_flag == False):
                # touched_flag = True
                # touched_angle = gripper_degree
                # print("Touched")
                # updateConsole("Touched")

            entry_livesensor.delete(0, tk.END)
            entry_livesensor.insert(0, str("{:.4f}".format(sensor_value)))
            entry_livegripper.delete(0, tk.END)
            entry_livegripper.insert(0, str("{:.3f}".format(gripper_degree)))
            entry_16.delete(0, tk.END)
            entry_16.insert(
                0, str("{:.3f}".format(calcDeltaPressure(sensor_value))))
            entry_15.delete(0, tk.END)
            entry_15.insert(
                0, str("{:.3f}".format(calcDiameter(gripper_degree))))

            EStar = calcEstar(delta_pressure, diameter/2, deformation_to_mm)

            entry_14.delete(0, tk.END)
            entry_14.insert(0, str("{:.3f}".format(EStar)))

            export_delta_pressure.append(delta_pressure)
            export_radius.append(diameter)
            export_estar.append(EStar)

            # updateConsole("Sensor Value: " + str(sensor_value))
            # updateConsole("Gripper Value: " + str(gripper_degree))

        else:
            print("System is paused")
            # updateConsole("System is paused")

        time.sleep(1/float(sample_rate))


########################### Define All Global Variables ###########################
# 9 is double servo with pin 7 in reverse, 8 is base
servo_pins = [8, 9, 10, 11, 12]
servo_direction = [0, 0, 0, 0, 0]
servo_speed = [1, 1, 1, 1, 1]

gripper_pin = 13
gripper_speed = 0.1
gripper_precision = 0.001
gripper_direction = 0
gripper_angle_min = 20
gripper_angle_max = 70

sensor_value = 0
sensor = None
sensor_min = 0.130
sensor_max = 0.9999


board = None

boardSetup()

exit_flag = False
run_flag = False

export_radius = []
export_delta_pressure = []
export_estar = []

runSystem_flag = True

# degree_to_mm_factor = 1
sensor_to_pressure_factor = 91.0

sensor_touch_value = 0.150

########
touched_flag = False
touched_angle = 0.0
########

########
EStar = 0.0

sample_rate = 10

gripper_degree = 0.0
delta_pressure = 0.0
diameter = 0.0

deformation = 1.0
deformation_to_mm = ((0.0179*deformation*deformation) +
                     (0.241*deformation))*1.3


get_table_params_flag = False
table_index = 1

table_diameter = 0
table_estar = 0
table_deformation_mm = 0
table_delta_pressure = 0
########################### Define All Global Variables ###########################


window = Tk()

window.geometry("1200x750")
window.configure(bg="#FFFFFF")


canvas = Canvas(
    window,
    bg="#FFFFFF",
    height=750,
    width=1200,
    bd=0,
    highlightthickness=0,
    relief="ridge"
)

canvas.place(x=0, y=0)

######################## Animation Gif Viewer ########################
# Need to work on this, dont work or change animation
# animationPlayer = AnimatedGifViewer(window)
# animationPlayer.place(x=530, y=526)
# animationPlayer.set_size(257, 225)
# animationPlayer.load_gif("circuit.gif")

# gif_viewer = AnimatedGifViewer(root)
# gif_viewer.place(100, 100)
# gif_viewer.set_size(200, 200)
# gif_viewer.load_gif("example.gif")


# # Changing the source of the GIF image
# gif_viewer.change_source("example2.gif")

######################## Animation Gif Viewer ########################

##################### Base Servo Buttons #####################
# Servo Control Buttons
button_image_18 = PhotoImage(
    file=relative_to_assets("button_18.png"))
button_18 = Button(
    image=button_image_18,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_18.bind("<Button-1>", lambda event: handleButton(0, 1))
button_18.bind("<ButtonRelease-1>", lambda event: handleButton(0, 0))

button_18.place(
    x=88.0,
    y=266.0,
    width=75.0,
    height=35.0
)


button_image_21 = PhotoImage(
    file=relative_to_assets("button_21.png"))
button_21 = Button(
    image=button_image_21,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_21.bind("<Button-1>", lambda event: handleButton(0, 2))
button_21.bind("<ButtonRelease-1>", lambda event: handleButton(0, 0))
button_21.place(
    x=178.0,
    y=266.0,
    width=75.0,
    height=35.0
)

# Speed Control
entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    293.0,
    283.5,
    image=entry_image_1
)
entry_1 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)

entry_1.insert(0, servo_speed[0])

entry_1.place(
    x=266.0,
    y=266.0,
    width=54.0,
    height=33.0
)

button_image_6 = PhotoImage(
    file=relative_to_assets("button_6.png"))
button_6 = Button(
    image=button_image_6,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSpeed(0, entry_1.get()),
    relief="flat"
)
button_6.place(
    x=333.0,
    y=266.0,
    width=41.0,
    height=35.0
)
##################### Base Servo Buttons #####################

##################### Upper Base Servo Buttons #####################

# Servo Control Buttons
button_image_23 = PhotoImage(
    file=relative_to_assets("button_23.png"))
button_23 = Button(
    image=button_image_23,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_23.bind("<Button-1>", lambda event: handleButton(1, 1))
button_23.bind("<ButtonRelease-1>", lambda event: handleButton(1, 0))
button_23.place(
    x=88.0,
    y=216.0,
    width=75.0,
    height=35.0
)

button_image_22 = PhotoImage(
    file=relative_to_assets("button_22.png"))
button_22 = Button(
    image=button_image_22,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_22.bind("<Button-1>", lambda event: handleButton(1, 2))
button_22.bind("<ButtonRelease-1>", lambda event: handleButton(1, 0))
button_22.place(
    x=178.0,
    y=216.0,
    width=75.0,
    height=35.0
)

# Speed Control
entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    293.0,
    233.5,
    image=entry_image_2
)
entry_2 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_2.insert(0, servo_speed[1])
entry_2.place(
    x=266.0,
    y=216.0,
    width=54.0,
    height=33.0
)

button_image_5 = PhotoImage(
    file=relative_to_assets("button_5.png"))
button_5 = Button(
    image=button_image_5,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSpeed(1, entry_2.get()),
    relief="flat"
)
button_5.place(
    x=333.0,
    y=216.0,
    width=41.0,
    height=35.0
)
##################### Upper Base Servo Buttons #####################

##################### Shoulder Servo Buttons #####################
# Servo Control Buttons
button_image_24 = PhotoImage(
    file=relative_to_assets("button_24.png"))
button_24 = Button(
    image=button_image_24,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_24.bind("<Button-1>", lambda event: handleButton(2, 1))
button_24.bind("<ButtonRelease-1>", lambda event: handleButton(2, 0))
button_24.place(
    x=88.0,
    y=166.0,
    width=75.0,
    height=35.0
)


button_image_27 = PhotoImage(
    file=relative_to_assets("button_27.png"))
button_27 = Button(
    image=button_image_27,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_27.bind("<Button-1>", lambda event: handleButton(2, 2))
button_27.bind("<ButtonRelease-1>", lambda event: handleButton(2, 0))
button_27.place(
    x=178.0,
    y=166.0,
    width=75.0,
    height=35.0
)

# Speed Control
entry_image_3 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_3 = canvas.create_image(
    293.0,
    183.5,
    image=entry_image_3
)
entry_3 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_3.insert(0, servo_speed[2])
entry_3.place(
    x=266.0,
    y=166.0,
    width=54.0,
    height=33.0
)

button_image_4 = PhotoImage(
    file=relative_to_assets("button_4.png"))
button_4 = Button(
    image=button_image_4,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSpeed(2, entry_3.get()),
    relief="flat"
)
button_4.place(
    x=333.0,
    y=166.0,
    width=41.0,
    height=35.0
)
##################### Shoulder Servo Buttons #####################

##################### Neck Servo Buttons #####################
# Servo Control Buttons
button_image_19 = PhotoImage(
    file=relative_to_assets("button_19.png"))
button_19 = Button(
    image=button_image_19,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_19.bind("<Button-1>", lambda event: handleButton(3, 1))
button_19.bind("<ButtonRelease-1>", lambda event: handleButton(3, 0))
button_19.place(
    x=88.0,
    y=116.0,
    width=75.0,
    height=35.0
)

button_image_20 = PhotoImage(
    file=relative_to_assets("button_20.png"))
button_20 = Button(
    image=button_image_20,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_20.bind("<Button-1>", lambda event: handleButton(3, 2))
button_20.bind("<ButtonRelease-1>", lambda event: handleButton(3, 0))
button_20.place(
    x=178.0,
    y=116.0,
    width=75.0,
    height=35.0
)

# Speed Control
entry_image_4 = PhotoImage(
    file=relative_to_assets("entry_4.png"))
entry_bg_4 = canvas.create_image(
    293.0,
    133.5,
    image=entry_image_4
)
entry_4 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_4.insert(0, servo_speed[3])
entry_4.place(
    x=266.0,
    y=116.0,
    width=54.0,
    height=33.0
)
button_image_3 = PhotoImage(
    file=relative_to_assets("button_3.png"))
button_3 = Button(
    image=button_image_3,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSpeed(3, entry_4.get()),
    relief="flat"
)
button_3.place(
    x=333.0,
    y=116.0,
    width=41.0,
    height=35.0
)
##################### Neck Servo Buttons ######################

##################### Elbow Servo Buttons #####################
# Servo Control Buttons
button_image_25 = PhotoImage(
    file=relative_to_assets("button_25.png"))
button_25 = Button(
    image=button_image_25,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_25.bind("<Button-1>", lambda event: handleButton(4, 1))
button_25.bind("<ButtonRelease-1>", lambda event: handleButton(4, 0))
button_25.place(
    x=88.0,
    y=66.0,
    width=75.0,
    height=35.0
)

button_image_26 = PhotoImage(
    file=relative_to_assets("button_26.png"))
button_26 = Button(
    image=button_image_26,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_26.bind("<Button-1>", lambda event: handleButton(4, 2))
button_26.bind("<ButtonRelease-1>", lambda event: handleButton(4, 0))
button_26.place(
    x=178.0,
    y=66.0,
    width=75.0,
    height=35.0
)

# Speed Control
entry_image_5 = PhotoImage(
    file=relative_to_assets("entry_5.png"))
entry_bg_5 = canvas.create_image(
    293.0,
    83.5,
    image=entry_image_5
)
entry_5 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_5.insert(0, servo_speed[4])
entry_5.place(
    x=266.0,
    y=66.0,
    width=54.0,
    height=33.0
)

button_image_2 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_2 = Button(
    image=button_image_2,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSpeed(4, entry_5.get()),
    relief="flat"
)
button_2.place(
    x=333.0,
    y=66.0,
    width=41.0,
    height=35.0
)
##################### Elbow Servo Buttons #######################

##################### Gripper Servo Buttons #####################

button_image_28 = PhotoImage(
    file=relative_to_assets("button_28.png"))
button_28 = Button(
    image=button_image_28,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_28.bind("<Button-1>", lambda event: handleGripperButton(1))
# button_28.bind("<ButtonRelease-1>", lambda event: handleGripperButton(0))
button_28.place(
    x=89.0,
    y=406.0,
    width=136.0,
    height=46.0
)

button_image_29 = PhotoImage(
    file=relative_to_assets("button_29.png"))
button_29 = Button(
    image=button_image_29,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_29.bind("<Button-1>", lambda event: handleGripperButton(2))
# button_29.bind("<ButtonRelease-1>", lambda event: handleGripperButton(0))
button_29.place(
    x=239.0,
    y=406.0,
    width=136.0,
    height=46.0
)

##################### Gripper Servo Buttons #####################

##################### Gripper Params & Buttons #######################

entry_image_6 = PhotoImage(
    file=relative_to_assets("entry_6.png"))
entry_bg_6 = canvas.create_image(
    130.5,
    510.5,
    image=entry_image_6
)
entry_6 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_6.insert(0, gripper_precision)
entry_6.place(
    x=89.0,
    y=493.0,
    width=83.0,
    height=33.0
)


entry_image_7 = PhotoImage(
    file=relative_to_assets("entry_7.png"))
entry_bg_7 = canvas.create_image(
    280.5,
    510.5,
    image=entry_image_7
)
entry_7 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_7.insert(0, gripper_speed)
entry_7.place(
    x=239.0,
    y=493.0,
    width=83.0,
    height=33.0
)


button_image_7 = PhotoImage(
    file=relative_to_assets("button_7.png"))
button_7 = Button(
    image=button_image_7,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setGripperPrecision(entry_6.get()),
    relief="flat"
)
button_7.place(
    x=184.0,
    y=493.0,
    width=41.0,
    height=35.0
)

button_image_8 = PhotoImage(
    file=relative_to_assets("button_8.png"))
button_8 = Button(
    image=button_image_8,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setGripperSpeed(entry_7.get()),
    relief="flat"
)
button_8.place(
    x=334.0,
    y=493.0,
    width=41.0,
    height=35.0
)

##################### Gripper Params & Buttons #######################

###################### Exit Button ######################
button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=close_action,
    relief="flat"
)
button_1.place(
    x=1005.0,
    y=691.0,
    width=136.0,
    height=46.0
)
###################### Exit Button ######################

###################### Speed Slider ######################


def slider_changed(event):
    speed = slider.get()
    servo_number = 99
    setSpeed(servo_number, speed)


style = ttk.Style()
style.configure("Custom.Horizontal.TScale", troughcolor="white")

slider = ttk.Scale(window, from_=0, to=50, orient=tk.HORIZONTAL,
                   length=265, command=slider_changed, style="Custom.Horizontal.TScale")
slider.set(1)
slider.place(x=98, y=327)


###################### Speed Slider ######################

########################### Console Object ###########################

consoleBox = Text(
    bd=0,
    bg="#ECFFEF",
    fg="#000716",
    highlightthickness=0
)
consoleBox.insert("end", "This is the main console"+"\n" +
                  "It will display all the actions performed by the robot" + "\n")

consoleBox.place(
    x=946.0,
    y=57.0,
    width=253.0,
    height=620.0
)


def clearConsoleFunc():
    global export_radius, export_delta_pressure, export_estar

    consoleBox.delete(1.0, "end")
    export_delta_pressure = []
    export_radius = []
    export_estar = []


console_button_image = PhotoImage(
    file=relative_to_assets("console_button.png"))
console_clear_button = Button(
    image=console_button_image,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: clearConsoleFunc(),
    relief="flat"
)
console_clear_button.place(
    x=1152.0,
    y=677.0,
    width=48.0,
    height=30.0
)

########################### Console Object ###########################

########################### Gripper Angle Setting Object ###########################
entry_image_8 = PhotoImage(
    file=relative_to_assets("entry_8.png"))
entry_bg_8 = canvas.create_image(
    129.5,
    575.5,
    image=entry_image_8
)
entry_8 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_8.insert(0, gripper_angle_min)
entry_8.place(
    x=88.0,
    y=558.0,
    width=83.0,
    height=33.0
)

entry_image_9 = PhotoImage(
    file=relative_to_assets("entry_9.png"))
entry_bg_9 = canvas.create_image(
    279.5,
    575.5,
    image=entry_image_9
)
entry_9 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_9.insert(0, gripper_angle_max)
entry_9.place(
    x=238.0,
    y=558.0,
    width=83.0,
    height=33.0
)

button_image_9 = PhotoImage(
    file=relative_to_assets("button_9.png"))
button_9 = Button(
    image=button_image_9,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setGripperAngle(0, entry_8.get()),
    relief="flat"
)
button_9.place(
    x=183.0,
    y=558.0,
    width=41.0,
    height=35.0
)


button_image_10 = PhotoImage(
    file=relative_to_assets("button_10.png"))
button_10 = Button(
    image=button_image_10,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setGripperAngle(1, entry_9.get()),
    relief="flat"
)
button_10.place(
    x=333.0,
    y=558.0,
    width=41.0,
    height=35.0
)

########################### Gripper Angle Setting Object ###########################

########################### Sensor MinMax Setting Object ###########################

entry_image_10 = PhotoImage(
    file=relative_to_assets("entry_10.png"))
entry_bg_10 = canvas.create_image(
    131.5,
    639.5,
    image=entry_image_10
)
entry_10 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_10.insert(0, sensor_min)
entry_10.place(
    x=90.0,
    y=622.0,
    width=83.0,
    height=33.0
)

entry_image_11 = PhotoImage(
    file=relative_to_assets("entry_11.png"))
entry_bg_11 = canvas.create_image(
    281.5,
    639.5,
    image=entry_image_11
)
entry_11 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_11.insert(0, sensor_max)
entry_11.place(
    x=240.0,
    y=622.0,
    width=83.0,
    height=33.0
)

button_image_11 = PhotoImage(
    file=relative_to_assets("button_11.png"))
button_11 = Button(
    image=button_image_11,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSensorRange(0, entry_10.get()),
    relief="flat"
)
button_11.place(
    x=185.0,
    y=622.0,
    width=41.0,
    height=35.0
)

button_image_12 = PhotoImage(
    file=relative_to_assets("button_12.png"))
button_12 = Button(
    image=button_image_12,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: setSensorRange(1, entry_11.get()),
    relief="flat"
)
button_12.place(
    x=335.0,
    y=622.0,
    width=41.0,
    height=35.0
)

########################### Sensor MinMax Setting Object ###########################


def updateSampleRate(val):
    global sample_rate
    sample_rate = val
    print("sample rate: ", sample_rate)
    updateConsole("Sample rate updated to: " + str(sample_rate))


entry_image_13 = PhotoImage(
    file=relative_to_assets("entry_13.png"))
entry_bg_13 = canvas.create_image(
    281.5,
    702.5,
    image=entry_image_13
)
entry_13 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)

entry_13.insert(0, sample_rate)

entry_13.place(
    x=240.0,
    y=685.0,
    width=83.0,
    height=33.0
)
button_image_13 = PhotoImage(
    file=relative_to_assets("button_13.png"))
button_13 = Button(
    image=button_image_13,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: updateSampleRate(float(entry_13.get())),
    relief="flat"
)
button_13.place(
    x=335.0,
    y=685.0,
    width=41.0,
    height=35.0
)

entry_image_12 = PhotoImage(
    file=relative_to_assets("entry_12.png"))
entry_bg_12 = canvas.create_image(
    158.0,
    702.5,
    image=entry_image_12
)
entry_12 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)

entry_12.place(
    x=90.0,
    y=685.0,
    width=136.0,
    height=33.0
)


########################## E* Entry ################################
entry_image_14 = PhotoImage(
    file=relative_to_assets("entry_14.png"))
entry_bg_14 = canvas.create_image(
    661.0,
    428.0,
    image=entry_image_14
)
entry_14 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_14.insert(0, EStar)

entry_14.place(
    x=593.0,
    y=405.0,
    width=136.0,
    height=44.0
)

########################## E* Entry ################################

########################## Live sensor entry #######################
entry_image_livesensor = PhotoImage(
    file=relative_to_assets("entry_15.png"))
entry_bg_livesensor = canvas.create_image(
    496.0,
    428.0,
    image=entry_image_livesensor
)
entry_livesensor = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_livesensor.insert(0, "0")
entry_livesensor.place(
    x=550.0,
    y=550.0,
    width=136.0,
    height=44.0
)

########################## Live sensor entry #######################

########################## Live gripper entry #######################
entry_image_livegripper = PhotoImage(
    file=relative_to_assets("entry_15.png"))
entry_bg_livegripper = canvas.create_image(
    496.0,
    428.0,
    image=entry_image_livegripper
)
entry_livegripper = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_livegripper.insert(0, "0")
entry_livegripper.place(
    x=700.0,
    y=550.0,
    width=136.0,
    height=44.0
)

########################## Live gripper entry #######################

########################## Diameter entry ####################
entry_image_15 = PhotoImage(
    file=relative_to_assets("entry_15.png"))
entry_bg_15 = canvas.create_image(
    496.0,
    428.0,
    image=entry_image_15
)
entry_15 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_15.insert(0, "0")
entry_15.place(
    x=428.0,
    y=405.0,
    width=136.0,
    height=44.0
)
########################## Diameter entry ####################

#################### Sensor Val show entry ####################

entry_image_16 = PhotoImage(
    file=relative_to_assets("entry_16.png"))
entry_bg_16 = canvas.create_image(
    833.0,
    427.0,
    image=entry_image_16
)
entry_16 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_16.insert(0, "None")


entry_16.place(
    x=765.0,
    y=404.0,
    width=136.0,
    height=44.0
)

#################### Sensor Val show entry ####################

button_image_14 = PhotoImage(
    file=relative_to_assets("button_14.png"))
button_14 = Button(
    image=button_image_14,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: print("button_14 clicked"),
    relief="flat"
)
button_14.place(
    x=428.0,
    y=465.0,
    width=96.12933349609375,
    height=46.0
)


def updateSysFlag(flag):
    global runSystem_flag

    if (flag == '1'):
        runSystem_flag = True
        print("System is running")
        updateConsole("System is running...")
    else:
        runSystem_flag = False
        print("System is paused")
        updateConsole("System is paused")

    print("runSystem_flag: ", runSystem_flag)


button_image_15 = PhotoImage(
    file=relative_to_assets("button_15.png"))
button_15 = Button(
    image=button_image_15,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: updateSysFlag('1'),
    relief="flat"
)
button_15.place(
    x=553.62353515625,
    y=465.0,
    width=96.12933349609375,
    height=46.0
)

button_image_16 = PhotoImage(
    file=relative_to_assets("button_16.png"))
button_16 = Button(
    image=button_image_16,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: updateSysFlag('0'),
    relief="flat"
)
button_16.place(
    x=679.2471313476562,
    y=465.0,
    width=96.12933349609375,
    height=46.0
)

button_image_17 = PhotoImage(
    file=relative_to_assets("button_17.png"))
button_17 = Button(
    image=button_image_17,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: export_data(export_radius, export_delta_pressure, export_estar, str(
        formatted_datetime)+'_data.csv'),
    relief="flat"
)
button_17.place(
    x=804.8706665039062,
    y=465.0,
    width=96.12933349609375,
    height=46.0
)

canvas.create_rectangle(
    -1.0,
    29.0,
    1200.0003662109375,
    30.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    387.5,
    28.9913330078125,
    388.5,
    751.0086669921875,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    945.0,
    28.0,
    946.0,
    750.017333984375,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    -1.0,
    373.0,
    946.0,
    374.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    -0.99932861328125,
    461.5,
    387.001953125,
    462.5,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    386.0,
    525.0,
    946.0,
    526.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    944.9960327148438,
    56.0,
    1200.00390625,
    57.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    945.0,
    676.0,
    1200.0078735351562,
    677.0,
    fill="#D6D6D6",
    outline="")


########### GUI ###########

canvas.create_rectangle(
    0.0,
    29.0,
    388.0,
    374.0,
    fill="#FFFFFF",
    outline="")

canvas.create_rectangle(
    0.0,
    373.0,
    388.0,
    463.0,
    fill="#FFFFFF",
    outline="")

canvas.create_rectangle(
    387.0,
    29.0,
    946.0,
    374.0,
    fill="#7D7D7D",
    outline="")

canvas.create_rectangle(
    387.0,
    373.0,
    946.0,
    751.0,
    fill="#FFFFFF",
    outline="")

canvas.create_rectangle(
    0.0,
    462.0,
    388.0,
    751.0,
    fill="#FFFFFF",
    outline="")

canvas.create_rectangle(
    9.0,
    76.0,
    24.0,
    91.0,
    fill="#4B5DFF",
    outline="")

canvas.create_rectangle(
    9.0,
    126.0,
    24.0,
    141.0,
    fill="#38BC6D",
    outline="")

canvas.create_rectangle(
    9.0,
    176.0,
    24.0,
    191.0,
    fill="#FAB603",
    outline="")

canvas.create_rectangle(
    9.0,
    226.0,
    24.0,
    241.0,
    fill="#FF5656",
    outline="")

canvas.create_rectangle(
    9.0,
    276.0,
    24.0,
    291.0,
    fill="#E57FFF",
    outline="")

canvas.create_rectangle(
    9.0,
    332.0,
    24.0,
    347.0,
    fill="#73CE00",
    outline="")

canvas.create_rectangle(
    9.0,
    420.0,
    24.0,
    435.0,
    fill="#00CECE",
    outline="")

canvas.create_rectangle(
    9.0,
    502.0,
    24.0,
    517.0,
    fill="#0008CE",
    outline="")

canvas.create_rectangle(
    9.0,
    567.0,
    24.0,
    582.0,
    fill="#8C00CE",
    outline="")

canvas.create_rectangle(
    10.0,
    631.0,
    25.0,
    646.0,
    fill="#CE003D",
    outline="")

canvas.create_rectangle(
    10.0,
    694.0,
    25.0,
    709.0,
    fill="#CE9400",
    outline="")

###### Slider Background ###########
canvas.create_rectangle(
    88.0,
    316.0,
    374.0,
    362.0,
    fill="#FFE3B9",
    outline="")


canvas.create_rectangle(
    946.0,
    57.0,
    1200.0,
    677.0,
    fill="#DCFFD0",
    outline="")

canvas.create_rectangle(
    388.0,
    526.0,
    845.0,
    751.0,
    fill="#ffffff",
    outline="")

canvas.create_rectangle(
    845.0,
    526.0,
    946.0,
    751.0,
    fill="#DAE4FF",
    outline="")

canvas.create_text(
    467.0,
    1.0,
    anchor="nw",
    text="SOFT BOT CONTROL PANEL",
    fill="#727272",
    font=("Inter SemiBold", 24 * -1)
)

canvas.create_text(
    28.0,
    74.0,
    anchor="nw",
    text="Motor 1",
    fill="#3B3D6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    28.0,
    224.0,
    anchor="nw",
    text="Motor 4",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    28.0,
    124.0,
    anchor="nw",
    text="Motor 2",
    fill="#6F6F6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    28.0,
    174.0,
    anchor="nw",
    text="Motor 3",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    28.0,
    124.0,
    anchor="nw",
    text="Motor 2",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    28.0,
    274.0,
    anchor="nw",
    text="Motor 5",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    31.0,
    322.0,
    anchor="nw",
    text="Overall\nSpeed",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    29.0,
    418.0,
    anchor="nw",
    text="Control",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    34.0,
    500.0,
    anchor="nw",
    text="Move",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    103.0,
    470.0,
    anchor="nw",
    text="Precision",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    615.0,
    380.0,
    anchor="nw",
    text="E*",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    450.0,
    380.0,
    anchor="nw",
    text="Jaw Gap(mm)",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    767.0,
    379.0,
    anchor="nw",
    text="Delta Pressure (kPa)",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    260.0,
    470.0,
    anchor="nw",
    text="Speed",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    110.0,
    40.0,
    anchor="nw",
    text="MOTOR CONTROL",
    fill="#6F6F6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    162.0,
    379.0,
    anchor="nw",
    text="GRASPER CONTROL",
    fill="#6F6F6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    1039.0,
    38.0,
    anchor="nw",
    text="CONSOLE",
    fill="#6F6F6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    270.0,
    40.0,
    anchor="nw",
    text="SPEED",
    fill="#6F6F6F",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    33.0,
    565.0,
    anchor="nw",
    text="Angle",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    116.0,
    535.0,
    anchor="nw",
    text="Min",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    266.0,
    535.0,
    anchor="nw",
    text="Max",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    32.0,
    629.0,
    anchor="nw",
    text="Sensor",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    32.0,
    692.0,
    anchor="nw",
    text="Others",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    117.0,
    599.0,
    anchor="nw",
    text="Resting",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    267.0,
    599.0,
    anchor="nw",
    text="Max",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    117.0,
    663.0,
    anchor="nw",
    text="Port Selection",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    256.0,
    663.0,
    anchor="nw",
    text="Sample Rate/s",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    852.0,
    726.0,
    anchor="nw",
    text="NIFE LAB",
    fill="#4E4D4D",
    font=("Inter SemiBold", 20 * -1)
)


canvas.create_rectangle(
    -1.0,
    29.0,
    1200.0003662109375,
    30.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    387.5,
    28.9913330078125,
    388.5,
    751.0086669921875,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    945.0,
    28.0,
    946.0,
    750.017333984375,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    -1.0,
    373.0,
    946.0,
    374.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    -0.99932861328125,
    461.5,
    387.001953125,
    462.5,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    386.0,
    525.0,
    946.0,
    526.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    944.9960327148438,
    56.0,
    1200.00390625,
    57.0,
    fill="#D6D6D6",
    outline="")

canvas.create_rectangle(
    945.0,
    676.0,
    1200.0078735351562,
    677.0,
    fill="#D6D6D6",
    outline="")

##########################

canvas.create_text(
    418.0,
    529.0,
    anchor="nw",
    text="Touch (+Rest)",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    563.0,
    529.0,
    anchor="nw",
    text="Live Sensor Read",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    710.0,
    529.0,
    anchor="nw",
    text="Live Gripper Angle",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)


canvas.create_text(
    416.0,
    627.0,
    anchor="nw",
    text="Deformation",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    516.0,
    627.0,
    anchor="nw",
    text="Deformation(mm)",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

canvas.create_text(
    430.0,
    604.0,
    anchor="nw",
    text="FACTORS",
    fill="#3B3D70",
    font=("Inter ExtraBold", 15 * -1)
)

canvas.create_text(
    416.0,
    690.0,
    anchor="nw",
    text="Sensor to kPa",
    fill="#3B3D70",
    font=("Inter SemiBold", 14 * -1)
)

entry_image_1001 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1001 = canvas.create_image(
    433.5,
    567.5,
    image=entry_image_1001
)

entry_1001 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_1001.insert(0, sensor_touch_value)

entry_1001.place(
    x=392.0,
    y=550.0,
    width=83.0,
    height=33.0
)


def set_sensor_touch_value(value):
    global sensor_touch_value
    sensor_touch_value = value


button_image_1001 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_1001 = Button(
    image=button_image_1001,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: set_sensor_touch_value(float(entry_1001.get())),
    relief="flat"
)
button_1001.place(
    x=487.0,
    y=550.0,
    width=41.0,
    height=35.0
)

entry_image_1002 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_1002 = canvas.create_image(
    433.5,
    729.5,
    image=entry_image_1002
)
entry_1002 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_1002.insert(0, sensor_to_pressure_factor)
entry_1002.place(
    x=392.0,
    y=712.0,
    width=83.0,
    height=33.0
)


def set_sensor_to_pressure_factor(value):
    global sensor_to_pressure_factor
    sensor_to_pressure_factor = value


button_image_1002 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_1002 = Button(
    image=button_image_1002,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: set_sensor_to_pressure_factor(float(entry_1002.get())),
    relief="flat"
)
button_1002.place(
    x=487.0,
    y=712.0,
    width=41.0,
    height=35.0
)

entry_image_1003 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_1003 = canvas.create_image(
    433.5,
    666.5,
    image=entry_image_1003
)
entry_1003 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_1003.insert(0, deformation)
entry_1003.place(
    x=392.0,
    y=649.0,
    width=83.0,
    height=33.0
)

entry_image_10031 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_10031 = canvas.create_image(
    583.5,
    666.5,
    image=entry_image_1003
)
entry_10031 = Entry(
    bd=0,
    bg="#DAE4FF",
    fg="#000716",
    highlightthickness=0
)
entry_10031.insert(0, str("{:.3f}".format(deformation_to_mm)))
entry_10031.place(
    x=542.0,
    y=649.0,
    width=83.0,
    height=33.0
)


def set_deformation_val(value):
    global deformation
    deformation = value
    deformation_to_mm = (
        (0.0179*deformation*deformation) + (0.241*deformation))*1.3
    entry_10031.delete(0, tk.END)
    entry_10031.insert(0, str("{:.3f}".format(deformation_to_mm)))


button_image_1003 = PhotoImage(
    file=relative_to_assets("button_3.png"))
button_1003 = Button(
    image=button_image_1003,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: set_deformation_val(float(entry_1003.get())),
    relief="flat"
)
button_1003.place(
    x=487.0,
    y=649.0,
    width=41.0,
    height=35.0
)

###########################


####### GUI #######


servo_thread = threading.Thread(target=servoRunLoop)
servo_thread.daemon = True
servo_thread.start()


gripper_thread = threading.Thread(target=runGripperLoop)
gripper_thread.daemon = True
gripper_thread.start()


sensor_thread = threading.Thread(target=runSensorLoop)
sensor_thread.daemon = True
sensor_thread.start()


window.resizable(False, False)
window.mainloop()
