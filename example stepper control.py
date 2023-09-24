import tkinter as tk
from tkinter import ttk
import matplotlib
import matplotlib.pyplot as plt
import random
import ctypes
# from ctypes import windll
from tkinter import HORIZONTAL
import pyfirmata
import threading
from threading import Timer
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
import sys
import matplotlib.animation as animation
from idlelib.tooltip import Hovertip
from time import sleep
import numpy as np
import csv


# BEGIN REPEAT TIMER DEFINITION

# Repeat timer class to allow a function to after a certain amount of time

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

# END REPEAT TIMER DEFINITION


# BEGIN PLOTTING FUNCTION

# Plots and updates the data on the GUI plot

def animate(i):

    global t, p, d, time, interval, flagData, pressurePlot, distancePlot, sensorPin

    t.append(time)
    try:
        p.append(calcPressure(sensorPin.read()))
        d.append(calcPosition(servoPin.read()))
    except:
        print("Random Data")
        p.append(random.randint(3, 10))
        d.append(random.randint(15, 30))

    pressurePlot.clear()
    distancePlot.clear()
    distancePlot.set_title("Distance Between Jaws")
    distancePlot.set_ylabel("distance (mm)")
    distancePlot.set_xlabel("time (s)")

    pressurePlot.set_title("Pressure")
    pressurePlot.set_xlabel("time (s)")
    pressurePlot.set_ylabel("Pressure (kPa)")
    pressurePlot.plot(t, p)
    distancePlot.plot(t, d)

    time = time + interval/1000.0

# END PLOTTING FUNCTION


# BEGIN MOVEMENT FUNCTIONS

# These are the functions that move the motors

def motor1moveClock():

    global board

    board.digital[4].write(int(0))
    board.digital[3].write(int(1))

    sleep(.0001)

    board.digital[3].write(int(0))


def motor1moveCounter():

    global board

    board.digital[4].write(int(1))
    board.digital[3].write(int(1))

    sleep(delay)

    board.digital[3].write(int(0))


def motor2moveUp():

    global board

    board.digital[direction2].write(int(0))
    board.digital[step2].write(int(1))

    sleep(delay)

    board.digital[step2].write(int(0))


def motor2moveDown():

    global board

    board.digital[direction2].write(int(1))
    board.digital[step2].write(int(1))

    sleep(delay)

    board.digital[step2].write(int(0))


def motor3moveUp():

    global board

    board.digital[direction3].write(int(0))
    board.digital[step3].write(int(1))

    sleep(delay)

    board.digital[step3].write(int(0))


def motor3moveDown():

    global board

    board.digital[direction3].write(int(1))
    board.digital[step3].write(int(1))

    sleep(delay)

    board.digital[step3].write(int(0))


def motor4moveLeft():

    global board

    board.digital[direction4].write(int(0))
    board.digital[step4].write(int(1))

    sleep(delay)

    board.digital[step4].write(int(0))


def motor4moveRight():

    global board

    board.digital[direction4].write(int(1))
    board.digital[step4].write(int(1))

    sleep(delay)

    board.digital[step4].write(int(0))


def motor5moveLeft():

    global board

    board.digital[direction5].write(int(0))
    board.digital[step5].write(int(1))

    sleep(delay)

    board.digital[step5].write(int(0))


def motor5moveRight():

    global board

    board.digital[direction5].write(int(1))
    board.digital[step5].write(int(1))

    sleep(delay)

    board.digital[step5].write(int(0))


# Function to close the servo jaws:
def closeJaws():

    # Define the pulse width values for the 0 and 180 degree positions of the servo
    # These values should be adjusted to match the pulse width range supported by the servo model you are using
    pulse_width_0_degrees = 1000
    pulse_width_180_degrees = 3000

    # Define the delay time between each step in the angle change loop
    delay_time = 0.03

    # Define the minimum angle change for each step in the loop
    angle_change = 0.1

    curPos = servoPin.read()

    while True:
        if float(sensorPin.read()*1023*.058) < strainMax:
            # Increment the current position of the servo by the angle change value
            curPos = curPos + angle_change

            # Convert the float angle value to a pulse width value for the servo
            newAngle = int(curPos / 180.0 * (pulse_width_180_degrees -
                           pulse_width_0_degrees) + pulse_width_0_degrees)

            # Write the pulse width value to the servo to move it to the desired angle position
            servoPin.write_microseconds(newAngle)
        else:
            break

        sleep(delay_time)


### END MOTOR MOVEMENT FUNCTIONS ###

### BEGIN CALCULATE DATA FUNCTIONS ###

# Updating these functions updates the distance and pressure calculation formulas everywhere

def calcPosition(x):
    position = .9256*(x*-1.0893 + 61.922)
    return position


def calcPressure(x):
    pressure = x*1023*.058
    return x


### END CALCULATE DATA FUNCTIONS ###


### BEGIN YOUNG'S MODULUS DATA COLLECTION FUNCTION ###

# This function collects and stores the required data to calculate the Young's Modulus as a .txt file

def ymT():

    E1star = 27000  # This is the bulk modulus

    def exportYM():  # This function exports the data to a txt file

        dataCollected = list([deformations, pressures, youngsMods])
        fields = ['Square root of Deformations (m^.5)', 'Pressures (Pa)']

        with open('Youngs Modulus GUI Data.txt', 'w') as f:

            write = csv.writer(f)
            write.writerow(fields)
            write.writerow(dataCollected)

    global strainMax

    thresholdP = strainMax
    flag = False  # Flag indicating whether the robot has touched the object yet or not
    i = 0

    deformations = []  # Store deformations^.5 here
    pressures = []  # Store pressures in Pa here

    diameter = 0.0

    curPosition = calcPosition(feedbackPin.read())

    while curPosition > .85*diameter:  # Adjust the factor that diameter is multiplied by to change the amount of strain applied

        i += 1  # increment position

        servoPin.write(i)
        curP = calcPressure(sensorPin.read())
        appliedP = curP - thresholdP

        if (curP > thresholdP) & (flag == False):

            # Diameter is printed to the command window after it is measured
            diameter = calcPosition(servoPin.read())
            print("Diameter: ")
            print(diameter)
            flag = True

        elif flag == True:  # This happens if the robot is in contact with the object
            d = (diameter - calcPosition(servoPin.read()))/2.0
            d_12 = (d/1000.0)**(1.0/2.0)
            deformations = np.append(deformations, [d_12])
            pressures = np.append(pressures, [appliedP*1000.0])

        sleep(2)

        curPosition = calcPosition(servoPin.read())

    ymWin = tk.Tk()
    ymWin.title("Young's Modulus Calculation")
    ymWin.geometry('1500x800')

    exportButton = tk.Button(
        ymWin, text="Export Data Points", command=exportYM)
    exportButton.grid(row=16, column=6)

    ymWin.mainloop()

### END YOUNG'S MODULUS DATA COLLECTION FUNCTION ###

### BEGIN EXPORT PRESSURE AND DISTANCE DATA FUNCTION ###


def export():

    dataCollected = list([t, p, d])
    fields = ['Time (s)', 'Pressure (kPa)', 'distance between jaws (mm)']

    with open('Main GUI Data', 'w') as f:

        write = csv.writer(f)

        write.writerow(fields)
        write.writerow(dataCollected)

### END EXPORT PRESSURE AND DISTANCE DATA FUNCTION ###


# BEGIN BUTTON FUNCTIONS

# These functions are called directly by the buttons. Typically these start a thread that carries out the actual function.

# In the case of the motor buttons a thread is started repeating the motor movement loop until the button is unpressed.

def Clock1(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor1moveClock)
        movemotor.start()


def Counter1(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor1moveCounter)
        movemotor.start()


def Up2(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor2moveUp)
        movemotor.start()


def Down2(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor2moveDown)
        movemotor.start()


def Down3(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor3moveUp)
        movemotor.start()


def Up3(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor3moveDown)
        movemotor.start()


def Clock4(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor4moveLeft)
        movemotor.start()


def Counter4(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor4moveRight)
        movemotor.start()


def Down5(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor5moveLeft)
        movemotor.start()


def Up5(event=0):

    global movemotor

    if movemotor.is_alive():
        movemotor.cancel()
    else:
        movemotor = RepeatTimer(delayStop, motor5moveRight)
        movemotor.start()

# Opens the jaws:


def openJaws():

    pos = servoPin.read()

    servoPin.write(0)

# Updates the threshold pressure value for sensing contact


def pressureUp():

    global strainMax

    try:
        strainMax = float(maxStrain.get())
        maxStrain.delete(0, tk.END)
        strainVar.set(str(strainMax))

    except:
        maxStrain.delete(0, tk.END)
        tk.messagebox.showerror('Value Error', 'Invalid value entered')


def Exit():

    win.destroy()
    sys.exit()

# Reset data collection space


def refresh():

    global t, p, d, time

    t = []
    p = []
    d = []
    time = 0.0


def closeJawsButton():

    jawsThread = threading.Thread(target=closeJaws)
    jawsThread.start()


def ymTButton():
    ymThread = threading.Thread(target=ymT)
    ymThread.start()

# END BUTTON FUNCTIONS


if __name__ == "__main__":

    # Increase Resolution:
    # windll.shcore.SetProcessDpiAwareness(1)

    global board, movemotor, sensorPin, servoPin, it, strainMax

    strainMax = 2.32

    # It is important to note that there are two Arduino boards required one for the movement of the robot and another
    # for the movement of the jaws and the collection of data. If you only would like to use one of these functions you
    # can connect just the Arduino board with the functions you are interested in. The program will thrown an error but
    # it does not effect functionallity

    try:
        board = pyfirmata.Arduino('COM4')
        it = pyfirmata.util.Iterator(board)
        it.start()

    except:
        tk.messagebox.showerror('Arduino 1 Error', 'Arduino not found')

    try:
        board2 = pyfirmata.Arduino('COM7')
        it2 = pyfirmata.util.Iterator(board2)
        it2.start()
    except:
        tk.messagebox.showerror('Arduino 2 Error', 'Arduino not found')

    # BEGIN PIN VARIABLE ASSIGNMENTS

    delay = 1e-5
    delayStop = 2e-6

    step1 = 3
    direction1 = 4

    step2 = 5
    direction2 = 6

    step3 = 7
    direction3 = 8

    step4 = 9
    direction4 = 10

    step5 = 11
    direction5 = 12

    # END PIN VARIABLE ASSIGNMENTS

    # this is required to initialize the movemotor variable which is used in threads
    movemotor = RepeatTimer(delayStop, Up5)

    # Change the number in the pin variable statements to change the associated Arduino pin
    try:
        sensorPin = board2.get_pin('a:0:o')
    except:
        tk.messagebox.showerror('Pressure Sensor Error',
                                'Unable to find and initialize sensor pin')

    try:
        servoPin = board2.get_pin('d:5:s')
    except:
        tk.messagebox.showerror(
            'Servo Error', 'Unable to find and initialize servo pin')

    try:
        feedbackPin = board2.get_pin('a:2:o')
    except:
        tk.messagebox.showerror('Feeback Pin Error',
                                'Unable to find and initialize feedback pin')

    win = tk.Tk()
    win.title("NIFE Soft Robot GUI")
    win.geometry('1600x1000')

    # BEGIN MOTOR CONTROL FORMATTING

    # Variables for consistent formatting
    res = 0.01
    digits = 5
    scalePadx = 40
    inputWidth = 20
    inputPady = 8
    butHeight = 1
    butPady = 5

    motorFrame = tk.Frame(win)
    motorFrame.grid(row=3, column=0, rowspan=4)

    upArrow = tk.PhotoImage(file='ArrowUp.png')
    downArrow = tk.PhotoImage(file='ArrowDown.png')
    rightArrow = tk.PhotoImage(file='ArrowRight.png')
    leftArrow = tk.PhotoImage(file='ArrowLeft.png')
    clockArrow = tk.PhotoImage(file='ClockwiseArrow.png')
    counterArrow = tk.PhotoImage(file='CounterClockwiseArrow.png')

    # Motor 1

    motor1Label = tk.Label(motorFrame, text="Motor 1")
    motor1Label.grid(row=0, column=2, columnspan=4)

    UpButton1 = tk.Button(motorFrame, image=clockArrow)
    UpButton1.grid(row=1, column=2, pady=butPady)

    UpButton1.bind('<ButtonPress-1>', Clock1)
    UpButton1.bind('<ButtonRelease-1>', Clock1)

    DownButton1 = tk.Button(motorFrame, image=counterArrow)
    DownButton1.grid(row=1, column=4, pady=butPady)

    DownButton1.bind('<ButtonPress-1>', Counter1)
    DownButton1.bind('<ButtonRelease-1>', Counter1)

    # Motor 2

    motor2Label = tk.Label(motorFrame, text="Motor 2 Controls")
    motor2Label.grid(row=2, column=2, columnspan=4)

    directionButton2 = tk.Button(motorFrame, image=upArrow)
    directionButton2.grid(row=3, column=2, pady=butPady)

    direction2Button2 = tk.Button(motorFrame, image=downArrow)
    direction2Button2.grid(row=3, column=4, pady=butPady)

    directionButton2.bind('<ButtonPress-1>', Up2)
    directionButton2.bind('<ButtonRelease-1>', Up2)

    direction2Button2.bind('<ButtonPress-1>', Down2)
    direction2Button2.bind('<ButtonRelease-1>', Down2)

    # Motor 3

    motor3Label = tk.Label(motorFrame, text="Motor 3 Controls")
    motor3Label.grid(row=4, column=2, columnspan=4)

    directionButton3 = tk.Button(motorFrame, image=upArrow)
    directionButton3.grid(row=5, column=2, pady=butPady)

    direction2Button3 = tk.Button(motorFrame, image=downArrow)
    direction2Button3.grid(row=5, column=4, pady=butPady)

    directionButton3.bind('<ButtonPress-1>', Up3)
    directionButton3.bind('<ButtonRelease-1>', Up3)

    direction2Button3.bind('<ButtonPress-1>', Down3)
    direction2Button3.bind('<ButtonRelease-1>', Down3)

    # Motor 4
    motor4Label = tk.Label(motorFrame, text="Motor 4 Controls")
    motor4Label.grid(row=6, column=2, columnspan=4)

    directionButton4 = tk.Button(motorFrame, image=clockArrow)
    directionButton4.grid(row=7, column=2, pady=butPady)

    direction2Button4 = tk.Button(motorFrame, image=counterArrow)
    direction2Button4.grid(row=7, column=4, pady=butPady)

    directionButton4.bind('<ButtonPress-1>', Clock4)
    directionButton4.bind('<ButtonRelease-1>', Clock4)

    direction2Button4.bind('<ButtonPress-1>', Counter4)
    direction2Button4.bind('<ButtonRelease-1>', Counter4)

    # Motor 5
    motor5Label = tk.Label(motorFrame, text="Motor 5 Controls")
    motor5Label.grid(row=8, column=2, columnspan=4)

    directionButton5 = tk.Button(motorFrame, image=upArrow)
    directionButton5.grid(row=9, column=2, pady=butPady)

    direction2Button5 = tk.Button(motorFrame, image=downArrow)
    direction2Button5.grid(row=9, column=4, pady=butPady)

    directionButton5.bind('<ButtonPress-1>', Up5)
    directionButton5.bind('<ButtonRelease-1>', Up5)

    direction2Button5.bind('<ButtonPress-1>', Down5)
    direction2Button5.bind('<ButtonRelease-1>', Down5)

    # Jaws Motor

    jawButton = tk.Button(win, text="Close Jaws", command=closeJawsButton)
    jawButton.grid(row=8, column=0)

    jawButton2 = tk.Button(win, text="Open Jaws", command=openJaws)
    jawButton2.grid(row=9, column=0)

    youngsModulusTestButton = tk.Button(
        win, text="Determine Young's Modulus", command=ymTButton)
    youngsModulusTestButton.grid(row=10, column=0)

    exportDataButton = tk.Button(win, text='Export Data', command=export)
    exportDataButton.grid(row=12, column=0)

    # END MOTOR CONTROL FORMATTING

    # BEGIN START & END BUTTON

    startButton = tk.Button(
        win, text="Restart Data Collection", command=refresh)
    startButton.grid(row=1, column=0, pady=10)

    # endButton = tk.Button(win, text = "Stop", command = exit)
    # endButton.grid(row = 2, column = 0)

    # END START & END BUTTON

    # BEGIN FEATURES

    maxStrain = tk.Entry(win, width=20)
    maxStrain.grid(row=0, column=1, sticky="S")

    strainButton = tk.Button(
        win, text="Update Threshold Pressure Value", command=pressureUp)
    strainButton.grid(row=1, column=1)

    strainVar = tk.StringVar()

    strainVar.set(str(strainMax))

    strainInfoValue = tk.Label(win, textvariable=strainVar)
    strainInfoValue.grid(row=1, column=6)

    strainInfo = tk.Label(win, text="Current Pressure Sensor Threshold (kPa):")
    strainInfo.grid(row=0, column=6)

    # END FEATURES

    # BEGIN PLOTS

    f = Figure(figsize=(4.85, 5), dpi=100)

    pressurePlot = f.add_subplot(211)
    distancePlot = f.add_subplot(212)

    f.subplots_adjust(wspace=.8)

    f.subplots_adjust(hspace=.5)

    canvas = FigureCanvasTkAgg(f, master=win)
    canvas.draw()
    canvas.get_tk_widget().grid(row=6, column=1, columnspan=6, rowspan=6)

    t = []
    p = []
    d = []
    time = 0.0
    interval = 2000.0

    plots = animation.FuncAnimation(f, animate, interval=interval)

    # END PLOTS

    # BEGIN TOOLTIPS

    tpStrain = Hovertip(
        maxStrain, 'Enter the maximum\nstrain value to\nbe applied')
    # tpStop = Hovertip(endButton, 'Close the GUI')

    # END TOOLTIPS

    # BEGIN WIP
    # END WIP

    win.mainloop()
