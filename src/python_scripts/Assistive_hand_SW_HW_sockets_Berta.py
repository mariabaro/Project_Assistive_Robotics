import os
import time
import socket
import tkinter as tk
from tkinter import messagebox
from math import radians, degrees, pi
import numpy as np
from robodk.robolink import *
from robodk.robomath import *

# Load RoboDK project from relative path
relative_path = "src/roboDK/Assistive_UR5e_Sergi.rdk"
absolute_path = os.path.abspath(relative_path)
RDK = Robolink()
RDK.AddFile(absolute_path)

# Robot setup
robot = RDK.Item("UR5e")
base = RDK.Item("UR5e Base")
tool = RDK.Item('Hand')
Bring_glass_target = RDK.Item('Bring_glass')

robot.setPoseFrame(base)
robot.setPoseTool(tool)
robot.setSpeed(20)

# Robot Constants
ROBOT_IP = '192.168.1.5'
ROBOT_PORT = 30002
accel_mss = 1.2
speed_ms = 0.75
blend_r = 0.0
timej = 6
timel = 4

[     0.995588,    -0.064032,     0.068585,   511.685000 ;
      0.062269,    -0.095905,    -0.993441,  -539.157000 ;
      0.070190,     0.993329,    -0.091494,   297.033000 ;
      0.000000,     0.000000,     0.000000,     1.000000 ];


# URScript commands
set_tcp = "set_tcp(p[0.000000, 0.000000, 0.050000, 0.000000, 0.000000, 0.000000])"
movej_Home = f"movej([-35.691, -781.749, 693.748, 76.982, -28.646, -27.158],1.20000,0.75000,{timel},0.0000)"
movej_PreGrasp = f"movej([-2.268404, -1.482966, -2.153143, -2.647089, -2.268404, 0.000000],{accel_mss},{speed_ms},{timel},0.000)"
movel_Grasp = f"movel([-2.268404, -1.663850, -2.294637, -2.324691, -2.268404, 0.000000],{accel_mss},{speed_ms},{timel/2},0.000)"
movel_Drink_Start = f"movel([-2.280779, -1.556743, -2.129529, 5.257071, -1.570796, 2.280779],{accel_mss},{speed_ms},{timel},0.000)"
movel_Drink_End = f"movel([-2.195869, -1.642206, -2.040971, 5.253965, -1.570796, 2.195869],{accel_mss},{speed_ms},{timel/2},0.000)"

# Check robot connection
def check_robot_port(ip, port):
    global robot_socket
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.settimeout(1)
        robot_socket.connect((ip, port))
        return True
    except (socket.timeout, ConnectionRefusedError):
        return False
# Send URScript command
def send_ur_script(command):
    robot_socket.send((command + "\n").encode())

# Wait for robot response
def receive_response(t):
    try:
        print("Waiting time:", t)
        time.sleep(t)
    except socket.error as e:
        print(f"Error receiving data: {e}")
        exit(1)

# Movements
def Init():
    print("Init")
    robot.MoveL(Init_target, True)
    print("Init_target REACHED")
    if robot_is_connected:
        print("Init REAL UR5e")
        send_ur_script(set_tcp)
        receive_response(1)
        send_ur_script(movej_init)
        receive_response(timej)
    else:
        print("UR5e not connected. Simulation only.")

def Hand_shake():
    print("Hand Shake")
    robot.setSpeed(20)
    robot.MoveL(App_shake_target, True)
    robot.setSpeed(100)
    robot.MoveL(Shake_target, True)
    robot.MoveL(App_shake_target, True)
    print("Hand Shake FINISHED")
    if robot_is_connected:
        print("App_shake REAL UR5e")
        send_ur_script(set_tcp)
        receive_response(1)
        send_ur_script(movel_app_shake)
        receive_response(timel)
        send_ur_script(movel_shake)
        receive_response(timel)
        send_ur_script(movel_app_shake)
        receive_response(timel)

def Give_me_5():
    print("Give me 5!")
    robot.setSpeed(20)
    robot.MoveL(App_give5_target, True)
    robot.setSpeed(100)
    robot.MoveL(Give5_target, True)
    robot.MoveL(App_give5_target, True)
    print("Give me 5! FINISHED")
    if robot_is_connected:
        print("Give5 REAL UR5e")
        send_ur_script(set_tcp)
        receive_response(1)
        send_ur_script(movel_app_give5)
        receive_response(timel)
        send_ur_script(movel_give5)
        receive_response(timel)
        send_ur_script(movel_app_give5)
        receive_response(timel)

# Confirmation dialog to close RoboDK
def confirm_close():
    root = tk.Tk()
    root.withdraw()
    response = messagebox.askquestion(
        "Close RoboDK",
        "Do you want to save changes before closing RoboDK?",
        icon='question'
    )
    if response == 'yes':
        RDK.Save()
        RDK.CloseRoboDK()
        print("RoboDK saved and closed.")
    else:
        RDK.CloseRoboDK()
        print("RoboDK closed without saving.")

# Main function
def main():
    global robot_is_connected
    robot_is_connected = check_robot_port(ROBOT_IP, ROBOT_PORT)
    Init()
    Hand_shake()
    Give_me_5()
    if robot_is_connected:
        robot_socket.close()

# Run and close
if __name__ == "__main__":
    main()
    #confirm_close()
