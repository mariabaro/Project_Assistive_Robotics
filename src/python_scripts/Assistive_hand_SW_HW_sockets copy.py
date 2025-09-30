from math import radians
import os
import time
import socket
from robodk.robolink import *
from robodk.robomath import *

# ------------------------------
# Cargar el proyecto RoboDK
# ------------------------------
relative_path = "src/roboDK/Assistive_UR5e_Sergi.rdk"
absolute_path = os.path.abspath(relative_path)
RDK = Robolink()
RDK.AddFile(absolute_path)

# ------------------------------
# Configuración del robot
# ------------------------------
robot = RDK.Item("UR5e")
base = RDK.Item("UR5e Base")
gripper = RDK.Item("Robotiq 2F-85 Gripper (Open)")
robot.setPoseFrame(base)
robot.setPoseTool(gripper)
robot.setSpeed(20)

# ------------------------------
# Targets definidos en RoboDK
# ------------------------------
home = RDK.Item("Home")
pregrasp = RDK.Item("PreGrasp")
grasp = RDK.Item("Grasp")
drink_start = RDK.Item("Drink_Start")
drink_end = RDK.Item("Drink_End")

# ------------------------------
# Conexión al robot real por socket
# ------------------------------
ROBOT_IP = '192.168.1.5'   # <- cambia esto a la IP de tu UR5e
ROBOT_PORT = 30002
accel_mss = 1.2
speed_ms = 0.25
timel = 4

def check_robot_port(ip, port):
    global robot_socket
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.settimeout(1)
        robot_socket.connect((ip, port))
        return True
    except:
        return False

def send_ur_script(command):
    robot_socket.send((command + "\n").encode())

def receive_response(t):
    time.sleep(t)

# ------------------------------
# Funciones de movimiento
# ------------------------------
def go_home():
    print("Going HOME")
    robot.MoveJ(home, True)
    if robot_is_connected:
        cmd = f"movej([{','.join(map(str, [radians(86.846080), radians(-58.600474), radians(53.442740), radians(174.588056), radians(-123.261480), radians(178.112169)]))}], {accel_mss}, {speed_ms},{timel},0)"
        send_ur_script(cmd)
        receive_response(timel)

def pick_glass():
    print("Going to PreGrasp")
    robot.MoveL(pregrasp, True)
    robot.MoveL(grasp, True)
    print("Grasp done (simulación: cerrar gripper)")
    # Aquí deberías añadir comando para cerrar gripper si lo tienes

def drink_motion():
    print("Drinking motion")
    robot.MoveL(drink_start, True)
    robot.MoveL(drink_end, True)
    print("Drink finished")
    # Abrir gripper al final si procede

# ------------------------------
# Main
# ------------------------------
def main():
    global robot_is_connected
    robot_is_connected = check_robot_port(ROBOT_IP, ROBOT_PORT)

    go_home()
    pick_glass()
    drink_motion()
    go_home()

    if robot_is_connected:
        robot_socket.close()

if __name__ == "_main_":
    main()