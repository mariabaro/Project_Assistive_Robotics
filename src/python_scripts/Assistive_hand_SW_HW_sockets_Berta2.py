from math import radians
import os
import time
import socket
from robodk.robolink import *   # API de RoboDK
from robodk.robomath import *   # Funciones de matemáticas de RoboDK

# ------------------------------
# Cargar el proyecto RoboDK
# ------------------------------
relative_path = "src/roboDK/Assistive_UR5e_Maria.rdk"
absolute_path = os.path.abspath(relative_path)
RDK = Robolink()
RDK.AddFile(absolute_path)

# ------------------------------
# Configuración del robot
# ------------------------------
robot = RDK.Item("UR5e")
base = RDK.Item("UR5e Base")
gripper = RDK.Item("Hand")   # asegúrate que coincida con el nombre en tu RDK

robot.setPoseFrame(base)
robot.setPoseTool(gripper)
robot.setSpeed(20)

# ------------------------------
# Targets definidos en RoboDK
# ------------------------------
init = RDK.Item("Init")
pregrasp = RDK.Item("PreGrasp")
grasp = RDK.Item("Grasp")
pregive = RDK.Item("PreGive")
give = RDK.Item("Give")

# ------------------------------
# Conexión al robot real por socket
# ------------------------------
ROBOT_IP = "192.168.1.5"   # Cambia a la IP de tu UR5e
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
def go_init():
    print("Going to INIT")
    robot.MoveJ(init, True)
    if robot_is_connected:
        cmd = f"movej([{','.join(map(str, [radians(-57.835746), radians(-65.391528), radians(-107.167117), radians(172.558645), radians(-57.835746), radians(0.0)]))}], {accel_mss}, {speed_ms},{timel},0)"
        send_ur_script(cmd)
        receive_response(timel)

def pick_object():
    print("Going to PreGrasp")
    robot.MoveL(pregrasp, True)
    robot.MoveL(grasp, True)
    print("Grasp done (simulación: cerrar gripper)")
    # aquí comando al gripper real si lo tienes
    time.sleep(2)  # ⏸ pausa de 3 segundos después del Grasp
    robot.MoveL(pregrasp, True)

def give_object():
    print("Giving object")
    robot.MoveL(pregive, True)
    robot.MoveL(give, True)
    print("Give done")
    # aquí comando de abrir gripper si procede
    time.sleep(2)  # ⏸ pausa de 3 segundos después del Give
    robot.MoveL(pregive, True)

# ------------------------------
# Main
# ------------------------------
def main():
    global robot_is_connected
    robot_is_connected = check_robot_port(ROBOT_IP, ROBOT_PORT)

    go_init()
    pick_object()
    give_object()
    go_init()

    if robot_is_connected:
        robot_socket.close()

if __name__ == "__main__":
    main()
