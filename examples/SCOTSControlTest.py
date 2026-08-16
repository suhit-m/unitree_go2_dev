import time, sys, subprocess, json, requests
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
import numpy as np
import random
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder


import SymbolicSetPython as scots

import threading

from typing import List

localization_server_url = "http://192.168.1.194:12345/OptiTrackRestServer"

optitrack_x = None
optitrack_y = None
optitrack_yaw = None


def start_motive():
    
    motive = subprocess.Popen(["cmd.exe", "/c", "start", "", "Motive_Best_Calibration.lnk"], cwd= "/mnt/c/Users/CUBLab/Desktop",stdout=subprocess.PIPE)


    # motive = subprocess.Popen("C:/Users/CUBLab/Desktop/Motive_Updated_Calibration_Dimmed_1_8_2026.lnk",stdout=subprocess.PIPE,shell=True)

    time.sleep(2)

    return motive

def start_localization_server():
    localization_server = subprocess.Popen(["cmd.exe", "/c", "start_admin.bat"], cwd = "/mnt/d/Workspace/OptiTrackRESTServer")
    return localization_server

def create_session():
    session = requests.Session()
    return session

def get_objects(session):
    try:
        return json.loads(session.get(localization_server_url).text)
    except Exception as e:
        print("unable to retrieve from localization server: "+str(e))

def parse_objects(object_name, session):
    while(True):
        try:
            objects = get_objects(session)
            coordinates = [float(x) for x in objects[object_name].split(',')]
            global optitrack_y
            global optitrack_x 
            global optitrack_yaw

            # positive y is towards other room, negative y is to kitchen
            optitrack_y = -coordinates[1]
            
            # positive x is towards computing server, negative x is to whiteboard
            optitrack_x = coordinates[2] 

            # 0 radians is positive x; -pi to pi counter clockwise
            optitrack_yaw = coordinates[3]

            
            # print(f"x: {optitrack_x}")
            # print(f"y: {optitrack_y}")
            # print(f"yaw: {optitrack_yaw}")

        except Exception as e:
            print(f"{object_name} cannot be found!")



        
def sport_client_move_globally(sportclient: SportClient, vx_global, vy_global, vyaw, current_yaw_rad):
    # 1. Create the rotation components
    c = math.cos(current_yaw_rad)
    s = math.sin(current_yaw_rad)
    
    # 2. Rotate global velocity vector into local frame
    # Standard 2D rotation matrix: [ c  s ] [ vx ]
    # Inverse (Global -> Local):   [ c  s ] [ vx ]  (Transpose of rotation matrix)
    #                              [-s  c ] [ vy ]
    
    vx_local = vx_global * c + vy_global * s
    vy_local = -vx_global * s + vy_global * c
        
        # 3. Move the client
    sportclient.Move(vx_local, vy_local, vyaw)



if __name__ == "__main__":

    # start_motive()

    # start_localization_server()

    # time.sleep(2)

    ChannelFactoryInitialize(0, "eth1")

    # suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)

    sport_client = SportClient()  

    sport_client.Init()

    session = create_session()

    thread = threading.Thread(target=parse_objects, args=("GO2-001", session))
    thread.start()


    mgr  = scots.Cudd()
    ctrl = scots.SymbolicSet(mgr, "scots_dev/go2_controller.bdd")
    target = scots.SymbolicSet(mgr, "scots_dev/go2_target.bdd")

    print("Check 1")

    while optitrack_x is None or optitrack_y is None or optitrack_yaw is None:
        print("Waiting for OptiTrack data...")
        time.sleep(0.1)

    # while(not target.isElement([float(optitrack_x), float(optitrack_y), float(optitrack_yaw)])):

    while(True):
    #     print(f"optitrack_x: {optitrack_x}")
    #     print(f"optitrack_y: {optitrack_y}")
    #     print(f"optitrack_yaw: {optitrack_yaw}")
    #     time.sleep(0.2)
    # while(not target.isElement([optitrack_x, optitrack_y, optitrack_yaw])):
        # query inputs for state [0, 0, 0]
        # ind = {0,1,2} means first 3 dims are the state
        inputs = ctrl.setValuedMap([optitrack_x, optitrack_y, optitrack_yaw], [0, 1, 2])


        print(f"optitrack_x: {optitrack_x}")
        print(f"optitrack_y: {optitrack_y}")
        print(f"optitrack_yaw: {optitrack_yaw}")

        if len(inputs) == 0:
            print("State outside winning set!")
            break

        # print("Check 2")

        # inputs is a list of [vx, vy, vyaw] vectors
        # Random Input
        # bound = len(inputs)-1
        # chosen_input_idx = random.randint(0, bound)
        # chosen_input = inputs[chosen_input_idx] 

        # Minimum Norm; conservative movement
        # norms = [sum(u[i]**2 for i in range(3)) for u in inputs]
        # chosen_input = inputs[norms.index(min(norms))]

        # Closest to target centre
        target_c = [-1.25, 0.0]
        chosen_input = min(inputs, key=lambda u: 
            (optitrack_x + (u[0]*math.cos(optitrack_yaw) - u[1]*math.sin(optitrack_yaw))*0.3 - target_c[0])**2 + 
            (optitrack_y + (u[0]*math.sin(optitrack_yaw) + u[1]*math.cos(optitrack_yaw))*0.3 - target_c[1])**2)

        
        

        print(f"x_input: {chosen_input[0]}")
        print(f"y_input: {chosen_input[1]}")
        print(f"yaw_input: {chosen_input[2]}")
        
        sport_client_move_globally(sport_client, -chosen_input[0], chosen_input[1], chosen_input[2], optitrack_yaw)
        # sport_client.Move(chosen_input[0], -chosen_input[1], chosen_input[2])
   

        time.sleep(0.3)

    while(True):
        print("Reached Target Set!!!")
        time.sleep(1)

    



