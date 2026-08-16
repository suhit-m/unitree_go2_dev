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
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder

import threading

from typing import List

localization_server_url = "http://192.168.1.194:12345/OptiTrackRestServer"

resolution_multiplier = 10

odom_x = None
odom_y = None
odom_z = None

odom_yaw = None
odom_pitch = None
odom_roll = None

optitrack_x = None
optitrack_y = None
optitrack_yaw = None

def callBackFunc(message):

    x = message.position[0]
    y = message.position[1]
    z = message.position[2]

    w = message.imu_state.quaternion[0]
    a = message.imu_state.quaternion[1]
    b = message.imu_state.quaternion[2]
    c = message.imu_state.quaternion[3]
    q_array = [w, a, b, c]
    q = R.from_quat(q_array)

    # extrinsic euler angle; good for odom.
    attitude_buf = q.as_euler("xyz", True)
    
    yaw = attitude_buf[0]
    pitch = attitude_buf[1]
    roll = attitude_buf[2]
    

    print(f"x: {x}")
    print(f"y: {y}")
    print(f"z: {z}")
    print(f"yaw: {yaw}")
    print(f"pitch: {pitch}")
    print(f"roll: {roll}")

    # print(f"x: {message.position[0]}")
    # print(f"y: {message.position[1]}")
    # print(f"z: {message.position[2]}")

def start_motive():
    
    motive = subprocess.Popen(["cmd.exe", "/c", "start", "", "Motive_Best_Calibration.lnk"], cwd= "/mnt/c/Users/CUBLab/Desktop",stdout=subprocess.PIPE)


    # motive = subprocess.Popen("C:/Users/CUBLab/Desktop/Motive_Updated_Calibration_Dimmed_1_8_2026.lnk",stdout=subprocess.PIPE,shell=True)

    time.sleep(10)

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

            # 0 radians is positive x; 0 to 2pi counter clockwise
            optitrack_yaw = coordinates[3]
        except Exception as e:
            print(f"{object_name} cannot be found!")

        
        

def get_vision_localization():
    objects = get_objects()
    name = list(objects.keys())[i]
    for i in range(len(objects.keys())-1,-1,-1):
        if objects[name] != "untracked":
                values = objects[name].split(',')
                if len(values) != 7: continue
                x = float(values[1]); y = float(values[2]); w = float(values[6]); l = float(values[5])
                angle = 0

def create_arena_matrix():
    # 5 meters by 5 meters (tiles are appx 4.8 by 4.8)
    # 0,0 is by ladder
    
    matrix = np.ones((5*resolution_multiplier, 5*resolution_multiplier, 1*resolution_multiplier), dtype=np.int8)
    arena = Grid(matrix = matrix) 
    return arena



def pathfind(arena: Grid, startpoint: Grid.node, endpoint: Grid.node):
    # Create an instance of the A* finder with diagonal movement allowed
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(startpoint, endpoint, arena)

    # Path will be a list with all the waypoints as nodes
    # Convert it to a list of coordinate tuples
    # path = [p.identifier for p in path]
    return path



def get_current_pos_node(arena):
    #standing height
    # nodes must be in centimeters!
    x = int(optitrack_x*resolution_multiplier)+int(2.5*resolution_multiplier)-1
    y = int(optitrack_y*resolution_multiplier)+int(2.5*resolution_multiplier)-1
    z = int(0.4*resolution_multiplier)

    coords = [x, y, z]
    for i in range(3):
        if coords[i] >= 5*resolution_multiplier:
            coords[i] = (5*resolution_multiplier)-1
        elif coords[i] < 0:
            coords[i] = 0

    # print(f"x: {coords[0]}")
    # print(f"y: {coords[1]}")
    # print(f"z: {coords[2]}")
    
    return arena.node(coords[0], coords[1], coords[2])

def create_node_w_meters(arena, xm, ym, zm):
    x = int(xm*resolution_multiplier)+int(2.5*resolution_multiplier)-1
    y = int(ym*resolution_multiplier)+int(2.5*resolution_multiplier)-1
    z = int(zm*resolution_multiplier)
    coords = [x, y, z]
    for i in range(3):
        if coords[i] >= 5*resolution_multiplier:
            coords[i] = (5*resolution_multiplier)-1
        elif coords[i] < 0:
            coords[i] = 0
        

    
    return arena.node(coords[0], coords[1], coords[2])

def followpath(arena: Grid, path: List[Grid.node], sportclient: SportClient):

    for pathpoint in path:
        current_pos_x = get_current_pos_node(arena).identifier[0]
        current_pos_y = get_current_pos_node(arena).identifier[1]
        next_pos_x = pathpoint.identifier[0]
        next_pos_y = pathpoint.identifier[1]
        
        x_diff = next_pos_x - current_pos_x
        y_diff = next_pos_y - current_pos_y
        
        while(np.linalg.norm(np.array([x_diff, y_diff])) > 0.01*resolution_multiplier):
            current_pos_x = get_current_pos_node(arena).identifier[0]
            current_pos_y = get_current_pos_node(arena).identifier[1]
            current_yaw = optitrack_yaw % (2 * np.pi) #radians -pi to pi
            next_pos_x = pathpoint.identifier[0]
            next_pos_y = pathpoint.identifier[1]
            
            x_diff = next_pos_x - current_pos_x
            y_diff = next_pos_y - current_pos_y
            next_yaw = math.atan2(y_diff, x_diff)
            yaw_diff = next_yaw - current_yaw

            # dir_vec_2d = np.array([x_diff, y_diff]) 
            # dir_vec_mag = np.linalg.norm(dir_vec_2d)
            

            # if dir_vec_mag < 0.01*resolution_multiplier:
            #     continue;
            
            x_multiplier = .25
            y_multiplier = .25
            yaw_multiplier = 0.1
            #	vx: Range [-2.5~3.8] (m/s); vy: Range [-1.0~1.0] (m/s); vyaw: Range [-4~4] (rad/s).
            # assuming x diff is about 10 and max 15, 
            vx = (x_diff) * x_multiplier
            if vx > 3.8:
                vx = 3.8
            elif vx < -2.5:
                vx = -2.5
            vy = (y_diff) * y_multiplier
            if vy > 1:
                vy = 1
            elif vy < -1:
                vy = -1
            vyaw = yaw_diff * yaw_multiplier
            if vyaw > 4:
                vyaw = 4
            elif vyaw < -4:
                vyaw = -4

            print("")
            print(f"current_x: {current_pos_x}")
            print(f"current_y: {current_pos_y}")
            print(f"current_yaw: {current_yaw}")
            print("")
            # print(f"next_pos_x: {next_pos_x}")
            # print(f"next_pos_y: {next_pos_y}")
            # # print(f"dirvec mag: {dir_vec_mag}")
            # print("")
            # print(f"x_diff: {x_diff}")
            # print(f"y_diff: {y_diff}")
            # print(f"yaw_diff: {yaw_diff}")
            # print("")

            # print(f"vx: {vx}")
            # print(f"vy: {vy}")
            # print(f"vyaw: {vyaw}")
            # print("")

        
            sport_client_move_globally(sportclient, vx, vy, vyaw, current_yaw)
            # time.sleep(0.1)
    sportclient.Move(0,0,0)

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

    start_motive()

    

    start_localization_server()

    time.sleep(2)

    

    ChannelFactoryInitialize(0, "eth1")
    # creates a singleton channel factory, also automatically done whenever a channel is created, but here you can set the network its on

    suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # rt means real time, lf means low freq

    # suber.Init(callBackFunc)



    sport_client = SportClient()  
    # sport_client = ChannelPublisher("rt/sport_client")
    sport_client.Init()

    # sport_client.Move(0,0,0)
    session = create_session()

    thread = threading.Thread(target=parse_objects, args=("GO2-001", session))
    thread.start()


    # while(True):
    #     # parse_objects("GO2-001", get_objects(session))
    #     try:
    #         print("\033[H\033[J", end="") 
    #         print(f"X: {optitrack_x}")
    #         print(f"Y: {optitrack_y}")
    #         print(f"Angle: {math.degrees(optitrack_yaw)}")
    #     except Exception as e:
    #         print("oops")
    time.sleep(2)

    arena = create_arena_matrix()

    # MUST BE IN CM
    # last num is irrelevant, 2d for now
    # while(True):
    #     try:
    #         print(f"currentpos: {get_current_pos_node(arena).identifier}")
            
    #     except Exception as e:
    #         # print("\033[H\033[J", end="")
    #         print(f"whyyyyy: {e}")

        # try: 
        #     print(f"endpos: {create_node_w_meters(arena, 1.5, 1.5, 0.4).identifier}")
        # except Exception as e:
            # print("why2")
    testpath = pathfind(arena, get_current_pos_node(arena), create_node_w_meters(arena, 1, 1, 0.4))
    followpath(arena, testpath, sport_client)

    time.sleep(3600)


    # 0.24
    # 0.16
    # 0.31

    # -1.4
    # 0.57
    # 0.31 


       
    # sport_client.Init()

   
    # time.sleep(3)
