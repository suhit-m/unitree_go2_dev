import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
from dataclasses import dataclass



def callBackFunc(message):
    print(message.position[0])
    print(message.position[1])
    print(message.position[2])


if __name__ == "__main__":

    ChannelFactoryInitialize(0, "eth1")

    suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # rt means real time, lf means low freq

    suber.Init(callBackFunc)

    # sport_client = SportClient()  
    

       
    # sport_client.Init()

   
    time.sleep(1)
