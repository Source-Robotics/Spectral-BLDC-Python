import Spectral_BLDC as Spectral
import SourceRoboticsToolbox as SourceRoboticsToolbox
import time
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS
from math import pi, sin, cos
from spatialmath import *
from oclock import Timer, loop, interactiveloop
import math
from typing import Union, Any, List, Optional, cast
INTERVAL_S = 0.2


Needed_joint_pos = np.array([10440,13555])
Joint_reduction_ratio = [1, 1] # Reduction ratio we have on our joints

Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM123', bitrate=1000000)

Motor: list[Union[Spectral.SpectralCAN,Spectral.SpectralCAN]] = []
Motor.append(Spectral.SpectralCAN(node_id=0, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=1, communication=Communication1))


Joint: list[Union[SourceRoboticsToolbox.Joint,SourceRoboticsToolbox.Joint]] = []
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Needed_joint_pos[0], gear_ratio = Joint_reduction_ratio[0], offset = 0, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Needed_joint_pos[1], gear_ratio = Joint_reduction_ratio[1], offset = 0, dir = 0))

timeout_setting = 0.001

initial = 0
initial_setup = [0,0]

# Initialize position values
position_values =  np.array([0.0,0.0])
Motor_values = np.array([0,0])

speed_values = np.array([0,0])

position_send = np.array([0,0])
speed_send = np.array([0,0])
T0 = 0
T1 = 0

received_ids = [0,0] 
Kp = 650
Kd = 0.00550 #  0.0060
Kk = 0.0003 #0.0003

timer = Timer(interval=INTERVAL_S, warnings=False, precise=True)
#while timer.elapsed_time < 1100000:
while(1):
    t1 = time.perf_counter()
    #Motor[0].Send_Respond_Encoder_data()
    #Motor[1].Send_Respond_Encoder_data()
    #Motor[0].Send_data_pack_PD(Position = 12000, Speed = 0, Current = 0)
    Motor[0].Send_data_pack_1(Position = None,Speed = None, Current = max(-2800, min(2800, int(T0))))   # the drive's limit; the library refuses beyond 16 bits
    Motor[1].Send_data_pack_1(Position = None,Speed = None, Current = max(-2800, min(2800, int(T1))))

    for i in range(1, 3):  # Loop 9-1=8 to check for received data
        message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
        #print(f"unpack{i} is: {UnpackedMessageID}")

        # Check if UnpackedMessageID is not None 
        if UnpackedMessageID is not None:
            
            # Update received id index; meaning that we received response from that CAN ID
            received_ids[UnpackedMessageID[0]] = 1
            Motor[UnpackedMessageID[0]].UnpackData(message,UnpackedMessageID)
            #print(f"Motor {UnpackedMessageID[0]}, position is: {Motor[UnpackedMessageID[0]].position}")
            unwrapped_position_raw = Joint[UnpackedMessageID[0]].unwrap_position(Motor[UnpackedMessageID[0]].position)
            Motor_values[UnpackedMessageID[0]] = Motor[UnpackedMessageID[0]].position
            position_values[UnpackedMessageID[0]] =  Joint[UnpackedMessageID[0]].get_joint_position(Motor[UnpackedMessageID[0]].position)

            if initial_setup[UnpackedMessageID[0]] == 0:
                initial_setup[UnpackedMessageID[0]] = 1
                Joint[UnpackedMessageID[0]].determine_sector(Motor[UnpackedMessageID[0]].position)


    T0 = Kp*( position_values[1] -  position_values[0]) + Kd*(Motor[1].speed - Motor[0].speed) - Kk*(Motor[0].speed)
    T1 = Kp*( position_values[0] -  position_values[1]) + Kd*(Motor[0].speed - Motor[1].speed) - Kk*(Motor[1].speed)
    
    """
    print(f"T0 value is: {int(T0)}")
    print(f"T1 value is: {int(T1)}")

    print(f"Position of GRIPPER 0 is: { position_values[0]}")
    print(f"Position of GRIPPER 1 is: { position_values[1]}")
    print(f"Speed of GRIPPER 0 is: { Motor[0].position}")
    print(f"Speed of GRIPPER 1 is: { Motor[1].position}")
    position_send[0] = Joint[0].get_encoder_position(position_values[0])
    position_send[1] = Joint[1].get_encoder_position(position_values[1])
    print(f"Bounce GRIPPER 0 is: { position_send[0]}")
    print(f"Bounce GRIPPER 1 is: { position_send[1]}")
    """

    t2 = time.perf_counter()
    #print(f"total time is {t2-t1}")
    #timer.checkpt()
