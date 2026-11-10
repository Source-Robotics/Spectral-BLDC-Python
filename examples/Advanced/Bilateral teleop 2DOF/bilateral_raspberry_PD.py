import Spectral_BLDC as Spectral
import SourceRoboticsToolbox
import socket
import numpy as np
import time
import sched
import psutil
import os
import socket
import select

os.system("sudo ifconfig can0 txqueuelen 1000")
os.system("sudo ifconfig can0 up")
time.sleep(0.2)
# Set the CPU affinity to CPU core 2
psutil.Process(os.getpid()).cpu_affinity([2])

# Configuration for both sending and receiving
sender_ip = "192.168.0.132"  # Replace with the sender's IP address
receiver_ip = "192.168.0.179"  # Replace with the receiver's IP address
send_port = 5002
receive_port = 5001

# Create a UDP socket for sending data
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a UDP socket for receiving data
receive_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the receiving socket to the sender's IP address and receive port
receive_sock.bind((sender_ip, receive_port))

# Set the receiving socket to non-blocking mode
receive_sock.setblocking(False)

# Set the scheduling policy and priority of the process
pid = os.getpid()
sched = os.SCHED_FIFO
param = os.sched_param(98)
os.sched_setscheduler(pid, sched, param)

Master_position = [4952,4194,10514]  
Joint_reduction_ratio = [1,1,1] # Reduction ratio we have on our joints

Communication1 = Spectral.CanCommunication(bustype='socketcan', channel='can0', bitrate=1000000)

Motor = []

Motor.append(Spectral.SpectralCAN(node_id=0, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=1, communication=Communication1))
Motor.append(Spectral.SpectralCAN(node_id=2, communication=Communication1))


Joint = []

Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[0], gear_ratio = Joint_reduction_ratio[0], offset = 0, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[1], gear_ratio = Joint_reduction_ratio[1], offset = 0, dir = 0))
Joint.append(SourceRoboticsToolbox.Joint(encoder_resolution = 14, master_position=Master_position[2], gear_ratio = Joint_reduction_ratio[2], offset = 0, dir = 0))

timeout_setting = 0.00001

initial = 0
initial_setup = [0,0,0]


# Initialize position values
position_values = np.array([0.0, 0.0, 0.0])
Motor_values = np.array([0, 0, 0])
received_ids = [0,0,0]
position_values_send = np.array([0.0, 0.0, 0.0])
position_values_receive = np.array([0.0, 0.0, 0.0])

Torques = np.array([0,0,0])
speed_value_send = np.array([0, 0, 0])
speed_values_receive = np.array([0, 0, 0])

prev_time = 0

Kp = 165    # 190

Kd = 0.0020 #  0.0020

Kk = 0.001 # 0.0009

while True:

    t1 = time.perf_counter()

    # This snippet will read the last value in the buffer
    # Check for incoming data
    latest_data = None
    while True:
        try:
            ready_to_read, _, _ = select.select([receive_sock], [], [], 0)  # Timeout of 0 seconds, non-blocking read
            if receive_sock in ready_to_read:
                latest_data, addr = receive_sock.recvfrom(1024)  # Buffer size is 1024 bytes
            else:
                break
        except Exception as e:
            print(f"Error: {e}")
            break

    if latest_data:
        tt = time.perf_counter()
        time_dif = tt - prev_time
        prev_time = tt
        tt2 = time.time_ns()
        msg = latest_data.decode()
        received_parts = msg.split(',')
        if len(received_parts) > 5 and received_parts[0] == "pos":
            sent_timestamp = int(received_parts[1])
            position_data = list(map(float, received_parts[2:5]))
            #speed_data = list(map(float, received_parts[5:]))
            position_values_receive[0] = position_data[0]
            position_values_receive[1] = position_data[1]
            position_values_receive[2] = position_data[2]
            try:
                speed_values_receive[0] = int(received_parts[5])
                speed_values_receive[1] = int(received_parts[6])
                speed_values_receive[2] = int(received_parts[7])
            except:
                speed_values_receive[0] = 0
                speed_values_receive[1] = 0
                speed_values_receive[2] = 0
            latency = (tt2 - sent_timestamp) / 1e9  # Convert to seconds
            #print(f"Received message: {msg} from {addr}, RTT is: {rtt:.6f} seconds .at time {tt}, diff is: {time_dif}")
            #print(f"Received message  {msg}: latency is: {latency:.6f} seconds, diff between 2 receives is: {time_dif}")
    t_udp = time.perf_counter()

    #Motor[1].Send_Respond_Encoder_data()
    #Motor[1].Send_Respond_Encoder_data()
    #Motor[2].Send_Respond_Encoder_data()
    Motor[1].Send_data_pack_1(Position = None,Speed = None, Current = max(-2800, min(2800, int(Torques[0]))))   # the drive's limit; the library refuses beyond 16 bits
    Motor[0].Send_data_pack_1(Position = None,Speed = None, Current = max(-2800, min(2800, int(Torques[1]))))
    Motor[2].Send_data_pack_1(Position = None,Speed = None, Current = max(-2800, min(2800, int(Torques[2]))))

    for i in range(1, 4):  # Loop 9-1=8 to check for received data
        # Check the CAN buffer, if there is something get message and CAN ID
        message, UnpackedMessageID = Communication1.receive_can_messages(timeout=timeout_setting)
        #print(f"unpack{i} is: {UnpackedMessageID}")

        # Check if Unpacked Message ID is not None 
        if UnpackedMessageID is not None:
            
            # Update received id index; meaning that we received response from that CAN ID
            received_ids[UnpackedMessageID[0]] = 1
            # Unpack the message for the received ID
            Motor[UnpackedMessageID[0]].UnpackData(message,UnpackedMessageID)
            # Get the position value in radians from received motor ticks
            position_values[UnpackedMessageID[0]] =  Joint[UnpackedMessageID[0]].get_joint_position(Motor[UnpackedMessageID[0]].position)
            Motor_values[UnpackedMessageID[0]] = Motor[UnpackedMessageID[0]].position
            # Initial setup
            if initial_setup[UnpackedMessageID[0]] == 0:
                initial_setup[UnpackedMessageID[0]] = 1
                Joint[UnpackedMessageID[0]].determine_sector(Motor[UnpackedMessageID[0]].position)

    speed_values_send = np.array([Motor[1].speed, Motor[0].speed, Motor[2].speed])
    position_values_send = np.array([position_values[1], position_values[0], position_values[2]])
    position_values_str =','.join(map(str, position_values_send))
    speed_values_str = ','.join(map(str, speed_values_send))
    msg = f"pos,{time.time_ns()},{position_values_str},{speed_values_str}"  # Include "pos" at the beginning
    #print(f"msg is: {msg}")
    #print(f"received speed: {speed_values_receive}")
    msg_bytes = msg.encode()
    send_sock.sendto(msg_bytes, (receiver_ip, send_port))

    try:
        speed_dif = Kd*(speed_values_receive[2] - speed_values_send[2])
        speed_var = int(Kk*speed_values_send[2])
    except: 
        speed_dif = 0
        speed_var = 0

    try:
        speed_dif1 = Kd*(speed_values_receive[1] - speed_values_send[1])
        speed_var1 = int(Kk*speed_values_send[1])
    except: 
        speed_dif1 = 0
        speed_var1 = 0

    try:
        speed_dif0 = Kd*(speed_values_receive[0] - speed_values_send[0])
        speed_var0 = int(Kk*speed_values_send[0])
    except: 
        speed_dif0 = 0
        speed_var0 = 0



    Torques[0] = Kp*( position_values_receive[0] -  position_values_send[0])  + speed_dif0 - speed_var0#- Kk*(Motor[0].speed)
    Torques[1] = Kp*( position_values_receive[1] -  position_values_send[1])  + speed_dif1 - speed_var1#- Kk*(Motor[0].speed)
    Torques[2] = Kp*( position_values_receive[2] -  position_values_send[2])  + speed_dif - speed_var#- Kk*(Motor[0].speed)


    #print(f"radian values: {position_values}")
    #print(f"Motor values: {Motor_values}")
    #print(f"Motor values: {Motor_values[1], Motor_values[0], Motor_values[2]}")
    t2 = time.perf_counter()
    #print(f"time in function is: {t2-t1}")

    #print(f"time in udp is: {t_udp-t1}")
    #print(f"time in can is: {t2-t_udp}")

    time_total = t2-t1
    #print(time_total)
    sleep_time = 0.005 - time_total
    print(sleep_time)

    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        # If the total time exceeds 0.005 seconds, log it and continue without sleeping
        print(f"Warning: Task took longer than 0.005 seconds: {time_total:.6f} seconds")