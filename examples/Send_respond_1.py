import Spectral_BLDC as Spectral
import time

Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)


while True:

    #Motor1.Send_Clear_Error()
    #Motor1.Send_Idle()
    #Motor1.Send_CAN_ID(_CAN_ID = 1)   # 0-15
    #Motor1.Send_Limits(Velocity_limit=800000,Current_limit= 1500)
    #Motor1.Send_Current_Gains(KPIQ=2,KIIQ=1)
    #Motor1.Send_Velocity_Gains(KPV=0.009,KIV=0.0002)
    #Motor1.Send_Position_Gains(KPP=10)
    #Motor1.Send_PD_Gains(KP=10,KD= 12)
    Motor1.Send_Respond_Voltage()
    #Motor1.Send_Respond_Ping()
    #Motor1.Send_Kt(Kt= 2.23)
    #Motor1.Send_Heartbeat_Setup(_rate = 12340)
    #Motor1.Send_Watchdog(_rate = 140,_action = 'Hold')
    #Motor1.Send_Respond_Iq_data()
    #Motor1.Send_Respond_State_of_Errors()
    #Motor1.Send_Respond_Encoder_data()
    #Motor1.Send_Respond_Device_Info()
    #Motor1.Send_data_pack_1(Position=-6175, Speed= 0, Current = 0)
    #Motor1.Send_Save_config()
    #Motor1.Send_gripper_data_pack(0,255,1500,1,1,0,0) 
    #Motor1.Send_gripper_data_pack(255,255,1500,1,1,0,0) 
    #Motor1.Send_gripper_data_pack()
    #Motor1.Send_gripper_calibrate()
    #Motor1.Send_data_pack_PD(Position=1000, Speed= 0,Current = 0)

    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=0.2) 

    if message is not None:
        print(f"Message is: {message}")
        print(f"Node ID is : {UnpackedMessageID.node_id}")
        print(f"Message ID is: {UnpackedMessageID.command_id}")
        print(f"Error bit is: {UnpackedMessageID.error_bit}")
        print(f"Message length is: {message.dlc}")
        print(f"Is is remote frame: {message.is_remote_frame}")
        print(f"Timestamp is: {message.timestamp}")

        Motor1.UnpackData(message,UnpackedMessageID)
    else:
        print("No message after timeout period!")
    print("")
    time.sleep(1 )
