import can
from .import CAN_utils as util
import logging
from collections import namedtuple
import struct

"""
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)
"""

CanMessage = namedtuple("CanMessage", ["node_id", "command_id", "error_bit"])


class CanCommunication:
    def __init__(self, bustype='slcan', channel='COM41', bitrate=1000000):
        self.bustype=bustype
        self.channel=channel
        self.bitrate=bitrate
        if self.channel != None:
            self.bus = can.interface.Bus(bustype = self.bustype, channel = self.channel, bitrate = self.bitrate)

    def send_can_message(self, message_id, data=b'', remote_frame=False):
        message = can.Message(arbitration_id=message_id, data=data, is_remote_frame=remote_frame, is_extended_id=False)
        try:
            self.bus.send(message)
        except can.CanOperationError as e:
            None#return False
        except Exception as e:
            None#return False

    def receive_can_messages(self,timeout = None):
        self.timeout = timeout
        message = self.bus.recv(timeout = self.timeout)
        if message is not None:
            node_id,command_id,error_bit = util.extract_from_can_id(message.arbitration_id)
            can_message = CanMessage(node_id, command_id, error_bit)
        else: 
            return None,None

        return message, can_message
        


class SpectralCAN:

    SEND_PD_GAINS_ID = 16
    SEND_CURRENT_GAINS_ID = 17
    SEND_VELOCITY_GAINS_ID = 18
    SEND_POSITION_GAINS_ID = 19
    SEND_LIMITS_ID = 20
    SEND_KT_ID = 22
    SEND_VOLTAGE_LIMIT_ID = 34
    SEND_CAN_ID = 11
    SEND_ESTOP_ID = 0
    SEND_IDLE_ID = 12
    SEND_SAVE_CONFIG_ID = 13
    SEND_RESET_ID = 14
    SEND_CLEAR_ERROR_ID = 1
    SEND_WATCHDOG_ID = 15
    SEND_HEARTBEAT_SETUP_ID = 30
    SEND_CYCLIC_COMMAND_ID = 29

    SEND_RESPOND_TEMPERATURE_ID = 23
    SEND_RESPOND_DEVICE_INFO_ID = 25
    SEND_RESPOND_ENCODER_DATA_ID = 28
    SEND_RESPOND_VOLTAGE_ID = 24
    SEND_RESPOND_STATE_OF_ERRORS_ID = 26
    SEND_RESPOND_IQ_DATA_ID = 27
    SEND_RESPOND_PING_ID = 10

    SEND_DATA_PACK_1_ID = 2
    SEND_DATA_PACK_2_ID = 6
    SEND_DATA_PACK_3_ID = 8
    SEND_DATA_PACK_PD_ID = 4
    SEND_HALL_INDEX = 31
    SEND_RESPOND_KT_ID = 33

    SEND_GRIPPER_DATA_PACK_ID = 61
    SEND_GRIPPER_CALIBRATE_ID = 62

    RESPOND_HEARTBEAT_ID = 9
    RESPOND_DATA_PACK_1_ID = 3
    RESPOND_DATA_PACK_2_ID = 5
    RESPOND_DATA_PACK_3_ID = 7
    RESPOND_GRIPPER_DATA_PACK_ID = 60
    RESPOND_DATA_HALL = 32
    


    def __init__(self, node_id, communication):
        self.node_id = node_id
        self.communication = communication
        
        self.temperature = None
        self.position = None
        self.speed = None
        self.current = None
        self.voltage = None
        self.heartbeat = None

        self.gripper_position = None
        self.gripper_speed = None
        self.gripper_current = None
        self.gripper_activated = None
        self.gripper_action_status = None
        self.gripper_object_detection = None
        self.gripper_temperature_error = None
        self.gripper_timeout_error = None
        self.gripper_estop_error = None
        self.gripper_calibrated = None

        self.error_bit = None
        self.error_bit = None
        self.error = None
        self.temperature_error = None
        self.encoder_error = None
        self.vbus_error = None
        self.driver_error = None
        self.velocity_error = None
        self.current_error = None
        self.estop_error = None
        self.calibrated = None
        self.activated = None
        self.watchdog_error = None
        self.HALL_trigger = None
        self.additional_pin2 = None
        self.hall_index = None
        self.Kt = None

        self.ping = None
        self.timestamp = None

        self.HARDWARE_VERSION = None
        self.BATCH_DATE = None
        self.SOFTWARE_VERSION = None
        self.SERIAL_NUMBER = None

        self.DLC_warrning = None


        # Configuration variables
        self._velocity_limit = 0

    def Set_2_none(self):
        self.temperature = None
        self.position = None
        self.speed = None
        self.current = None
        self.voltage = None
        self.heartbeat = None

        self.gripper_position = None
        self.gripper_speed = None
        self.gripper_current = None
        self.gripper_activated = None
        self.gripper_action_status = None
        self.gripper_object_detection = None
        self.gripper_temperature_error = None
        self.gripper_timeout_error = None
        self.gripper_estop_error = None
        self.gripper_calibrated = None

        self.error_bit = None
        self.error = None
        self.temperature_error = None
        self.encoder_error = None
        self.vbus_error = None
        self.driver_error = None
        self.velocity_error = None
        self.current_error = None
        self.estop_error = None
        self.calibrated = None
        self.activated = None
        self.HALL_trigger = None
        self.additional_pin2 = None
        self.hall_index = None
        self.Kt = None
        self.ping = None
        self.timestamp = None

        self.HARDWARE_VERSION = None
        self.BATCH_DATE = None
        self.SOFTWARE_VERSION = None
        self.SERIAL_NUMBER = None

        self.DLC_warrning = None
    

    @property
    def velocity_limit(self):
        return self._velocity_limit

    @velocity_limit.setter
    def velocity_limit(self, value):
        # Ensure the value is within acceptable range or add additional validation logic
        self._velocity_limit = value


    def UnpackData(self,message,UnpackedMessageID):

        #logging.debug(message.data)
        #logging.debug(UnpackedMessageID.node_id)
        self.Set_2_none()

        self.timestamp = message.timestamp
        match UnpackedMessageID.command_id:

            case self.RESPOND_HEARTBEAT_ID:
                self.heartbeat = 1
                #logging.debug("Got heartbeat")


            case self.RESPOND_DATA_PACK_1_ID:
                if message.dlc == 8:
                    self.position = util.fuse_3_bytes(b'\x00' + message.data[0:3])
                    self.speed = util.fuse_3_bytes(b'\x00' + message.data[3:6])
                    self.current = util.fuse_2_bytes(b'\x00' + b'\x00' + message.data[6:8])
                else:
                    self.DLC_warrning = 1
                #logging.debug(f"Encoder position ticks: {self.position}")
                #logging.debug(f"Encoder speed: {self.speed}")
                #logging.debug(f"current is: {self.current}")
     


            case self.RESPOND_DATA_PACK_2_ID:
                None

            case self.RESPOND_DATA_PACK_3_ID:
                None

            case self.RESPOND_DATA_HALL:
                if message.dlc == 4:
                    self.position = util.fuse_3_bytes(b'\x00' + message.data[0:3])
                    byte1 = util.split_2_bitfield(message.data[3])
                    self.HALL_trigger = byte1[0]
                    self.additional_pin2 = byte1[1]
                    self.hall_index = byte1[2]   # new: edge detected flag
                else:
                    self.DLC_warrning = 1
            

            case self.RESPOND_GRIPPER_DATA_PACK_ID:
                if message.dlc == 4:
                    self.gripper_position = message.data[0]
                    #self.gripper_speed = message.data[1]
                    self.gripper_current = util.fuse_2_bytes(b'\x00' + b'\x00' + message.data[1:3])
                    byte1 = util.split_2_bitfield(message.data[3])
                    self.gripper_activated = byte1[0]
                    self.gripper_action_status = byte1[1]
                    self.gripper_object_detection = util.combine_bits(byte1[2],byte1[3])
                    self.gripper_temperature_error = byte1[4]
                    self.gripper_timeout_error = byte1[5]
                    self.gripper_estop_error = byte1[6]
                    self.gripper_calibrated = byte1[7]
                else:
                    self.DLC_warrning = 1

                #logging.debug("Got gripper data")
                #logging.debug(f"Position is: {self.gripper_position}")
                #logging.debug(f"Current is: {self.gripper_current}")
                #logging.debug(f"Activated is: {self.gripper_activated}")
                #logging.debug(f"Action status is: {self.gripper_action_status}")
                #logging.debug(f"Object detection is: {self.gripper_object_detection}")
                #logging.debug(f"Temperature error is: {self.gripper_temperature_error}")
                #logging.debug(f"Timeout error is: {self.gripper_timeout_error}")
                #logging.debug(f"Estop error is: {self.gripper_estop_error}")
                #logging.debug(f"Calibrated is: {self.gripper_calibrated}")

                
                

            # Radi
            case self.SEND_RESPOND_TEMPERATURE_ID:
                if message.dlc == 2:
                    self.temperature = util.fuse_2_bytes(b'\x00'+ b'\x00' + message.data[0:2])
                else:
                    self.DLC_warrning = 1
                #logging.debug(f"Temperature is: {self.temperature}")


            # Radi
            case self.SEND_RESPOND_DEVICE_INFO_ID:
                if message.dlc == 7:
                    self.HARDWARE_VERSION = message.data[0]
                    self.BATCH_DATE = message.data[1]
                    self.SOFTWARE_VERSION = message.data[2]
                    self.SERIAL_NUMBER = util.fuse_4_bytes(message.data[3:7])
                else:
                    self.DLC_warrning = 1

                #logging.debug(f"Hardware version is: {self.HARDWARE_VERSION}")
                #logging.debug(f"Batch date is: {self.BATCH_DATE}")
                #logging.debug(f"Software version is: {self.SOFTWARE_VERSION}")
                #logging.debug(f"Serial number is: {self.SERIAL_NUMBER}")


            #Radi
            case self.SEND_RESPOND_ENCODER_DATA_ID:
                if message.dlc == 8:
                    self.position = util.fuse_4_bytes(message.data[0:4])
                    self.speed = util.fuse_4_bytes(message.data[4:8])
                else:
                    self.DLC_warrning = 1
                #logging.debug(f"Encoder position ticks: {self.position}")
                #logging.debug(f"Encoder speed: {self.speed}")

            # Radi
            case self.SEND_RESPOND_VOLTAGE_ID:
                if message.dlc == 2:
                    self.voltage = util.fuse_2_bytes(b'\x00'+ b'\x00' + message.data[0:2])
                else:
                    self.DLC_warrning = 1
                #logging.debug(f"Voltage is: {self.voltage}")


            case self.SEND_RESPOND_KT_ID:
                if message.dlc == 4:
                    self.Kt = struct.unpack('>f', bytes(message.data[0:4]))[0]
                else:
                    self.DLC_warrning = 1

            # Radi
            case self.SEND_RESPOND_STATE_OF_ERRORS_ID:
                if message.dlc == 2:
                    byte1 = util.split_2_bitfield(message.data[0])
                    byte2 = util.split_2_bitfield(message.data[1])
                    self.error = byte1[0]
                    self.temperature_error = byte1[1]
                    self.encoder_error = byte1[2]
                    self.vbus_error = byte1[3]
                    self.driver_error = byte1[4]
                    self.velocity_error = byte1[5]
                    self.current_error = byte1[6] 
                    self.estop_error = byte1[7]
                    self.calibrated = byte2[0]
                    self.activated = byte2[1]
                    self.watchdog_error = byte2[2]


                else:
                    self.DLC_warrning = 1

                #logging.debug(f"Error is: {self.error}")
                #logging.debug(f"Temperature rror is: {self.temperature_error}")
                #logging.debug(f"Encoder error is: {self.encoder_error}")
                #logging.debug(f"Vbus error is: {self.vbus_error}")
                #logging.debug(f"Driver error is: {self.driver_error}")
                #logging.debug(f"Velocity error is: {self.velocity_error}")
                #logging.debug(f"Current error is: {self.current_error}")
                #logging.debug(f"Estop error is: {self.estop_error}")
                #logging.debug(f"Watchdog error is: {self.watchdog_error}")
                #logging.debug(f"Calibrated is: {self.calibrated}")
                #logging.debug(f"Activated is: {self.activated}")


            # Radi
            case self.SEND_RESPOND_IQ_DATA_ID:
                if message.dlc == 2:
                    self.current = util.fuse_2_bytes(b'\x00'+ b'\x00' + message.data[0:2])
                else:
                    self.DLC_warrning = 1
                #logging.debug(f"Current is: {self.current}")

            # Radi
            case self.SEND_RESPOND_PING_ID:
                self.ping = 1
                #logging.debug(f"Ping is: {self.ping}")
    

    def Send_data_pack_1(self,Position = None,Speed = None, Current = None):
        
        if(Position != None and Speed != None and Current != None):
            _canid = util.combine_2_can_id(self.node_id,self.SEND_DATA_PACK_1_ID,0)
            _position = util.split_2_3_bytes(Position)
            _speed = util.split_2_3_bytes(Speed)
            _current = util.split_2_2_bytes(Current)
            combined = _position +  _speed + _current
            _combined = bytearray(combined)
            self.communication.send_can_message(message_id = _canid, data = _combined)
        elif(Position == None and Speed != None and Current != None):
            _canid = util.combine_2_can_id(self.node_id,self.SEND_DATA_PACK_1_ID,0)
            _speed = util.split_2_3_bytes(Speed)
            _current = util.split_2_2_bytes(Current)
            combined =  _speed + _current
            _combined = bytearray(combined)
            self.communication.send_can_message(message_id = _canid, data = _combined)
        elif(Position == None and Speed == None and Current != None):
            _canid = util.combine_2_can_id(self.node_id,self.SEND_DATA_PACK_1_ID,0)
            _current = util.split_2_2_bytes(Current)
            combined =  _current
            _combined = bytearray(combined)
            self.communication.send_can_message(message_id = _canid, data = _combined)
        else:
            #None
            logging.debug("Invalid command")


        #logging.debug("Send data pack 1")
        



    def Send_data_pack_2():
        None

    def Send_data_pack_3():
        None

    def Send_data_pack_PD(self,Position = 0,Speed = 0, Current = 0):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_DATA_PACK_PD_ID,0)
        _position = util.split_2_3_bytes(Position)
        _speed = util.split_2_3_bytes(Speed)
        _current = util.split_2_2_bytes(Current)
        combined = _position +  _speed + _current
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
  
        #logging.debug("Send data pack PD")

    def Send_data_pack_HALL(self,Speed = 0, trigger_value = 0):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_HALL_INDEX,0)
        _speed = util.split_2_3_bytes(Speed)
        # trigger_value sent as raw byte: 0=low level, 1=high level, 2=any edge
        combined = _speed + bytearray([trigger_value])
        self.communication.send_can_message(message_id=_canid, data=combined)
  
        #logging.debug("Send data pack HALL")

    def Send_gripper_data_pack(self,position_ = None,speed_ = None,current_ = None,activate_ = None,action_ = None,ESTOP_ = None,Release_dir_ = None):

        if(position_ != None and  speed_!= None and current_ != None and activate_ != None and action_ != None and ESTOP_ != None and Release_dir_ != None):
            _canid = util.combine_2_can_id(self.node_id,self.SEND_GRIPPER_DATA_PACK_ID,0)
            _current_ = util.split_2_2_bytes(current_)
            bitfield_list = [activate_,action_,ESTOP_,Release_dir_,0,0,0,0] 
            fused = util.fuse_bitfield_2_bytearray(bitfield_list)
            _combined = bytearray(bytes([position_]) + bytes([speed_]) + _current_ + fused)
            self.communication.send_can_message(message_id = _canid, data = _combined)
            #logging.debug("Send gripper data pack 1")
        else:
            _canid = util.combine_2_can_id(self.node_id,self.SEND_GRIPPER_DATA_PACK_ID,0)
            self.communication.send_can_message(message_id = _canid)


    def Send_gripper_calibrate(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_GRIPPER_CALIBRATE_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send gripper calibrate")
        

    # Radi
    def Send_PD_Gains(self,KP,KD):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_PD_GAINS_ID,0)
        _KP = util.float_to_bytes(KP)
        _KD = util.float_to_bytes(KD)
        combined = _KP +  _KD
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send PD gains")

    # Radi
    def Send_Current_Gains(self,KPIQ,KIIQ):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_CURRENT_GAINS_ID,0)
        _KPIQ = util.float_to_bytes(KPIQ)
        _KIIQ = util.float_to_bytes(KIIQ)
        combined = _KPIQ +  _KIIQ
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send current gains")

    # Radi
    def Send_Velocity_Gains(self,KPV,KIV):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_VELOCITY_GAINS_ID,0)
        _KPV = util.float_to_bytes(KPV)
        _KIV = util.float_to_bytes(KIV)
        combined = _KPV +  _KIV
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send velocity gains")

    # Radi
    def Send_Position_Gains(self,KPP):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_POSITION_GAINS_ID,0)
        _KPP = util.float_to_bytes(KPP)
        combined = _KPP
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send new position gains")

    # Radi
    def Send_Limits(self,Velocity_limit,Current_limit):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_LIMITS_ID,0)
        _v_limit = util.float_to_bytes(Velocity_limit)
        _c_limit = util.float_to_bytes(Current_limit)
        combined = _v_limit +  _c_limit
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send new velocity and current limits")

    # Radi
    def Send_Voltage_Limit(self,Voltage_limit):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_VOLTAGE_LIMIT_ID,0)
        _v_limit = util.split_2_4_bytes(Voltage_limit)  # mV, 4-byte big-endian int (0 = use VBUS_mV)
        combined = _v_limit
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send new voltage limit")

    # Radi
    def Send_Kt(self,Kt):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_KT_ID,0)
        _Kt = util.float_to_bytes(Kt)
        combined = _Kt
        _combined = bytearray(combined)
        self.communication.send_can_message(message_id = _canid, data = _combined)
        #logging.debug("Send new Kt")

    # Radi čudno
    def Send_CAN_ID(self,_CAN_ID):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_CAN_ID,0) # Node id, command id, error bit ( 0 )
        _data = _CAN_ID
        self.communication.send_can_message(message_id = _canid, data = [_data])
        #logging.debug("Send new CAN ID")
        
    # Radi
    def Send_ESTOP(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_ESTOP_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send ESTOP")
    
    #Radi
    def Send_Idle(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_IDLE_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send Idle")

    # Radi
    def Send_Save_config(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_SAVE_CONFIG_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send Save config")

    # Radi
    def Send_Reset(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESET_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send Reset")

    # Radi
    def Send_Clear_Error(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_CLEAR_ERROR_ID,0)
        self.communication.send_can_message(message_id = _canid)
        #logging.debug("Send Clear Error")

    # Radi
    def Send_Watchdog(self,_rate,_action = 'Idle'):
        if _action == 'Idle':
            _action_byte = bytes([0x00])
        elif _action == 'Hold':
            _action_byte = bytes([0x01])
        elif _action == 'Brake':
            _action_byte = bytes([0x02])
        
        _canid = util.combine_2_can_id(self.node_id,self.SEND_WATCHDOG_ID,0)
        watch_time = util.split_2_4_bytes(_rate)
        _combined = bytearray(watch_time + _action_byte)
        self.communication.send_can_message(message_id = _canid, data = _combined)

    # Radi
    def Send_Heartbeat_Setup(self,_rate):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_HEARTBEAT_SETUP_ID,0)
        Heart_rate = util.split_2_4_bytes(_rate)
        _combined = bytearray(Heart_rate)
        self.communication.send_can_message(message_id = _canid, data = _combined)

    # TODO
    def Send_Cyclic_command(self,ID):
        None

    # The DLC value does not necessarily define the number of bytes of data in a message.
    # Its purpose varies depending on the frame type - for data frames it represents the amount of data contained in the message, in remote frames it represents the amount of data being requested.
    
    def Send_Respond_Temperature(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_TEMPERATURE_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame=True)
        #logging.debug("Request temperature")

    def Send_Respond_Voltage(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_VOLTAGE_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)
        #logging.debug("Request voltage")

    def Send_Respond_Device_Info(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_DEVICE_INFO_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)
        #logging.debug("Request device info")

    def Send_Respond_State_of_Errors(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_STATE_OF_ERRORS_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)
        #logging.debug("Request state of all errors")

    def Send_Respond_Iq_data(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_IQ_DATA_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)   
        #logging.debug("Request Iq data")     

    def Send_Respond_Encoder_data(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_ENCODER_DATA_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)
        #logging.debug("Request encoder data")

    def Send_Respond_Ping(self):
        _canid = util.combine_2_can_id(self.node_id,self.SEND_RESPOND_PING_ID,0)
        self.communication.send_can_message(message_id = _canid, remote_frame = True)
        #logging.debug("Request ping")

    def Send_Respond_Kt(self):
        _canid = util.combine_2_can_id(self.node_id, self.SEND_RESPOND_KT_ID, 0)
        self.communication.send_can_message(message_id=_canid, remote_frame=True)
        #logging.debug("Request Kt")



if __name__ == "__main__":
    None