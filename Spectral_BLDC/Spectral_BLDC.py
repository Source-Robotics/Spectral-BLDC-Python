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
    SEND_SETTINGS_ID = 35            # IN_SETTINGS: 2 bytes [mask, value]; firmware V107+
    SETTINGS_BIT_BRAKE_COAST = 0x01  # bit 0: 1 = brake, 0 = coast (idle-mode phase behaviour)
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
    # Firmware 109 and later (both drivers share their version numbers from 109 on; the Spectral Micro has no 108)
    SEND_PARAMETER_ID = 36           # set a parameter: [number][value i32], no answer
    SEND_GET_PARAMETER_ID = 37       # ask for a parameter: [number]; the drive answers on 38
    RESPOND_PARAMETER_ID = 38        # [number][value i32], or [number] alone if the number is unknown
    SEND_RESPOND_IDENTITY_ID = 39    # remote frame: product, tool, versions, pole pairs, calibrated, flags

    # Parameter numbers (firmware constants.h PARAM_*); permanent once released
    PARAM_TOOL_ID = 1                # 0 none, 1 SSG-48, 11-33 MSG, 200-254 custom
    PARAM_WD_FALLBACK_TIME = 2       # ms watchdog hold/descend may run, 0-60000
    PARAM_WD_FALLBACK_CURRENT = 3    # mA descend applies, signed
    PARAM_WD_FALLBACK_SPEED = 4      # ticks/s, the speed cap of descend
    PARAM_WD_AFTER_ACTION = 5        # after hold/descend: 0 legacy, 2 brake, 3 coast
    PARAM_WD_TIME = 6                # watchdog time ms, 0 = off
    PARAM_WD_ACTION = 7              # watchdog action 0-4
    # The gains and limits in force (firmware 109): readable, and writable here too since 572a4ad / acaed1a
    PARAM_KPP = 8
    PARAM_KPV = 9
    PARAM_KIV = 10
    PARAM_KPIQ = 11
    PARAM_KIIQ = 12
    PARAM_KP = 13
    PARAM_KD = 14
    PARAM_VELOCITY_LIMIT = 15
    PARAM_CURRENT_LIMIT = 16
    PARAM_VOLTAGE_LIMIT = 17
    PARAM_BRAKE_COAST = 18
    PARAM_HEARTBEAT_MS = 19
    PARAM_KT = 20
    PARAM_SENSE1_OFFSET = 21         # current-sense zero 1, counts from 2048 (1 count = 1.6 mA); writable (RAM) since batch 10
    PARAM_SENSE2_OFFSET = 22
    PARAM_ID_SETPOINT = 58           # the d-axis current setpoint, mA (RAM, as #Id): no torque, more phase current
    PARAM_BOARD_PRODUCT = 23
    # Capture, histogram, anti-cogging (firmware 2026-11 batch 3). Capture/histogram: RAM only.
    PARAM_CAPTURE_SOURCE_A = 24      # signal of channel A (SIGNALS), also the trigger channel
    PARAM_CAPTURE_SOURCE_B = 25
    PARAM_CAPTURE_SOURCE_C = 26
    PARAM_CAPTURE_DIVIDER = 27       # control loops per sample, 1-65535
    PARAM_CAPTURE_TRIGGER = 28       # writing starts (TRIGGERS) or stops (0) a capture
    PARAM_CAPTURE_LEVEL = 29         # trigger level in channel A's units
    PARAM_CAPTURE_PRETRIGGER = 30    # samples kept from before the trigger, 0 to the capture length - 1 (1023)
    PARAM_CAPTURE_STATE = 31         # read-only, CAPTURE_STATES
    PARAM_CAPTURE_COUNT = 32         # read-only, samples readable (0 to the capture length, 1024)
    PARAM_CAPTURE_TRIGGER_INDEX = 33 # read-only, the trigger sample's index, -1 if not triggered
    PARAM_HIST_X = 34                # HISTOGRAM_X: writing 1/2 clears and starts, 0 stops
    PARAM_HIST_Y = 35                # the signal averaged per bin; set before PARAM_HIST_X
    PARAM_HIST_SAMPLES = 36          # read-only
    PARAM_COGGING_MODE = 37          # COGGING_MODES; saved with the harmonics and table
    PARAM_COGGING_SCALE = 38         # percent of the stored anti-cogging applied, 0-200
    PARAM_CAPTURE_LENGTH = 39        # read-only, samples per channel a capture holds (1024; older builds 512, no answer)
    PARAM_SENSE1_OFFSET_OFF = 40     # read-only, current-sense zero 1 measured with the bridge off (counts)
    PARAM_SENSE2_OFFSET_OFF = 41     # read-only, current-sense zero 2 measured with the bridge off
    PARAM_SENSE_ZERO_SWITCHING = 42  # read-only, the zero measured with the bridge switching: SENSE_ZERO_SWITCHING
    SENSE_ZERO_SWITCHING = {0: 'not run', 1: 'measuring', 2: 'in use', 3: 'rejected', 4: 'interrupted',
                            5: 'measured, not in use'}
    PARAM_SENSE_ZERO_USE_SWITCHING = 46  # 1: use the zero measured with the bridge switching, 0: the bridge-off one
    PARAM_SENSE1_OFFSET_SW = 47      # read-only, current-sense zero 1 measured with the bridge switching (counts)
    PARAM_SENSE2_OFFSET_SW = 48      # read-only, current-sense zero 2 measured with the bridge switching
    PARAM_CURRENT_BANDWIDTH = 49     # current-loop bandwidth, Hz (10 to loop rate / 10): writing sets Kp_iq and
                                     # Ki_iq at once from the measured R and L; reading gives the bandwidth the
                                     # gains in force amount to (default 160 since firmware 2026-11 batch 7)
    PARAM_SPEED_ESTIMATOR = 50       # the speed the loops use: SPEED_ESTIMATORS (firmware 2026-11 batch 8, RAM);
                                     # default STEPFOC 2 (PLL direct), Spectral Micro 0 (moving average)
    SPEED_ESTIMATORS = {0: 'moving average', 1: 'PLL', 2: 'PLL direct'}
    PARAM_SPEED_PLL_BANDWIDTH = 51   # the speed PLLs' natural frequency, Hz (10 to loop rate / 20; 160 = 1000 rad/s);
                                     # default STEPFOC 60, Spectral Micro 160
    PARAM_COGCAL = 43                # write 1: identify the anti-cogging on the drive, 2: and save, 0: abort
    PARAM_COGCAL_SPEED = 44          # ticks/s the identification turns at (1500)
    PARAM_COGCAL_RIPPLE = 45         # read-only, the identified cogging current, rms, 0.1 mA
    COGCAL_STATES = {0: 'idle', 1: 'turning forward', 2: 'turning backward', 3: 'done', 4: 'aborted',
                     5: 'failed: error or not calibrated', 6: 'failed: the shaft did not turn',
                     7: 'measuring the encoder table first', 8: 'failed: the encoder table',
                     9: 'failed: the current-sense trim did not help (zeros put back)',
                     10: 'off: the encoder table was measured again on its own; identify again',
                     11: 'done, without the current-sense trim (it did not lower the 1st harmonic)'}
    # The current-sense zero trim (batch 11): PARAM_COGCAL = 4 identifies, corrects the zeros from the 1st harmonic
    # and identifies again; 5 also saves.
    PARAM_ZERO_TRIM1 = 59            # the trim of current-sense zero 1, counts (saved; added at every start)
    PARAM_ZERO_TRIM2 = 60
    PARAM_THETA_OFFSET = 61          # read-only, the electrical offset, 0.1 deg electrical
    PARAM_TRIM_H1_BEFORE = 62        # read-only, the 1st harmonic before the last trim, 0.1 mA
    PARAM_TRIM_H1_AFTER = 63         # read-only, and after it
    PARAM_COMM_DELAY_US = 64         # commutation delay compensation, us (0 = off, RAM; STEPFOC default 166 since batch 17)
    PARAM_FAST_TRIG = 65             # 1: the transforms' sin/cos from the integer table, 0: qfplib (default)
    PARAM_PROFILE = {66: 'checks', 67: 'reading', 68: 'transforms', 69: 'control and PWM', 70: 'rest'}  # R, 0.1 us; STEPFOC
    PARAM_FAST_READ = 71             # 1: the ADC reads and the encoder SPI by register (STEPFOC; default 1 since batch 16)
    PARAM_FAST_PWM = 72              # 1: the PWM compare registers written directly (STEPFOC; default 1 since batch 16)
    # The velocity-loop details (firmware batch 18), RAM, 0 = off (the default):
    # 73 (an anti-windup decay, batch 18) was removed in batch 19
    PARAM_INPUT_FILTER_HZ = 74       # the position input filter, true Hz, position and PD modes; about the host's
                                     # setpoint rate / 12 (8 for 100 Hz); for hosts that send no speed
    PARAM_VEL_INT_DEADBAND = 75      # ticks: the velocity integral sees the position error less this band
    # The motor (firmware batch 20), as #R, #L, #PP; saved by Save config. The current gains follow at the next
    # calibration (or a write of PARAM_CURRENT_BANDWIDTH).
    PARAM_R_MOHM = 76                # phase resistance, milliohm
    PARAM_L_UH = 77                  # phase inductance, microhenry
    PARAM_POLE_PAIRS = 78            # pole pairs, only while idle; calibrate after a change
    PARAM_FEATURES = 79              # read-only bits: what is in force and what runs (FEATURES; batch 20)
    FEATURES = {0: 'calibrated', 1: 'encoder table measured', 2: 'encoder table in use',
                3: 'anti-cogging identified', 4: 'anti-cogging on', 5: 'anti-cogging as the table',
                6: 'anti-cogging saved', 7: 'current-sense zero trim', 8: 'switching zero in use',
                9: 'delay compensation', 10: 'speed PLL', 11: 'fast reading', 12: 'fast PWM',
                13: 'fast sin/cos', 14: 'input filter', 15: 'integral deadband',
                20: 'calibration running', 21: 'encoder table running', 22: 'anti-cogging running',
                23: 'runs after the calibration pending', 24: 'sound playing'}  # bits 16-17: parameter 56
    # Sound through the motor (firmware batch 22; Spectral_BLDC.music makes the events and samples). It plays while
    # the drive holds or moves (position, velocity, current or PD mode); idle or an error stops it.
    SEND_SOUND_ID = 44               # [op][channel][a u16][b u16][c u16]: see the Send_Sound_* functions
    SEND_SOUND_SAMPLES_ID = 45       # 1-8 samples (int8) per frame into the drive's 1024-sample buffer
    PARAM_SOUND_STATE = 80           # read-only: bits 0-7 queued events, 8-18 buffered samples, 24 tones,
                                     # 25 queue playing, 26 samples playing, 27 the samples ran dry
    PARAM_SOUND_LIMIT_MV = 81        # the sound's voltage at most, mV (default 2000; 0 refuses sound)
    PARAM_SOUND_AXIS = 82            # 1: the q axis (a small torque ripple; default, much louder), 0: the d axis
    PARAM_STACK_UNUSED = 83          # read-only: RAM the stack never reached since start, bytes (firmware 109)
    SOUND_QUEUE_FREE = 31            # events the drive's queue holds
    SOUND_BUFFER = 1023              # samples the drive's buffer holds
    # The encoder table on the drive (firmware 2026-11 batch 9): one shaft turn each way open loop, a 128-entry
    # correction of the encoder, saved in its own EEPROM block. PARAM_COGCAL or PARAM_ENCAL = 3 runs it, then the
    # anti-cogging, and saves both.
    PARAM_ENCAL = 52                 # write 1: measure and use, 2: and save, 3: then the anti-cogging, save all; 0 abort
    PARAM_ENCAL_USE = 53             # 1: correct the encoder with the table (only if one was measured)
    PARAM_ENCAL_PP = 54              # read-only, the table's error peak to peak, 0.01 deg of the shaft
    PARAM_ENCAL_ONCE = 55            # read-only, its once-per-turn part, 0.01 deg of the shaft
    PARAM_AFTER_CAL = 56             # what runs after a successful calibration: AFTER_CAL (saved with the table);
                                     # STEPFOC default 'both' since batch 20 (then saves everything), Spectral 'nothing'
    PARAM_ENCAL_OFFSET = 57          # read-only, how far the last run moved the electrical offset, 0.1 deg electrical
    ENCAL_STATES = {0: 'idle', 1: 'turning forward', 2: 'turning backward', 3: 'done', 4: 'aborted',
                    5: 'failed: error or not calibrated', 6: 'failed: the rotor did not follow',
                    7: 'rejected: implausible'}
    AFTER_CAL = {0: 'nothing', 1: 'anti-cogging', 2: 'encoder table and anti-cogging'}
    # Parameters whose 4 value bytes are an IEEE-754 float (the bit pattern), not an int32
    FLOAT_PARAMETERS = {8, 9, 10, 11, 12, 13, 14, 15, 16, 20}
    PARAMETER_NAMES = {1: 'tool id', 2: 'watchdog fallback time ms', 3: 'watchdog fallback current mA',
                       4: 'watchdog fallback speed ticks/s', 5: 'watchdog after-action', 6: 'watchdog time ms',
                       7: 'watchdog action', 8: 'position gain Kpp', 9: 'velocity gain Kpv', 10: 'velocity gain Kiv',
                       11: 'current gain Kpiq', 12: 'current gain Kiiq', 13: 'PD stiffness KP', 14: 'PD damping KD',
                       15: 'velocity limit ticks/s', 16: 'current limit mA', 17: 'voltage limit mV',
                       18: 'brake on idle', 19: 'heartbeat ms', 20: 'Kt', 21: 'current-sense zero 1 (counts)',
                       22: 'current-sense zero 2 (counts)', 23: 'board product stamp',
                       24: 'capture source A', 25: 'capture source B', 26: 'capture source C',
                       27: 'capture divider (loops)', 28: 'capture trigger', 29: 'capture level',
                       30: 'capture pre-trigger', 31: 'capture state', 32: 'capture count',
                       33: 'capture trigger index', 34: 'histogram x', 35: 'histogram y', 36: 'histogram samples',
                       37: 'anti-cogging mode', 38: 'anti-cogging scale %', 39: 'capture length',
                       40: 'current-sense zero 1, bridge off (counts)', 41: 'current-sense zero 2, bridge off (counts)',
                       42: 'current-sense zero with the bridge switching', 43: 'anti-cogging identification',
                       44: 'identification speed ticks/s', 45: 'identified cogging, 0.1 mA rms',
                       46: 'use the switching zero', 47: 'current-sense zero 1, switching (counts)',
                       48: 'current-sense zero 2, switching (counts)', 49: 'current-loop bandwidth Hz',
                       50: 'speed estimate (0 average, 1 PLL, 2 direct PLL)', 51: 'speed PLL bandwidth Hz',
                       52: 'encoder table', 53: 'use the encoder table', 54: 'encoder error p-p, 0.01 deg',
                       55: 'encoder error once per turn, 0.01 deg', 56: 'after calibration',
                       57: 'encoder offset move, 0.1 deg electrical', 58: 'Id setpoint mA',
                       59: 'zero trim 1 (counts)', 60: 'zero trim 2 (counts)', 61: 'theta offset, 0.1 deg el',
                       62: '1st harmonic before the trim, 0.1 mA', 63: '1st harmonic after the trim, 0.1 mA',
                       64: 'commutation delay us', 65: 'fast sin/cos', 71: 'fast reading', 72: 'fast PWM',
                       74: 'position input filter Hz',
                       75: 'velocity integral deadband ticks', 76: 'R milliohm', 77: 'L microhenry',
                       78: 'pole pairs', 79: 'features (bits)', 80: 'sound state (bits)',
                       81: 'sound limit mV', 82: 'sound axis (0 d, 1 q)', 83: 'stack never used, bytes'}
    SEND_RESPOND_TELEMETRY_ID = 40   # remote frame: temperature, Vbus, both error bytes, Iq in one snapshot
    # Buffers (firmware 2026-11 batch 3): the capture, the histogram and the anti-cogging data
    SEND_BUFFER_READ_ID = 41         # [kind][first u16][count u16]; the drive answers with one frame per entry on 42
    RESPOND_BUFFER_DATA_ID = 42      # [kind << 13 | index (u16)][6 bytes, by kind]
    SEND_BUFFER_WRITE_ID = 43        # the 42 layout; anti-cogging harmonics and table only
    BUFFER_CAPTURE = 0               # 3 channels (i16) per sample, oldest first, up to the capture length
    BUFFER_HISTOGRAM = 1             # count (u16), sum (i32) per bin; 256 bins
    BUFFER_COGGING_HARMONICS = 2     # harmonic, a (mA), b (mA) per slot; 8 slots
    BUFFER_COGGING_TABLE = 3         # 256 values (mA) per electrical turn, 3 per frame
    BUFFER_ENCODER_TABLE = 4         # 128 values (1/16 count) per shaft turn, 3 per frame; read only (batch 9)
    BUFFER_LENGTHS = {0: 1024, 1: 256, 2: 8, 3: 256, 4: 128}   # capture: the most a capture holds (parameter 39)
    SIGNALS = {'none': 0, 'iq': 1, 'id': 2, 'iq_setpoint': 3, 'velocity': 4, 'velocity_16': 5, 'position': 6,
               'position_error': 7, 'electrical_angle': 8, 'encoder': 9, 'vbus_10mV': 10, 'sense1': 11,
               'sense2': 12, 'loop_time_us': 13, 'cogging_ff': 14, 'iq_setpoint_loop': 15, 'uq_mV': 16,
               'ud_applied': 17, 'uq_applied': 18}
    TRIGGERS = {'stop': 0, 'now': 1, 'error': 2, 'rising': 3, 'falling': 4}
    CAPTURE_STATES = {0: 'idle', 1: 'armed', 2: 'recording', 3: 'done'}
    HISTOGRAM_X = {'off': 0, 'electrical': 1, 'encoder': 2}
    COGGING_MODES = {'off': 0, 'harmonics': 1, 'table': 2}

    PRODUCT_NAMES = {1: 'STEPFOC', 2: 'Spectral Micro'}
    WATCHDOG_ACTIONS = {'Idle': 0, 'Hold': 1, 'Brake': 2, 'Coast': 3, 'Descend': 4}

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


        # Identity and settings block 2 (firmware 2026-11 and later)
        self.product_id = None
        self.product_name = None
        self.tool_id = None
        self.firmware_version = None
        self.identity_hardware_version = None
        self.pole_pairs = None
        self.identity_calibrated = None
        self.identity_flags = None
        self.watchdog_fallback = None   # (time_ms, current_mA, speed_ticks_s, after_action)
        self.parameters = {}            # parameter number -> value, from RESPOND_PARAMETER_ID
        self.unknown_parameter = None   # the last number the drive did not know
        self.wrong_product = None       # error byte 2 bit 3: this firmware is on the other product's board
        self.capture = {}               # sample index -> (A, B, C), from RESPOND_BUFFER_DATA_ID
        self.histogram = {}             # bin -> (count, sum)
        self.cogging_harmonics = {}     # slot -> (harmonic, a mA, b mA)
        self.cogging_table = {}         # entry -> mA
        self.encoder_table = {}         # entry -> 1/16 count

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
                    # The firmware packs the value's low bit first (byte1[2]) and its high bit second
                    # (byte1[3]); 1.20 combined them the other way round, which swapped 1 and 2.
                    self.gripper_object_detection = util.combine_bits(byte1[3],byte1[2])
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


            case self.RESPOND_PARAMETER_ID:
                if message.dlc == 5:
                    number = message.data[0]
                    if number in self.FLOAT_PARAMETERS:
                        value = struct.unpack('>f', bytes(message.data[1:5]))[0]
                    else:
                        value = int.from_bytes(bytes(message.data[1:5]), 'big', signed=True)
                    self.parameters[number] = value
                    if number == self.PARAM_TOOL_ID:
                        self.tool_id = value
                    self.watchdog_fallback = tuple(self.parameters.get(n) for n in (
                        self.PARAM_WD_FALLBACK_TIME, self.PARAM_WD_FALLBACK_CURRENT,
                        self.PARAM_WD_FALLBACK_SPEED, self.PARAM_WD_AFTER_ACTION))
                elif message.dlc == 1:
                    self.unknown_parameter = message.data[0]
                else:
                    self.DLC_warrning = 1

            case self.RESPOND_BUFFER_DATA_ID:
                if message.dlc == 8:
                    d = bytes(message.data)
                    header = int.from_bytes(d[0:2], 'big')
                    kind, index = header >> 13, header & 0x1FFF
                    v = [int.from_bytes(d[k:k + 2], 'big', signed=True) for k in (2, 4, 6)]
                    if kind == self.BUFFER_CAPTURE:
                        self.capture[index] = tuple(v)
                    elif kind == self.BUFFER_HISTOGRAM:
                        self.histogram[index] = (int.from_bytes(d[2:4], 'big'), int.from_bytes(d[4:8], 'big', signed=True))
                    elif kind == self.BUFFER_COGGING_HARMONICS:
                        self.cogging_harmonics[index] = (int.from_bytes(d[2:4], 'big'), v[1], v[2])
                    elif kind == self.BUFFER_COGGING_TABLE:
                        for k in range(3):
                            if index + k < self.BUFFER_LENGTHS[self.BUFFER_COGGING_TABLE]:
                                self.cogging_table[index + k] = v[k]
                    elif kind == self.BUFFER_ENCODER_TABLE:
                        for k in range(3):
                            if index + k < self.BUFFER_LENGTHS[self.BUFFER_ENCODER_TABLE]:
                                self.encoder_table[index + k] = v[k]
                else:
                    self.DLC_warrning = 1

            case self.SEND_RESPOND_TELEMETRY_ID:
                if message.dlc == 8:
                    self.temperature = util.fuse_2_bytes(b'\x00' + b'\x00' + message.data[0:2])
                    self.voltage = util.fuse_2_bytes(b'\x00' + b'\x00' + message.data[2:4])
                    byte1 = util.split_2_bitfield(message.data[4])
                    byte2 = util.split_2_bitfield(message.data[5])
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
                    self.wrong_product = byte2[3]
                    self.current = util.fuse_2_bytes(b'\x00' + b'\x00' + message.data[6:8])
                else:
                    self.DLC_warrning = 1

            case self.SEND_RESPOND_IDENTITY_ID:
                if message.dlc == 8:
                    self.product_id = message.data[0]
                    self.product_name = self.PRODUCT_NAMES.get(message.data[0], 'unknown')
                    self.tool_id = message.data[1]
                    self.firmware_version = (message.data[2] << 8) | message.data[3]
                    self.identity_hardware_version = message.data[4]
                    self.pole_pairs = message.data[5]
                    self.identity_calibrated = message.data[6]
                    # bit 0 (least significant) I_AM_GRIPPER, bit 1 brake on idle,
                    # bit 2 commutation reversed, bit 3 settings block 2 available
                    self.identity_flags = {
                        'gripper': bool(message.data[7] & 0x01),
                        'brake_on_idle': bool(message.data[7] & 0x02),
                        'commutation_reversed': bool(message.data[7] & 0x04),
                        'settings_block2': bool(message.data[7] & 0x08),
                        'wrong_product': bool(message.data[7] & 0x10),
                        'sense_zero_measured': bool(message.data[7] & 0x20),
                        'anti_cogging_saved': bool(message.data[7] & 0x40),
                        'sense_zero_switching': bool(message.data[7] & 0x80),
                    }
                else:
                    self.DLC_warrning = 1

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
                    self.wrong_product = byte2[3]


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
        # What the driver does when the watchdog trips (_rate ms without a frame; 0 = off).
        #   'Idle'    0  legacy: whatever that firmware always did (STEPFOC brakes, Spectral coasts).
        #                The default, so an unmodified host behaves exactly as before.
        #   'Hold'    1  hold the position the joint had at the trip, for the fallback time, then the
        #                after-action (see Send_Watchdog_Fallback).
        #   'Brake'   2  short the phases.
        #   'Coast'   3  phases Hi-Z.
        #   'Descend' 4  apply the fallback current but never move faster than the fallback speed, for
        #                the fallback time, then the after-action: a joint under gravity lowers itself
        #                slowly, a gripper keeps its grip.
        # Firmware before 109 stores the byte and ignores it. The action only
        # applies when the watchdog is the SOLE active error; any other fault always gets the legacy
        # response. Hold and descend only start if the drive was in a closed-loop mode at the trip.
        if _action not in self.WATCHDOG_ACTIONS:
            raise ValueError("Send_Watchdog: unknown action %r (use one of %s)" % (_action, ', '.join(self.WATCHDOG_ACTIONS)))
        _action_byte = bytes([self.WATCHDOG_ACTIONS[_action]])

        _canid = util.combine_2_can_id(self.node_id,self.SEND_WATCHDOG_ID,0)
        watch_time = util.split_2_4_bytes(_rate)
        _combined = bytearray(watch_time + _action_byte)
        self.communication.send_can_message(message_id = _canid, data = _combined)

    # Radi
    def Send_Settings(self, mask, value):
        # Firmware V107+. Sets the bits selected by `mask` to the matching bits of `value`; bits
        # outside the mask are left alone. Exactly 2 bytes -- Spectral BLDC rejects any other length.
        _canid = util.combine_2_can_id(self.node_id,self.SEND_SETTINGS_ID,0)
        self.communication.send_can_message(message_id = _canid, data = bytearray([mask & 0xFF, value & 0xFF]))

    # Radi
    def Send_Brake_Coast(self, brake):
        # Idle-mode phase behaviour: True = brake (phases shorted), False = coast (Hi-Z).
        # Runtime only: no firmware stores brake_coast in EEPROM (Send_Save_config does NOT persist it),
        # so it returns to the firmware default at every power-up -- resend it after each boot.
        bit = self.SETTINGS_BIT_BRAKE_COAST
        self.Send_Settings(bit, bit if brake else 0)

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

    def Send_Parameter(self, number, value):
        # Set parameter `number` (PARAM_*) to `value` (int32). No answer: read it back with
        # Send_Get_Parameter. Firmware clamps values outside a parameter's range. Saved with Send_Save_config.
        if not (0 <= number <= 255 and -2**31 <= value < 2**31):
            raise ValueError('Send_Parameter: number 0-255, value int32')
        _canid = util.combine_2_can_id(self.node_id, self.SEND_PARAMETER_ID, 0)
        data = bytearray([number]) + bytearray(int(value).to_bytes(4, 'big', signed=True))
        self.communication.send_can_message(message_id=_canid, data=data)

    def Send_Get_Parameter(self, number):
        # The drive answers on RESPOND_PARAMETER_ID; the value lands in self.parameters[number]
        # (self.unknown_parameter if the firmware does not know the number).
        if not 0 <= number <= 255:
            raise ValueError('Send_Get_Parameter: number 0-255')
        _canid = util.combine_2_can_id(self.node_id, self.SEND_GET_PARAMETER_ID, 0)
        self.communication.send_can_message(message_id=_canid, data=bytearray([number]))

    def Send_Watchdog_Fallback(self, time_ms, current_mA=0, speed=1000, after='Idle'):
        # Parameters of watchdog 'Hold' and 'Descend', as four parameter frames:
        #   time_ms    how long hold/descend may run (the firmware clamps to 60000)
        #   current_mA the current 'Descend' applies (signed; 0 = let gravity do it)
        #   speed      the speed cap of 'Descend' in encoder ticks/s, either direction (16384 ticks = 1 turn)
        #   after      what follows: 'Idle' (legacy), 'Brake' or 'Coast'
        after_bytes = {'Idle': 0, 'Brake': 2, 'Coast': 3}
        if after not in after_bytes:
            raise ValueError("Send_Watchdog_Fallback: after must be 'Idle', 'Brake' or 'Coast', not %r" % (after,))
        self.Send_Parameter(self.PARAM_WD_FALLBACK_TIME, time_ms)
        self.Send_Parameter(self.PARAM_WD_FALLBACK_CURRENT, current_mA)
        self.Send_Parameter(self.PARAM_WD_FALLBACK_SPEED, speed)
        self.Send_Parameter(self.PARAM_WD_AFTER_ACTION, after_bytes[after])

    def Send_Respond_Watchdog_Fallback(self):
        # Four answers; self.watchdog_fallback becomes (time_ms, current_mA, speed, after_action).
        for number in (self.PARAM_WD_FALLBACK_TIME, self.PARAM_WD_FALLBACK_CURRENT,
                       self.PARAM_WD_FALLBACK_SPEED, self.PARAM_WD_AFTER_ACTION):
            self.Send_Get_Parameter(number)

    def Send_Tool_ID(self, tool_id):
        # What the drive is built into: 0 none, 1 SSG-48, 11-33 MSG (tens = rail 1/2/3 = 100/150/200 mm,
        # units = stepper 1/2/3 = 21.5/40/60 mm), 200-254 custom. Saved with Send_Save_config.
        if not 0 <= tool_id <= 255:
            raise ValueError('Send_Tool_ID: 0-255')
        self.Send_Parameter(self.PARAM_TOOL_ID, tool_id)

    def Send_Respond_Tool_ID(self):
        self.Send_Get_Parameter(self.PARAM_TOOL_ID)

    def _named(self, value, names, what):
        if isinstance(value, str):
            if value not in names:
                raise ValueError(f"unknown {what} {value!r}; use one of {list(names)}")
            return names[value]
        return int(value)

    def Send_Buffer_Read(self, kind, first=0, count=None):
        # Ask for buffer entries (BUFFER_*). The drive sends one frame per entry on RESPOND_BUFFER_DATA_ID (the
        # table: one per 3 values) as fast as the bus takes them, filling self.capture, self.histogram,
        # self.cogging_harmonics or self.cogging_table. Firmware 2026-11 batch 3 and later.
        if count is None:
            count = self.BUFFER_LENGTHS[kind] - first
        _canid = util.combine_2_can_id(self.node_id, self.SEND_BUFFER_READ_ID, 0)
        data = bytes([kind]) + int(first).to_bytes(2, 'big') + int(count).to_bytes(2, 'big')
        self.communication.send_can_message(message_id=_canid, data=bytearray(data))

    def _send_buffer_frame(self, kind, index, values):
        for v in values:
            if not -32768 <= int(v) <= 32767:
                raise ValueError(f"{v} does not fit in 16 bits")
        _canid = util.combine_2_can_id(self.node_id, self.SEND_BUFFER_WRITE_ID, 0)
        data = ((kind << 13) | (index & 0x1FFF)).to_bytes(2, 'big')
        data += b''.join(int(v).to_bytes(2, 'big', signed=True) for v in values)
        self.communication.send_can_message(message_id=_canid, data=bytearray(data))

    def Send_Cogging_Harmonic(self, slot, harmonic, a, b):
        # Anti-cogging slot 0-7: a*cos(h*angle) + b*sin(h*angle) mA, angle = the electrical angle from the encoder
        # (SIGNALS['electrical_angle'], 16384 per turn). Harmonic 0 clears the slot, 1-32 allowed. RAM until
        # Send_Save_config(); applied when the mode is 'harmonics'.
        if not 0 <= slot < 8 or not 0 <= harmonic <= 32:
            raise ValueError("slot 0-7, harmonic 0-32")
        self._send_buffer_frame(self.BUFFER_COGGING_HARMONICS, slot, (harmonic, a, b))

    def Send_Cogging_Table(self, values, first=0):
        # Anti-cogging table values in mA (256 per electrical turn, value i at angle i * 64), three per frame,
        # from entry `first`. RAM until Send_Save_config(); applied when the mode is 'table'. Every frame
        # writes 3 entries, so at least 3 values; the last frame overlaps the one before instead of padding.
        values = [int(round(v)) for v in values]
        if first < 0 or first + len(values) > 256:
            raise ValueError("the table has 256 entries")
        if len(values) < 3:
            raise ValueError("write at least 3 consecutive values (each frame carries 3)")
        starts = list(range(0, len(values) - 2, 3))
        if starts[-1] + 3 < len(values):
            starts.append(len(values) - 3)
        for i in starts:
            self._send_buffer_frame(self.BUFFER_COGGING_TABLE, first + i, values[i:i + 3])

    def Send_Sound(self, op, channel=0, a=0, b=0, c=0):
        # The raw sound command (SEND_SOUND_ID): op 0 stop, 1 tone, 2 queue event, 3 play queue, 4 play samples,
        # 5 samples off, 6 clear queue; a, b, c are u16.
        for v in (a, b, c):
            if not 0 <= int(v) <= 65535:
                raise ValueError('Send_Sound: a, b and c are 0-65535')
        if not 0 <= channel <= 3:
            raise ValueError('Send_Sound: channel 0-3')
        _canid = util.combine_2_can_id(self.node_id, self.SEND_SOUND_ID, 0)
        data = bytearray([op, channel]) + int(a).to_bytes(2, 'big') + int(b).to_bytes(2, 'big') + int(c).to_bytes(2, 'big')
        self.communication.send_can_message(message_id=_canid, data=data)

    def Send_Tone(self, freq_hz, current_mA=200, duration_ms=0, channel=0):
        # A tone now on channel 0-3 (up to 4 at once), loudness as a current (mA); duration 0 = until Send_Sound_Stop.
        self.Send_Sound(1, channel, int(round(freq_hz)), int(current_mA), int(duration_ms))

    def Send_Sound_Stop(self):
        # Everything off: tones, the queue, the samples.
        self.Send_Sound(0)

    def Send_Sound_Event(self, freq_hz, current_mA, wait_ms, channel=0):
        # Queue an event: set channel's pitch (0 = silence) and loudness, then wait (0: the next starts with it).
        self.Send_Sound(2, channel, int(round(freq_hz)), int(current_mA), int(wait_ms))

    def Send_Sound_Play(self):
        # Play the queued events (with loop-exact timing).
        self.Send_Sound(3)

    def Send_Sound_Clear(self):
        # Clear the queue; tones that sound keep sounding.
        self.Send_Sound(6)

    def Send_Samples(self, samples):
        # Samples (bytes or ints -128..127) into the drive's buffer, 8 per frame. Send some, then Send_Samples_Play.
        data = bytes(int(s) & 0xFF for s in samples) if not isinstance(samples, (bytes, bytearray)) else bytes(samples)
        _canid = util.combine_2_can_id(self.node_id, self.SEND_SOUND_SAMPLES_ID, 0)
        for i in range(0, len(data), 8):
            self.communication.send_can_message(message_id=_canid, data=bytearray(data[i:i + 8]))

    def Send_Samples_Play(self, rate_hz, full_scale_mV=1000):
        # Play the buffered samples at rate_hz (up to the loop rate), +-127 = full_scale_mV.
        self.Send_Sound(4, 0, int(rate_hz), int(full_scale_mV), 0)

    def Send_Samples_Stop(self):
        self.Send_Sound(5)

    @classmethod
    def decode_features(cls, value):
        # PARAM_FEATURES (79) as a list of names, then 'after calibration: ...' from bits 16-17.
        names = [n for b, n in cls.FEATURES.items() if value is not None and value >> b & 1]
        after = (value >> 16) & 3 if value is not None else 0
        names.append('after calibration: ' + {0: 'nothing', 1: 'anti-cogging', 2: 'encoder table and anti-cogging'}.get(after, str(after)))
        return names

    def Send_Cogging_Mode(self, mode, scale=None):
        # 'off', 'harmonics' or 'table' (or 0-2); scale in percent (0-200) of the stored values. Saved by Send_Save_config().
        # Since firmware batch 18 a mode the set holds nothing for is refused (it stays as it was; no answer, read
        # PARAM_COGGING_MODE back): load or identify the set first.
        self.Send_Parameter(self.PARAM_COGGING_MODE, self._named(mode, self.COGGING_MODES, 'mode'))
        if scale is not None:
            self.Send_Parameter(self.PARAM_COGGING_SCALE, int(scale))

    def Send_Capture_Setup(self, a='velocity', b='iq', c='electrical_angle', divider=1, pretrigger=0, level=0):
        # Channels (SIGNALS names or numbers), control loops per sample, samples kept before the trigger, the
        # trigger level (channel A's units). Start with Send_Capture_Trigger().
        for n, s in zip((self.PARAM_CAPTURE_SOURCE_A, self.PARAM_CAPTURE_SOURCE_B, self.PARAM_CAPTURE_SOURCE_C), (a, b, c)):
            self.Send_Parameter(n, self._named(s, self.SIGNALS, 'signal'))
        self.Send_Parameter(self.PARAM_CAPTURE_DIVIDER, int(divider))
        self.Send_Parameter(self.PARAM_CAPTURE_PRETRIGGER, int(pretrigger))
        self.Send_Parameter(self.PARAM_CAPTURE_LEVEL, int(level))

    def Send_Capture_Trigger(self, trigger):
        # 'now', 'error', 'rising', 'falling' start a capture; 'stop' stops it (the data stays).
        self.Send_Parameter(self.PARAM_CAPTURE_TRIGGER, self._named(trigger, self.TRIGGERS, 'trigger'))

    def Send_Histogram(self, x, y=None):
        # Start a histogram over x ('electrical' or 'encoder', 256 bins; clears it) of signal y, or stop it ('off').
        if y is not None:
            self.Send_Parameter(self.PARAM_HIST_Y, self._named(y, self.SIGNALS, 'signal'))
        self.Send_Parameter(self.PARAM_HIST_X, self._named(x, self.HISTOGRAM_X, 'histogram x'))

    def Send_Cogging_Calibrate(self, save=False, speed=None, trim=False):
        # Identify the anti-cogging on the drive itself: it turns one motor turn each way (the shaft must be free to
        # turn), fits the harmonics and the table, switches the harmonics on and with save=True saves. Read
        # PARAM_COGCAL for the state (COGCAL_STATES). A motion command aborts it. Firmware 2026-11 batch 5 and later.
        if speed is not None:
            self.Send_Parameter(self.PARAM_COGCAL_SPEED, int(speed))
        # trim=True (firmware batch 11): first correct the current-sense zeros from the 1st harmonic, then again.
        self.Send_Parameter(self.PARAM_COGCAL, (5 if save else 4) if trim else (2 if save else 1))

    def Send_Encoder_Calibrate(self, save=False, then_cogging=False):
        # Measure the encoder table on the drive itself: it turns the field open loop one shaft turn each way (about
        # 20 s; the shaft must be free), fits a 128-entry correction, moves the electrical offset to match and uses
        # both; save=True saves. then_cogging=True also identifies the anti-cogging afterwards and saves everything.
        # Read PARAM_ENCAL for the state (ENCAL_STATES), or PARAM_COGCAL for a combined run. Firmware batch 9.
        self.Send_Parameter(self.PARAM_ENCAL, 3 if then_cogging else (2 if save else 1))

    def Send_Respond_Telemetry(self):
        # One remote frame instead of four requests: fills temperature, voltage, the error flags (as
        # Send_Respond_State_of_Errors does) and current, all from the same moment. Firmware 2026-11 and later.
        _canid = util.combine_2_can_id(self.node_id, self.SEND_RESPOND_TELEMETRY_ID, 0)
        self.communication.send_can_message(message_id=_canid, remote_frame=True)

    def Send_Respond_Identity(self):
        # The reply fills product_id/product_name, tool_id, firmware_version, identity_hardware_version,
        # pole_pairs, identity_calibrated and identity_flags.
        _canid = util.combine_2_can_id(self.node_id, self.SEND_RESPOND_IDENTITY_ID, 0)
        self.communication.send_can_message(message_id=_canid, remote_frame=True)

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