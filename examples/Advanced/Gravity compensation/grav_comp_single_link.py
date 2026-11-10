from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS
from math import pi, sin, cos
import numpy as np
from oclock import Timer, loop, interactiveloop
import Spectral_BLDC as Spectral
import math
import time

# Set interval in seconds
INTERVAL_S = 0.005 #  0.005 - 0.01

# link 1
# We are not taking into account mass and inertia of the ROTOR!!!!
# Check our blog post, github repo, youtube video or docs to see how this is calculated!
m1 = 0.242600 # Mass of the whole link
l1 = 0.112 # Length of the link, 0.112 adjusted
com = [-0.0262031,0,0] # Position of COM with respect to the link frame

# inertia tensor of link with respect to
# center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
I=[0.0000474, 0.0001519, 0.0001528, 0, 0, 0] #Taken at the center of mass
g = 9.81
payload = 0
Kt = 0.30240 # Torque constant of the motor 
zero_pos = 2703 # Value of encoder when the motor is idle
MAX_ENCODER_VALUE = 2**14 # 14 bit encoder

# actuator inertia
Jm= 0.000015 #50e-6 

# actuator viscous friction coefficient (measured at the motor)
B=1e-4

# actuator Coulomb friction coefficient for
# direction [-,+] (measured at the motor)
Tc=[0.02, -0.02]

Commanded_Current = 0 # Current we will command to the motor
prev_speed = 0 # Prev speed value used to calculate acceleration
prev_time = 0

# Init the robot object 
L1 = RevoluteDH(a=l1, m=m1, r=com, I = I, Jm = Jm, B = B, Tc = Tc)
robot = DHRobot([L1], gravity=[0, g, 0])

# Initi Motor CAN 
Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM32', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)


# Function to convert ticks to radians
def revert_to_single_turn(encoder_value):
    single_turn_position =  encoder_value % MAX_ENCODER_VALUE
    radians = (single_turn_position / MAX_ENCODER_VALUE) * (1 * 2 * math.pi)
    return radians

# Function to convert ticks per second to radians per second
def ticks_to_radians(ticks_per_second):
    # Convert ticks per second to radians per tick
    radians_per_tick = (1 * 2 * math.pi) / MAX_ENCODER_VALUE
    # Convert ticks per second to radians per second
    radians_per_second = ticks_per_second * radians_per_tick
    return radians_per_second

# Low pass for accel
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None

    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value


# Define the alpha value for the low-pass filter (adjust as needed)
alpha_accel = 0.2  # Smaller alpha for more smoothing 

# Create an instance of the low-pass filter for acceleration
accel_filter = LowPassFilter(alpha_accel)
# Set interval

timer = Timer(interval=INTERVAL_S, warnings=False, precise=True)
while timer.elapsed_time < 1100000:

    time1 = time.perf_counter()
    Motor1.Send_data_pack_1(Position=None, Speed= None, Current = max(-2800, min(2800, Commanded_Current)))  # the drive's limit

    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=None) 

    if message is not None:
        
        # Print current motor data received from CAN
        Motor1.UnpackData(message,UnpackedMessageID)
        print(f"Motor position ticks: {Motor1.position}")
        #print(f"Motor Current: {Motor1.current}")
        #print(f"Motor Speed: {Motor1.speed}")
        #print(f"Motor timestamp: {Motor1.timestamp}")

        # Convert the received position to radians and offset everything to match our DH diagram
        # - 4096 is 90 degre; motor is at 0 position when it looks like this:  O--
        radians_position  = revert_to_single_turn(Motor1.position - zero_pos - 4096)
        #radians_position  = get_radians(Motor1.position - zero_pos - 4096)
        print(f"Single turn position:", {radians_position} )

        # Convert speed to radians per second
        radians_per_second = ticks_to_radians(Motor1.speed)
        #print(f"Radians/s: ", {radians_per_second} )

        # Calculate the acceleration
        accel = (radians_per_second - prev_speed) / INTERVAL_S

        # NOTE NOT USED
        # Filter the acceleration
        filtered_accel = accel_filter.update(accel)
        #print(f"Filtered acceleration: {filtered_accel}")
        prev_speed = radians_per_second

        # NOTE ACCELRATION AND INTERTIA MATRIX GIVE US THE 
        # TORQUE NEEDED TO ACCELERATE THE SERIAL LINK MANIPULATOR 
        # NOT NEEDED FOR GRAVITY COMPENSATION!
        Grav_load = robot.rne([radians_position],[0],[0])  #radians_per_second
 
        #Grav_load = robot.rne([radians_position],[0],[0])
        #print(f"gravload is",Grav_load) # Units of Nm

        # Proof that Coriolis and centripetal forces do NOT exist in a 1-DOF system!
        #Cor_comp = robot.coriolis(radians_position,radians_per_second) * 1000  #radians_per_second
        #print(f"Cor comp is",Cor_comp) # Units of Nm

        # T = Kt * Iq
        # NOTE the commanded current needs to be reverese of calculated current!
        # Since we want our motor to "fight" the gravity
        Commanded_Current = -int((Grav_load[0] / Kt) * 1000) # Units of mA
        #print(f"Needed current is: ",{Commanded_Current}) # Units of mA

        time2 = time.perf_counter()
        print(f"Loop exec time is: {(time2-time1) * 1000} ms")

    else:
        print("No message after timeout period!")

    #print("Elapsed time: {:.3f}".format(timer.elapsed_time))

    timer.checkpt()