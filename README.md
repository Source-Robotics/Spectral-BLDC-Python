# Spectral-BLDC

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)  ![Issues](https://img.shields.io/github/issues/Source-Robotics/Spectral-BLDC-Python) ![release](https://img.shields.io/github/v/release/Source-Robotics/Spectral-BLDC-Python)

Python lib for controlling Spectral BLDC motor controllers over CAN bus. For more info about this API and all available commands check [DOCS](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/can/)

# Related projects

| Product | Description | Links |
|---|---|---|
| **Spectral Micro BLDC Controller** | Compact brushless motor controller with CAN bus, position/velocity/torque/impedance control modes | [GitHub](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller) |
| **SSG-48 Adaptive Electric Gripper** | Adaptive electric gripper controllable over CAN bus | [GitHub](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper) |
| **MSG Gripper** | Compliant AI stepper gripper with force control, open-source firmware, GUI and Python API; designed for assembly tasks and AI training | [Store](https://source-robotics.com/products/msg-gripper) · [GitHub](https://github.com/Source-Robotics/MSG-compliant-AI-stepper-gripper) · [Docs](https://docs.source-robotics.com/MSG-gripper-docs/) |
| **STEPFOC Stepper Controller** | FOC-based stepper motor controller that turns low-cost steppers into high-performance robotic servos; CAN & UART, 14-bit encoder, open-source | [Store](https://source-robotics.com/products/stepfoc-stepper-controller) · [GitHub](https://github.com/Source-Robotics/STEPFOC-stepper-controller) · [Docs](https://docs.source-robotics.com/STEPFOC-docs/) |

# How to install

  pip install Spectral-BLDC

# Basic example


``` py 
import Spectral_BLDC as Spectral
import time


Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)

while True:

    Motor1.Send_Respond_Encoder_data()

    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=0.2)

    if message is not None:

        Motor1.UnpackData(message,UnpackedMessageID)
        print(f"Motor position is: {Motor1.position}")
        print(f"Motor speed is: {Motor1.speed}")

    else:
        print("No message after timeout period!")
    print("")
    time.sleep(1 )
```


# More examples

Check out the [Examples folder](https://github.com/Source-Robotics/Spectral-BLDC-Python/tree/main/examples) for more examples!
Available examples:
* Send_respond_1
* Get_encoder_data
* SSG48_gripper_test
