# Spectral-BLDC

**Python library for the Spectral Micro and STEPFOC motor drivers, over CAN.** Send commands to every driver on the
bus and read its encoder, current, temperature, voltage and errors; it also drives the SSG-48 and MSG grippers.

[Documentation](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/can/) ·
[PyPI](https://pypi.org/project/Spectral-BLDC/) ·
[Examples](https://github.com/Source-Robotics/Spectral-BLDC-Python/tree/main/examples) ·
[Discord](https://discord.com/invite/prjUvjmGpZ)

[![PyPI](https://img.shields.io/pypi/v/Spectral-BLDC?style=flat-square&color=143952&label=pypi)](https://pypi.org/project/Spectral-BLDC/)
[![Python](https://img.shields.io/badge/python-3.10_%7C_3.11-143952?style=flat-square)](https://pypi.org/project/Spectral-BLDC/)
[![License](https://img.shields.io/badge/license-MIT-143952?style=flat-square)](https://github.com/Source-Robotics/Spectral-BLDC-Python/blob/main/LICENSE)
[![Discord](https://img.shields.io/discord/1072498136284667955?style=flat-square&logo=discord&logoColor=white&label=discord&color=292B37)](https://discord.com/invite/prjUvjmGpZ)

## Install

```
pip install Spectral-BLDC
```

You need a CAN adapter that speaks slcan, such as the
[CANvas USB-to-CAN adapter](https://source-robotics.com/products/canvas-usb-to-can-adapter).

## Quick start

```python
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

Every command and its fields: the [CAN reference](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/can/)
in the Spectral Micro manual, and the same for [STEPFOC](https://docs.source-robotics.com/STEPFOC-docs/latest/06-operation/can/).

## Works with

| Product | Firmware | Library | Repository |
|---|---|---|---|
| Spectral Micro BLDC controller | 107 | 1.20 or newer | [Spectral-Micro-BLDC-controller](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller) |
| STEPFOC stepper controller | 108 | 1.20 or newer | [STEPFOC-stepper-controller](https://github.com/Source-Robotics/STEPFOC-stepper-controller) |
| SSG-48 adaptive electric gripper | Spectral Micro firmware | as for Spectral Micro | [SSG-48-adaptive-electric-gripper](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper) |
| MSG compliant AI stepper gripper | STEPFOC firmware | as for STEPFOC | [MSG-compliant-AI-stepper-gripper](https://github.com/Source-Robotics/MSG-compliant-AI-stepper-gripper) |

Older firmware: the compatibility pages of the
[Spectral Micro](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/appendix/compatibility/) and
[STEPFOC](https://docs.source-robotics.com/STEPFOC-docs/latest/appendix/compatibility/) manuals.

## Examples

In [examples](https://github.com/Source-Robotics/Spectral-BLDC-Python/tree/main/examples):

| Example | What it does |
|---|---|
| `Send_respond_1.py` | goes through the commands: settings, gains, limits, the gripper commands, and reads the answers |
| `Get_encoder_data.py` | reads the encoder position and speed |
| `SSG48_gripper_test.py` | clears the errors, calibrates and moves the SSG-48 gripper |
| `Advanced/Bilateral teleop 2DOF` | bilateral teleoperation with two motors, also over UDP with a Raspberry Pi |
| `Advanced/Gravity compensation` | a single link that holds itself against gravity, with its BOM, STL files and setup guide |
| `Advanced/SSG48 gripper bilateral teleop` | bilateral teleoperation with the SSG-48 gripper |

## Community

Ask in [Discussions](https://github.com/orgs/Source-Robotics/discussions), chat on
[Discord](https://discord.com/invite/prjUvjmGpZ), report a bug in
[Issues](https://github.com/Source-Robotics/Spectral-BLDC-Python/issues). Follow Source Robotics:

[![Discord](https://img.shields.io/discord/1072498136284667955?style=flat-square&logo=discord&logoColor=white&label=discord&color=143952)](https://discord.com/invite/prjUvjmGpZ)
[![Discussions](https://img.shields.io/badge/discussions-ask_a_question-143952?style=flat-square&logo=github&logoColor=white)](https://github.com/orgs/Source-Robotics/discussions)
[![YouTube](https://img.shields.io/badge/youtube-%40source--robotics-143952?style=flat-square&logo=youtube&logoColor=white)](https://www.youtube.com/@source-robotics)
[![Instagram](https://img.shields.io/badge/instagram-%40source__robotics-143952?style=flat-square&logo=instagram&logoColor=white)](https://www.instagram.com/source_robotics/)
[![X](https://img.shields.io/badge/x-%40SourceRobotics-143952?style=flat-square&logo=x&logoColor=white)](https://x.com/SourceRobotics)
[![LinkedIn](https://img.shields.io/badge/linkedin-Source_Robotics-143952?style=flat-square&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI%2BPHBhdGggZmlsbD0id2hpdGUiIGQ9Ik0yMC40NDcgMjAuNDUyaC0zLjU1NHYtNS41NjljMC0xLjMyOC0uMDI3LTMuMDM3LTEuODUyLTMuMDM3LTEuODUzIDAtMi4xMzYgMS40NDUtMi4xMzYgMi45Mzl2NS42NjdIOS4zNTFWOWgzLjQxNHYxLjU2MWguMDQ2Yy40NzctLjkgMS42MzctMS44NSAzLjM3LTEuODUgMy42MDEgMCA0LjI2NyAyLjM3IDQuMjY3IDUuNDU1djYuMjg2ek01LjMzNyA3LjQzM2MtMS4xNDQgMC0yLjA2My0uOTI2LTIuMDYzLTIuMDY1IDAtMS4xMzguOTItMi4wNjMgMi4wNjMtMi4wNjMgMS4xNCAwIDIuMDY0LjkyNSAyLjA2NCAyLjA2MyAwIDEuMTM5LS45MjUgMi4wNjUtMi4wNjQgMi4wNjV6bTEuNzgyIDEzLjAxOUgzLjU1NVY5aDMuNTY0djExLjQ1MnpNMjIuMjI1IDBIMS43NzFDLjc5MiAwIDAgLjc3NCAwIDEuNzI5djIwLjU0MkMwIDIzLjIyNy43OTIgMjQgMS43NzEgMjRoMjAuNDUxQzIzLjIgMjQgMjQgMjMuMjI3IDI0IDIyLjI3MVYxLjcyOUMyNCAuNzc0IDIzLjIgMCAyMi4yMjIgMGguMDAzeiIvPjwvc3ZnPg%3D%3D)](https://www.linkedin.com/company/source-robotics/)
[![Blog](https://img.shields.io/badge/blog-source--robotics.com-143952?style=flat-square&logo=rss&logoColor=white)](https://source-robotics.com/blogs/blog)

## Licensing

The library is under the [MIT License](https://github.com/Source-Robotics/Spectral-BLDC-Python/blob/main/LICENSE).
