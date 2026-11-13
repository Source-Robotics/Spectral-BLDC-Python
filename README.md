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

## Helpers

`Spectral_BLDC.helpers` sends a request and waits for the drive's answer, so reading a value takes one line
instead of a receive loop:

```python
import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers

Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)

print(helpers.read_identity(Motor1))                      # product, firmware, tool id, pole pairs, ...
print(helpers.get_parameter(Motor1, Motor1.PARAM_KPV))    # the velocity gain in force
print(helpers.set_parameter(Motor1, Motor1.PARAM_SOUND_LIMIT_MV, 2500))   # True: the drive took it
print(helpers.read_position(Motor1))                      # encoder ticks, 16384 per turn
```

A helper returns `None` when the drive does not answer within its timeout, or does not know a parameter. The
helpers read the bus themselves: use them to set up, diagnose, calibrate and play sound, not inside a fast control
loop or while another thread receives from the same bus. They are tested offline against a simulated drive:
`python -m unittest discover -s tests`.

<details>
<summary>Every helper</summary>

| Helper | What it does |
|---|---|
| `get_parameter`, `get_parameters`, `set_parameter` | read one parameter or many; write one and read it back (`True` taken, `False` clamped or refused) |
| `read_identity`, `read_telemetry`, `errors`, `read_position`, `features`, `loop_rate` | what the drive is; temperature, voltage, current and the flags in one snapshot; the error flags; the encoder position; parameter 79 in words; the control loop rate |
| `hold` | holds the shaft in position mode where it is, sending the command again so a watchdog stays fed |
| `wait_for_state` | follows a run on the drive (encoder table, anti-cogging, capture) until it ends |
| `read_buffer` | reads the capture, histogram, anti-cogging and encoder-table buffers, asking again for lost frames |
| `capture`, `start_capture`, `wait_capture`, `read_capture`, `unwrap16` | records three signals on the drive and reads them back |
| `play_events`, `stream_samples` | plays queued notes on one drive or several, topping the queue up; streams samples such as speech. With `hold=True` (the default) the drive holds its position while it plays |
| `wait_reply`, `drain`, `flush` | the receive steps the others are built on |

</details>

## Works with

| Product | Firmware | Library | Repository |
|---|---|---|---|
| Spectral Micro BLDC controller | 110 | 1.22 or newer (1.21 works; it reads 90-93 by number) | [Spectral-Micro-BLDC-controller](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller) |
| STEPFOC stepper controller | 110 | 1.22 or newer (1.21 works; it reads 90-93 by number) | [STEPFOC-stepper-controller](https://github.com/Source-Robotics/STEPFOC-stepper-controller) |
| SSG-48 adaptive electric gripper | Spectral Micro firmware | as for Spectral Micro | [SSG-48-adaptive-electric-gripper](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper) |
| MSG compliant AI stepper gripper | STEPFOC firmware | as for STEPFOC | [MSG-compliant-AI-stepper-gripper](https://github.com/Source-Robotics/MSG-compliant-AI-stepper-gripper) |

Older firmware: the compatibility pages of the
[Spectral Micro](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/appendix/compatibility/) and
[STEPFOC](https://docs.source-robotics.com/STEPFOC-docs/latest/appendix/compatibility/) manuals.

## Firmware 110

Firmware 110 adds four parameters, named from 1.22: `PARAM_RSTINT` (90, `#Rstint`), `PARAM_THERMISTOR` (91, `#Term`),
`PARAM_GRIPPER_MODE` (92, `#Gripper`, written only while the drive is idle), each 0 or 1 and kept by
`Send_Save_config()`, and `PARAM_GRIPPER_STROKE` (93, read only: the gripper's stroke in ticks, 0 until calibrated).
Brake or coast when idle (parameter 18) is kept by a save from 110 on. A 109 drive answers 90-93 as unknown.

## Firmware 109

Firmware 109 is the release from which the STEPFOC and the Spectral Micro share their version numbers. The library
speaks everything it adds, from 1.21:

- **Parameters.** One numbered list of settings and read-outs behind a single set and get command: the gains and
  limits in force, the watchdog fallback, the tool id, the motor's resistance, inductance and pole pairs, and more.
- **Identity and telemetry.** One frame says what the drive is: product, firmware, hardware, tool, pole pairs,
  calibrated or not. Another gives the temperature, the bus voltage, the current and every error flag from the same
  moment.
- **Watchdog actions.** When the host goes silent, the drive can hold its position, brake, coast or let a load down
  slowly, instead of only stopping. Hold and let-down run for a set time, then the drive brakes or coasts.
- **Capture.** A three-channel recorder inside the drive, 1024 samples per channel, started at once, on an error or
  when a signal crosses a level: step responses and faults, seen from the control loop.
- **Calibration on the drive.** The drive measures its encoder's error and its motor's cogging by itself, a turn each
  way, corrects both and saves them. Parameter 79 says what is in force and what is running.
- **Sound.** The motor plays tones, chords of up to four notes, melodies from a queue, and streamed samples such as
  speech, while it holds or moves.

Most of these need firmware 109 on the drive: older firmware does not answer the new requests, and the helpers
return `None`. Every command and parameter is on the manuals' CAN pages:
[STEPFOC](https://docs.source-robotics.com/STEPFOC-docs/latest/06-operation/can/) and
[Spectral Micro](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/can/).

## Examples

In [examples](https://github.com/Source-Robotics/Spectral-BLDC-Python/tree/main/examples):

| Example | What it does |
|---|---|
| `Send_respond_1.py` | goes through the commands: settings, gains, limits, the gripper commands, and reads the answers |
| `Get_encoder_data.py` | reads the encoder position and speed |
| `SSG48_gripper_test.py` | clears the errors, calibrates and moves the SSG-48 gripper |
| `parameters_and_status.py` | identity, telemetry, what is in force (parameter 79) and every parameter; reads only (firmware 109) |
| `capture.py` | a step response recorded on the drive, saved as CSV; **moves the motor** 1000 ticks (`--step 0` does not move it) (firmware 109) |
| `calibration_features.py` | the encoder table and anti-cogging: what is in force and the results; with `--run` **the motor turns** for about a minute and the drive saves the results (firmware 109) |
| `watchdog.py` | the watchdog actions and fallback, set, read back and put back; with `--trip` **the drive holds, then acts** when the script goes silent (firmware 109) |
| `sound.py` | a tone, a chord, a ringtone and a WAV file through the motor; **the motor is energised and holds its position** while it plays (firmware 109) |
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
