"""Record three signals on the drive and save them as CSV, with Spectral_BLDC.helpers (firmware 109).

MOVES THE MOTOR (by default): the drive holds its position, then steps --step ticks (1000 = 22 degrees) forward.
The capture triggers when the position error rises through a tenth of the step and keeps 50 samples from before
it: the step response, one sample every --divider control loops. The shaft must be free to turn that far.

With --step 0 the motor does not move: the drive stays idle and the capture starts at once, recording the control
interrupt's time, Iq and the encoder (turn the shaft by hand to see the encoder change).

Usage:  python capture.py --bustype slcan --channel COM41 --node 0 [--step 1000] [--divider 4] [--out capture.csv]
"""

import argparse
import csv

import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers

STEP_SIGNALS = ("position_error", "iq_setpoint", "velocity")   # channel A is also the trigger channel
IDLE_SIGNALS = ("loop_time_us", "iq", "encoder")


def record(motor, args):
    if helpers.get_parameter(motor, motor.PARAM_CAPTURE_LENGTH) is None:
        print("No answer to parameter 39: the capture needs firmware 109.")
        return None
    if not args.step:
        motor.Send_Idle()
        return helpers.capture(motor, IDLE_SIGNALS, divider=args.divider, trigger="now",
                               timeout=1.0 + 1024 * args.divider / 5000)

    motor.Send_Clear_Error()
    state = helpers.errors(motor)
    if state is None or state["error"] or not state["calibrated"]:
        print("The drive does not answer, is in error, or is not calibrated:", state)
        return None
    start = helpers.hold(motor, seconds=0.3)             # position mode where the shaft is
    if start is None:
        print("No answer to the position request.")
        return None
    level = max(1, abs(args.step) // 10)

    def drive(t):                                          # called every 10 ms while the capture waits
        target = start + (args.step if t > 0.1 else 0)
        motor.Send_data_pack_1(Position=target, Speed=0, Current=0)

    result = helpers.capture(motor, STEP_SIGNALS, divider=args.divider,
                             trigger="rising" if args.step > 0 else "falling",
                             level=level if args.step > 0 else -level, pretrigger=50,
                             timeout=1.0 + 1024 * args.divider / 5000, while_waiting=drive)
    helpers.hold(motor, seconds=0.2, position=start + args.step)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bustype", default="slcan", help="python-can interface: slcan, pcan, socketcan, ...")
    p.add_argument("--channel", default="COM41", help="adapter port or channel, e.g. COM41, PCAN_USBBUS1, can0")
    p.add_argument("--bitrate", type=int, default=1000000)
    p.add_argument("--node", type=int, default=0, help="CAN id of the drive")
    p.add_argument("--step", type=int, default=1000, help="position step in ticks (16384 = one turn); 0: no motion")
    p.add_argument("--divider", type=int, default=4, help="control loops per sample")
    p.add_argument("--out", default="capture.csv")
    args = p.parse_args()

    comm = Spectral.CanCommunication(bustype=args.bustype, channel=args.channel, bitrate=args.bitrate)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)
    try:
        result = record(motor, args)
    finally:
        motor.Send_Idle()
        comm.bus.shutdown()
    if result is None:
        print("No capture (it did not trigger within the timeout, or the read-back failed).")
        return

    dt = result.sample_period or 0.0
    first = max(result.trigger_index, 0)
    with open(args.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["sample", "time_s"] + list(result.signals))
        for i, row in enumerate(zip(*result.channels)):
            w.writerow([i, f"{(i - first) * dt:.6f}"] + list(row))
    a, b, c = result.channels
    print(f"{len(a)} samples, {dt * 1000:.3f} ms apart, trigger at sample {result.trigger_index}; wrote {args.out}")
    for name, values in zip(result.signals, result.channels):
        print(f"  {name:<15} {min(values):7d} to {max(values):7d}")


if __name__ == "__main__":
    main()
