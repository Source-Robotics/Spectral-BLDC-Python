"""The drive's own calibration features (firmware 109): the encoder table and the anti-cogging, with
Spectral_BLDC.helpers. Shows what is in force (parameter 79), the last runs and their results.

Without --run it only reads: the motor does not move.

With --run THE MOTOR TURNS BY ITSELF for about a minute, then the drive SAVES the results: it measures the encoder
table (one shaft turn each way, open loop), identifies the anti-cogging (one motor turn each way) and saves both
to its EEPROM (parameter 43 = 3). The shaft must be free to turn full turns: no load, nothing attached that cannot
turn. The drive must be calibrated first. Ctrl+C aborts the run (nothing is saved then).

Usage:  python calibration_features.py --bustype slcan --channel COM41 --node 0 [--run]
"""

import argparse
import time

import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers


def scaled(value, scale, digits=2):
    return "?" if value is None else f"{value / scale:.{digits}f}"


def show(motor):
    f = helpers.features(motor)
    if f is None:
        print("No answer to parameter 79: these features need firmware 109.")
        return False
    print("In force and running (parameter 79): "
          + (", ".join(name for name, on in f.items() if on is True) or "nothing"))
    g = {n: helpers.get_parameter(motor, n) for n in (43, 45, 52, 54, 55, 57, 59, 60, 61, 62, 63)}
    print(f"  last encoder table run: {motor.ENCAL_STATES.get(g[52], g[52])}; "
          f"last anti-cogging run: {motor.COGCAL_STATES.get(g[43], g[43])}")
    print(f"  encoder table: error {scaled(g[54], 100)} deg of the shaft peak to peak, "
          f"{scaled(g[55], 100)} deg of it once per turn; the last run moved the electrical offset "
          f"{scaled(g[57], 10, 1)} deg")
    print(f"  anti-cogging: identified cogging current {scaled(g[45], 10, 1)} mA rms")
    print(f"  current-sense zero trims {scaled(g[59], 1, 0)} / {scaled(g[60], 1, 0)} counts; "
          f"electrical offset {scaled(g[61], 10, 1)} deg; "
          f"1st harmonic at the last trim {scaled(g[62], 10, 1)} -> {scaled(g[63], 10, 1)} mA")
    return True


def run(motor):
    motor.Send_Clear_Error()
    state = helpers.errors(motor)
    if state is None or state["error"] or not state["calibrated"]:
        print("The drive does not answer, is in error, or is not calibrated:", state)
        return
    print("\nThe drive measures the encoder table, then the anti-cogging, then saves both. The motor turns.")
    motor.Send_Parameter(motor.PARAM_COGCAL, 3)          # the same as motor.Send_Encoder_Calibrate(then_cogging=True)
    state = helpers.wait_for_state(
        motor, motor.PARAM_COGCAL, timeout=180,
        on_change=lambda t, s: print(f"  {t:5.1f} s: {motor.COGCAL_STATES.get(s, s)}"))
    if state in (3, 11):                                 # done (11: without the current-sense trim)
        time.sleep(1.5)                                  # the drive is saving; it answers again after that
        print("Done and saved.\n")
    else:
        why = motor.COGCAL_STATES.get(state, state)
        if state == 8:
            enc = helpers.get_parameter(motor, motor.PARAM_ENCAL)
            why = f"{why} ({motor.ENCAL_STATES.get(enc, enc)})"
        print(f"Not finished: {why}. Is the shaft free to turn?\n")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bustype", default="slcan", help="python-can interface: slcan, pcan, socketcan, ...")
    p.add_argument("--channel", default="COM41", help="adapter port or channel, e.g. COM41, PCAN_USBBUS1, can0")
    p.add_argument("--bitrate", type=int, default=1000000)
    p.add_argument("--node", type=int, default=0, help="CAN id of the drive")
    p.add_argument("--run", action="store_true", help="run the encoder table and the anti-cogging and save them "
                                                      "(THE MOTOR TURNS)")
    args = p.parse_args()

    comm = Spectral.CanCommunication(bustype=args.bustype, channel=args.channel, bitrate=args.bitrate)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)
    try:
        if show(motor) and args.run:
            run(motor)
            show(motor)
    except KeyboardInterrupt:
        print("\nAborted.")
        motor.Send_Parameter(motor.PARAM_ENCAL, 0)       # abort whichever part is running
        motor.Send_Parameter(motor.PARAM_COGCAL, 0)
    finally:
        if args.run:
            motor.Send_Idle()
        comm.bus.shutdown()


if __name__ == "__main__":
    main()
