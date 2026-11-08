"""The watchdog's actions and fallback (firmware 109): what the drive does when the host goes silent, with
Spectral_BLDC.helpers.

Reads the settings in force, sets the ones given, reads them back, and puts the old ones back at the end (nothing
is saved; motor.Send_Save_config() would keep them). The motor does not move, unless:

--trip  THE DRIVE HOLDS, THEN ACTS ON ITS OWN: it holds its position (position mode) for 2 s while this script
        talks to it, then the script goes silent. After --timeout ms the watchdog trips and the drive does
        --action; try the shaft by hand. Bench only, motor without load: never try Descend or Coast first on a
        joint that carries a load.

Actions: Idle (as before firmware 109: the STEPFOC brakes, the Spectral Micro coasts), Brake (phases shorted),
Coast (phases open), Hold (keeps the position it had for --fb-time ms, then --after), Descend (applies --fb-current
but never turns faster than --fb-speed ticks/s, for --fb-time ms, then --after: a joint under gravity lowers itself
slowly, a gripper keeps its grip). The action applies when the watchdog is the only error; hold and descend only
start from a closed-loop mode. Motion commands and requests (status, parameters) feed the watchdog.

Usage:  python watchdog.py --bustype slcan --channel COM41 --node 0 --timeout 200 --action Hold [--trip]
"""

import argparse
import time

import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers

ACTION_NAMES = {v: k for k, v in Spectral.SpectralCAN.WATCHDOG_ACTIONS.items()}
AFTER_NAMES = {0: "Idle", 2: "Brake", 3: "Coast"}


def read_settings(motor):
    numbers = (motor.PARAM_WD_TIME, motor.PARAM_WD_ACTION, motor.PARAM_WD_FALLBACK_TIME,
               motor.PARAM_WD_FALLBACK_CURRENT, motor.PARAM_WD_FALLBACK_SPEED, motor.PARAM_WD_AFTER_ACTION)
    return dict(zip(("time", "action", "fb_time", "fb_current", "fb_speed", "after"),
                    (helpers.get_parameter(motor, n) for n in numbers)))


def describe(s):
    return (f"watchdog {s['time']} ms ({'off' if s['time'] == 0 else ACTION_NAMES.get(s['action'], s['action'])}); "
            f"fallback {s['fb_time']} ms, {s['fb_current']} mA, at most {s['fb_speed']} ticks/s, "
            f"then {AFTER_NAMES.get(s['after'], s['after'])}")


def trip(motor, args):
    motor.Send_Clear_Error()
    state = helpers.errors(motor)
    if state is None or state["error"] or not state["calibrated"]:
        print("The drive does not answer, is in error, or is not calibrated:", state)
        return
    position = helpers.read_position(motor)
    print(f"\nHolding position {position} for 2 s ...")
    helpers.hold(motor, seconds=2.0, position=position)  # every frame feeds the watchdog
    silence = args.timeout / 1000 + (args.fb_time / 1000 if args.action in ("Hold", "Descend") else 0) + 3
    print(f"Silent now: the watchdog trips after {args.timeout} ms and the drive does {args.action}"
          + (f", then {args.after} after {args.fb_time} ms" if args.action in ("Hold", "Descend") else "")
          + f". Try the shaft by hand ({silence:.0f} s).")
    time.sleep(silence)                                  # send nothing: any frame would feed the watchdog
    state = helpers.errors(motor)
    print("Watchdog error after the silence:", "yes" if state and state["watchdog_error"] else "no")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bustype", default="slcan", help="python-can interface: slcan, pcan, socketcan, ...")
    p.add_argument("--channel", default="COM41", help="adapter port or channel, e.g. COM41, PCAN_USBBUS1, can0")
    p.add_argument("--bitrate", type=int, default=1000000)
    p.add_argument("--node", type=int, default=0, help="CAN id of the drive")
    p.add_argument("--timeout", type=int, default=200, help="ms without a frame before the watchdog trips")
    p.add_argument("--action", type=str.capitalize, default="Hold", choices=list(ACTION_NAMES.values()))
    p.add_argument("--fb-time", type=int, default=2000, help="ms Hold or Descend runs (at most 60000)")
    p.add_argument("--fb-current", type=int, default=0, help="mA Descend applies (signed)")
    p.add_argument("--fb-speed", type=int, default=1000, help="ticks/s Descend never exceeds (16384 = 1 turn/s)")
    p.add_argument("--after", type=str.capitalize, default="Brake", choices=list(AFTER_NAMES.values()),
                   help="what follows Hold or Descend")
    p.add_argument("--trip", action="store_true", help="let the watchdog trip (THE DRIVE HOLDS, THEN ACTS)")
    args = p.parse_args()

    comm = Spectral.CanCommunication(bustype=args.bustype, channel=args.channel, bitrate=args.bitrate)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)
    old = None
    try:
        old = read_settings(motor)
        if None in old.values():
            print("No answer to the watchdog parameters: actions and fallback need firmware 109.")
            return
        print("In force:", describe(old))
        motor.Send_Watchdog_Fallback(args.fb_time, args.fb_current, args.fb_speed, args.after)
        motor.Send_Watchdog(args.timeout, args.action)
        print("Set:     ", describe(read_settings(motor)))
        if args.trip:
            trip(motor, args)
    finally:
        if old is not None and None not in old.values():  # put the old settings back
            motor.Send_Watchdog(old["time"], ACTION_NAMES.get(old["action"], "Idle"))
            motor.Send_Watchdog_Fallback(old["fb_time"], old["fb_current"], old["fb_speed"],
                                         AFTER_NAMES.get(old["after"], "Idle"))
            motor.Send_Clear_Error()                     # a watchdog error from the trip or a slow moment
            motor.Send_Idle()
            print("Put back:", describe(read_settings(motor)))
        comm.bus.shutdown()


if __name__ == "__main__":
    main()
