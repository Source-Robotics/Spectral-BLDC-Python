"""Identity, state, features (parameter 79) and every parameter of a drive, with Spectral_BLDC.helpers.

READS ONLY: the motor does not move. Needs firmware 109 (an older drive only answers the error flags).
With --tool-id N it also writes the tool id, to show set_parameter (in RAM until Send_Save_config()).

Usage:  python parameters_and_status.py --bustype slcan --channel COM41 --node 0 [--tool-id 1]
"""

import argparse

import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers


def value_text(number, value):
    if isinstance(value, float):
        return f"{value:.6g}"
    if number in (79, 80):                      # bit fields
        return f"0x{value & 0xFFFFFFFF:08X}"
    if number == 84:                            # the git commit, bit 28 = built with uncommitted changes
        return f"{value & 0xFFFFFFF:07x}" + (" (uncommitted changes)" if value >> 28 & 1 else "")
    if number in (86, 87):                      # board and calibration ID
        return f"{value & 0xFFFFFFFF:08X}"
    return str(value)


def show(motor, args):
    ident = helpers.read_identity(motor)
    if ident is None:
        state = helpers.errors(motor)
        if state is None:
            print(f"No answer from node {args.node}: check the adapter, the channel and the CAN id.")
        else:
            print("The drive answers, but not the identity request: its firmware is older than 109.")
            print("Flags:", ", ".join(k for k, v in state.items() if v) or "none")
        return

    flags = [k.replace("_", " ") for k, v in ident["flags"].items() if v]
    print(f"{ident['product']}, firmware {ident['firmware_version']}, hardware {ident['hardware_version']}, "
          f"tool id {ident['tool_id']}, {ident['pole_pairs']} pole pairs, "
          f"{'calibrated' if ident['calibrated'] else 'NOT calibrated'}")
    print("  identity flags:", ", ".join(flags) or "none")

    t = helpers.read_telemetry(motor)
    if t is not None:
        on = [k.replace("_", " ") for k, v in t.items() if v is True]
        print(f"  {t['temperature']} deg C, {t['voltage'] / 1000:.2f} V, Iq {t['current']} mA; flags: "
              + (", ".join(on) or "none"))

    f = helpers.features(motor)
    if f is not None:
        print("\nIn force and running (parameter 79):")
        print("  " + (", ".join(name for name, on in f.items() if on is True) or "nothing"))

    print("\nA few parameters:")
    for n in (motor.PARAM_KPP, motor.PARAM_KPV, motor.PARAM_KIV, motor.PARAM_CURRENT_LIMIT, motor.PARAM_WD_TIME,
              motor.PARAM_WD_ACTION, motor.PARAM_SOUND_LIMIT_MV):
        v = helpers.get_parameter(motor, n)
        print(f"  {n:3d}  {motor.PARAMETER_NAMES[n]:<32} {'no answer' if v is None else value_text(n, v)}")

    if args.tool_id is not None:
        ok = helpers.set_parameter(motor, motor.PARAM_TOOL_ID, args.tool_id)
        if ok:
            print(f"\nTool id {args.tool_id}: taken, in RAM (motor.Send_Save_config() keeps it).")
        elif ok is False:
            print(f"\nTool id {args.tool_id}: not taken; the drive holds {motor.parameters.get(motor.PARAM_TOOL_ID)}.")
        else:
            print("\nTool id: no answer.")

    print("\nEvery parameter (like #Par on the UART):")
    for n, v in helpers.get_parameters(motor).items():
        name = motor.PARAMETER_NAMES.get(n) or f"loop profile: {motor.PARAM_PROFILE[n]}, 0.1 us"
        print(f"  {n:3d}  {name:<46} {value_text(n, v)}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bustype", default="slcan", help="python-can interface: slcan, pcan, socketcan, ...")
    p.add_argument("--channel", default="COM41", help="adapter port or channel, e.g. COM41, PCAN_USBBUS1, can0")
    p.add_argument("--bitrate", type=int, default=1000000)
    p.add_argument("--node", type=int, default=0, help="CAN id of the drive")
    p.add_argument("--tool-id", type=int, help="write this tool id: 0 none, 1 SSG-48, 11-33 MSG, 200-254 custom")
    args = p.parse_args()

    comm = Spectral.CanCommunication(bustype=args.bustype, channel=args.channel, bitrate=args.bitrate)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)
    try:
        show(motor, args)
    finally:
        comm.bus.shutdown()


if __name__ == "__main__":
    main()
