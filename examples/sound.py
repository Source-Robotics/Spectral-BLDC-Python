"""Sound through the motor (firmware 109): a tone, a chord, a ringtone through the drive's queue and a WAV file,
with Spectral_BLDC.helpers and Spectral_BLDC.music.

THE MOTOR IS ENERGISED: a drive plays sound only while it holds or moves, so this script holds the shaft where it
is (position mode) while it plays. The motor does not turn, but it resists a hand. It is loudest fixed to something
that rings, such as a box or a table. At the end: sound off, idle.

  --wav FILE    also play a WAV file (8 or 16 bit; speech or music) as samples, at half the drive's loop rate
  --current MA  loudness of the notes, as a current (default 250 mA)
  --volume MV   full scale of the samples, mV (default 2000; the drive limits sound to parameter 81, 2000 mV
                unless it is set higher, at most 6000)

Usage:  python sound.py --bustype slcan --channel COM41 --node 0 [--wav hello.wav]
"""

import argparse

import Spectral_BLDC as Spectral
from Spectral_BLDC import helpers, music

# Beethoven's Ode to Joy (public domain) as a ringtone (RTTTL): 30 notes, 61 queue events; the queue holds 31, so
# play_events tops it up while it plays
RINGTONE = "Ode:d=4,o=5,b=140:e,e,f,g,g,f,e,d,c,c,d,e,e.,8d,2d,e,e,f,g,g,f,e,d,c,c,d,e,d.,8c,2c"
CHORD = ("C4", "E4", "G4", "C5")


def play(motor, args):
    if helpers.get_parameter(motor, motor.PARAM_SOUND_STATE) is None:
        print("No answer to parameter 80: sound needs firmware 109.")
        return
    motor.Send_Clear_Error()
    state = helpers.errors(motor)
    if state is None or state["error"] or not state["calibrated"]:
        print("The drive does not answer, is in error, or is not calibrated:", state)
        return
    position = helpers.hold(motor, seconds=0.3)          # position mode where the shaft is
    if position is None:
        print("No answer to the position request.")
        return

    print("A tone: A4, 440 Hz, 0.6 s")
    motor.Send_Tone(440, args.current, 600)
    helpers.hold(motor, seconds=0.9, position=position)  # keeps holding (and a watchdog fed) while it sounds

    print("A chord: four notes at once, " + " ".join(CHORD))
    for channel, name in enumerate(CHORD):
        motor.Send_Tone(music.note_hz(name), args.current, 1500, channel=channel)
    helpers.hold(motor, seconds=1.8, position=position)

    name, melody = music.parse_rtttl(RINGTONE)
    events = music.melody_events(melody, args.current)
    print(f"A ringtone through the queue: {name}, {len(melody)} notes, {len(events)} events")
    if not helpers.play_events(motor, events):
        print("  it did not play to the end")

    if args.wav:
        rate = (helpers.loop_rate(motor) or 5000) // 2
        r_mohm = helpers.get_parameter(motor, motor.PARAM_R_MOHM)
        l_uh = helpers.get_parameter(motor, motor.PARAM_L_UH)
        tau = (l_uh / 1e6) / (r_mohm / 1000) if r_mohm and l_uh else 0.0   # the coil's L / R: boost the highs
        samples = music.wav_samples(args.wav, rate, coil_tau_s=tau, clip_percentile=0.95)
        print(f"{args.wav}: {len(samples) / rate:.1f} s at {rate} samples/s, full scale {args.volume} mV")
        dry = helpers.stream_samples(motor, samples, rate, args.volume)
        print(f"  done; the buffer ran dry {dry} time(s)")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bustype", default="slcan", help="python-can interface: slcan, pcan, socketcan, ...")
    p.add_argument("--channel", default="COM41", help="adapter port or channel, e.g. COM41, PCAN_USBBUS1, can0")
    p.add_argument("--bitrate", type=int, default=1000000)
    p.add_argument("--node", type=int, default=0, help="CAN id of the drive")
    p.add_argument("--wav", help="a WAV file to play as samples")
    p.add_argument("--current", type=int, default=250, help="loudness of the notes, mA")
    p.add_argument("--volume", type=int, default=2000, help="full scale of the samples, mV")
    args = p.parse_args()

    comm = Spectral.CanCommunication(bustype=args.bustype, channel=args.channel, bitrate=args.bitrate)
    motor = Spectral.SpectralCAN(node_id=args.node, communication=comm)
    try:
        play(motor, args)
    finally:
        motor.Send_Sound_Stop()
        motor.Send_Idle()
        comm.bus.shutdown()


if __name__ == "__main__":
    main()
