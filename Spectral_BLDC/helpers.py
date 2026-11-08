"""Blocking request-and-answer helpers for the STEPFOC and Spectral Micro drives (firmware 109).

SpectralCAN sends one frame per call, and an answer is only decoded when you pass it to UnpackData, so reading a
value means writing a receive loop. These functions are that loop: each sends a request, receives until the drive's
answer arrives or a timeout passes, and returns the value.

    import Spectral_BLDC as Spectral
    from Spectral_BLDC import helpers

    comm = Spectral.CanCommunication(bustype='slcan', channel='COM41', bitrate=1000000)
    motor = Spectral.SpectralCAN(node_id=0, communication=comm)

    print(helpers.read_identity(motor))                     # product, firmware, tool, pole pairs, ...
    print(helpers.get_parameter(motor, motor.PARAM_KPV))     # the velocity gain in force (a float)
    helpers.set_parameter(motor, motor.PARAM_SOUND_LIMIT_MV, 2500)   # True: the drive took it

Good to know:

- None means no answer within the timeout, or a parameter this firmware does not know. Most of what is here needs
  firmware 109: older firmware does not answer a parameter request at all.
- The helpers receive from the bus themselves. Every frame from the motor's node is unpacked into the motor object
  as it arrives; frames from other nodes are dropped, unless their SpectralCAN objects are passed as `others`, in
  which case they are unpacked into those. Do not call a helper while another thread receives from the same bus.
- They wait for answers, so they are for setting up, diagnostics, calibration and sound, not for the inside of a
  fast control loop.
- Values travel in the drive's units: encoder ticks (16384 per motor turn), ticks/s, mA, mV, ms.
"""

import struct
import time
from typing import NamedTuple, Optional

from .Spectral_BLDC import SpectralCAN

TICKS_PER_TURN = 16384
LOOP_RATES = {1: 6250, 2: 5000}  # control loop rate in Hz by product id (1 STEPFOC, 2 Spectral Micro)

# Parameters a write starts or stops something with; reading them gives a state, not the value written
ACTION_PARAMETERS = frozenset({SpectralCAN.PARAM_CAPTURE_TRIGGER, SpectralCAN.PARAM_COGCAL, SpectralCAN.PARAM_ENCAL})

# The states of a run that mean "still going", by the parameter that reports them
BUSY_STATES = {
    SpectralCAN.PARAM_ENCAL: (1, 2),             # turning forward, turning backward
    SpectralCAN.PARAM_COGCAL: (1, 2, 7),         # turning forward, backward, measuring the encoder table first
    SpectralCAN.PARAM_CAPTURE_STATE: (1, 2),     # armed, recording
}

_HOLD_PERIOD = 0.2  # seconds between position-hold frames while sound plays


# --- receiving ---------------------------------------------------------------------------------------------------

def _route(motor, others):
    """{node_id: SpectralCAN} of the motors whose frames are unpacked."""
    nodes = {m.node_id: m for m in others}
    nodes[motor.node_id] = motor
    return nodes


def _unpack(nodes, message, unpacked):
    target = nodes.get(unpacked.node_id)
    if target is not None:
        target.UnpackData(message, unpacked)
    return target


def wait_reply(motor, command_id, timeout=0.2, others=()):
    """Receive until a frame with `command_id` arrives from the motor's node; return that frame, or None after
    `timeout` seconds.

    Every frame from the motor's node is unpacked into `motor` on the way (a heartbeat too), so right after this
    returns, the motor's attributes hold what the answer said. Frames from the nodes of `others` are unpacked into
    those objects; the rest are dropped.
    """
    nodes = _route(motor, others)
    comm = motor.communication
    deadline = time.perf_counter() + timeout
    while True:
        left = deadline - time.perf_counter()
        if left <= 0:
            return None
        message, unpacked = comm.receive_can_messages(timeout=left)
        if message is None:
            return None
        if _unpack(nodes, message, unpacked) is motor and unpacked.command_id == command_id:
            return message


def drain(motor, seconds, others=()):
    """Receive and unpack every frame for `seconds` (keeps the adapter's queue short while the drive answers
    motion commands)."""
    nodes = _route(motor, others)
    comm = motor.communication
    deadline = time.perf_counter() + seconds
    while True:
        left = deadline - time.perf_counter()
        if left <= 0:
            return
        message, unpacked = comm.receive_can_messages(timeout=left)
        if message is None:
            return
        _unpack(nodes, message, unpacked)


def flush(motor, others=(), limit=1000):
    """Unpack the frames already waiting in the adapter and return at once, so a late answer to an earlier request
    is not taken for the answer to the next one. Returns how many frames were read."""
    nodes = _route(motor, others)
    comm = motor.communication
    for n in range(limit):
        message, unpacked = comm.receive_can_messages(timeout=0)
        if message is None:
            return n
        _unpack(nodes, message, unpacked)
    return limit


# --- parameters ---------------------------------------------------------------------------------------------------

def get_parameter(motor, number, timeout=0.3, retries=1, others=()):
    """The value of parameter `number` (motor.PARAM_*) in force on the drive, or None.

    A float for the gains and limits (motor.FLOAT_PARAMETERS), an int otherwise. None when the drive does not know
    the number (it says so at once) or did not answer within `timeout` seconds, after `retries` more requests. The
    value also stays in motor.parameters[number].
    """
    flush(motor, others)
    for _ in range(retries + 1):
        motor.parameters.pop(number, None)
        motor.unknown_parameter = None
        motor.Send_Get_Parameter(number)
        deadline = time.perf_counter() + timeout
        while True:
            if number in motor.parameters:
                return motor.parameters[number]
            if motor.unknown_parameter == number:
                return None
            left = deadline - time.perf_counter()
            if left <= 0 or wait_reply(motor, motor.RESPOND_PARAMETER_ID, left, others) is None:
                break
    return None


def get_parameters(motor, numbers=None, timeout=0.3):
    """Several parameters, one request each: {number: value} of those the drive answered.

    `numbers` defaults to every parameter the library has a name for (motor.PARAMETER_NAMES, and the STEPFOC's
    loop profile 66-70), which gives a listing like the UART's #Par. Unknown numbers are left out.
    """
    if numbers is None:
        numbers = sorted(set(motor.PARAMETER_NAMES) | set(motor.PARAM_PROFILE))
    values = {}
    for n in numbers:
        v = get_parameter(motor, n, timeout=timeout)
        if v is not None:
            values[n] = v
    return values


def set_parameter(motor, number, value, check=True, timeout=0.3, others=()):
    """Write parameter `number` and, with check=True, read it back.

    Returns True when the drive holds `value` afterwards, False when it answered with another value (it clamped
    the value, refused it in this state, or the parameter is read-only), None when it did not answer (or does not
    know the number), and None with check=False. What the drive holds is in motor.parameters[number].

    The parameters that start something (ACTION_PARAMETERS: the capture trigger 28, the anti-cogging run 43, the
    encoder table 52) read back a state, not the value written: they are written and not checked; follow them with
    wait_for_state() or wait_capture(). The float parameters (the gains and limits, motor.FLOAT_PARAMETERS) travel
    as their bit pattern; firmware 109 builds from 572a4ad (STEPFOC) / acaed1a (Spectral) accept them, older ones
    refuse them (False): set them there with their own commands (Send_Position_Gains, Send_Limits, ...).

    A written value is in RAM until motor.Send_Save_config().
    """
    if number in motor.FLOAT_PARAMETERS:
        want = struct.unpack('>f', struct.pack('>f', float(value)))[0]       # as the drive will hold it
        motor.Send_Parameter(number, struct.unpack('>i', struct.pack('>f', want))[0])
        if not check:
            return None
        got = get_parameter(motor, number, timeout=timeout, others=others)
        if got is None:
            return None
        return abs(got - want) <= 1e-6 * max(1.0, abs(want))
    if int(value) != value:
        raise ValueError(f"parameter {number} takes an integer, not {value!r}")
    motor.Send_Parameter(number, int(value))
    if not check or number in ACTION_PARAMETERS:
        return None
    got = get_parameter(motor, number, timeout=timeout, others=others)
    if got is None:
        return None
    return got == int(value)


# --- state --------------------------------------------------------------------------------------------------------

def _flags(motor):
    names = ('error', 'temperature_error', 'encoder_error', 'vbus_error', 'driver_error', 'velocity_error',
             'current_error', 'estop_error', 'watchdog_error', 'wrong_product', 'calibrated', 'activated')
    return {n: bool(getattr(motor, n)) for n in names}


def read_identity(motor, timeout=0.3, retries=1):
    """What the drive is (firmware 109), or None.

    {'product_id': 1 STEPFOC or 2 Spectral Micro, 'product': its name, 'firmware_version': 109, 'hardware_version',
    'tool_id', 'pole_pairs', 'calibrated': bool, 'flags': {'gripper', 'brake_on_idle', 'commutation_reversed',
    'settings_block2', 'wrong_product', 'sense_zero_measured', 'anti_cogging_saved', 'sense_zero_switching'}}.
    The same values stay in the motor's attributes (product_id, product_name, firmware_version, ...).
    """
    flush(motor)
    for _ in range(retries + 1):
        motor.Send_Respond_Identity()
        if wait_reply(motor, motor.SEND_RESPOND_IDENTITY_ID, timeout) is not None:
            return {'product_id': motor.product_id, 'product': motor.product_name,
                    'firmware_version': motor.firmware_version, 'hardware_version': motor.identity_hardware_version,
                    'tool_id': motor.tool_id, 'pole_pairs': motor.pole_pairs,
                    'calibrated': bool(motor.identity_calibrated), 'flags': dict(motor.identity_flags)}
    return None


def read_telemetry(motor, timeout=0.2):
    """One snapshot (firmware 109): {'temperature': deg C, 'voltage': bus mV, 'current': Iq mA} and the flags of
    errors(), all from the same moment. None if no answer."""
    flush(motor)
    motor.Send_Respond_Telemetry()
    if wait_reply(motor, motor.SEND_RESPOND_TELEMETRY_ID, timeout) is None:
        return None
    out = {'temperature': motor.temperature, 'voltage': motor.voltage, 'current': motor.current}
    out.update(_flags(motor))
    return out


def errors(motor, timeout=0.2):
    """The error and state flags as bools, or None: 'error' (any error), 'temperature_error', 'encoder_error',
    'vbus_error', 'driver_error', 'velocity_error', 'current_error', 'estop_error', 'watchdog_error',
    'wrong_product' (this firmware is on the other product's board; firmware 109), 'calibrated', 'activated'.
    Works with older firmware too."""
    flush(motor)
    motor.Send_Respond_State_of_Errors()
    if wait_reply(motor, motor.SEND_RESPOND_STATE_OF_ERRORS_ID, timeout) is None:
        return None
    return _flags(motor)


def read_position(motor, timeout=0.2, others=()):
    """The encoder position in ticks (16384 per motor turn, multi-turn), or None. The speed of the same answer is in
    motor.speed (ticks/s). Works with older firmware too."""
    flush(motor, others)
    motor.Send_Respond_Encoder_data()
    if wait_reply(motor, motor.SEND_RESPOND_ENCODER_DATA_ID, timeout, others) is None:
        return None
    return motor.position


def features(motor, timeout=0.3):
    """Parameter 79 decoded: what is in force and what runs, or None (firmware before 109).

    {name: bool} for every motor.FEATURES name ('calibrated', 'encoder table in use', 'anti-cogging on',
    'calibration running', 'sound playing', ...), and 'after calibration': what runs after a successful
    calibration ('nothing', 'anti-cogging', 'encoder table and anti-cogging'). The raw bits stay in
    motor.parameters[79].
    """
    value = get_parameter(motor, motor.PARAM_FEATURES, timeout=timeout)
    if value is None:
        return None
    out = {name: bool(value >> bit & 1) for bit, name in motor.FEATURES.items()}
    after = (value >> 16) & 3
    out['after calibration'] = motor.AFTER_CAL.get(after, str(after))
    return out


def loop_rate(motor):
    """The drive's control loop rate in Hz (6250 STEPFOC, 5000 Spectral Micro), from its identity; None if unknown."""
    if motor.product_id is None:
        read_identity(motor)
    return LOOP_RATES.get(motor.product_id)


def hold(motor, seconds=0.0, position=None, period=0.1):
    """Hold the shaft in position mode at `position` (default: where it is now), for `seconds`.

    The drive goes into position mode with zero speed and current feed-forward: the motor is energised and resists
    being turned, and it moves to `position` if that is not where it is. The frame is sent again every `period`
    seconds while this waits (so a watchdog stays fed); with seconds=0 it is sent once. The drive keeps holding
    afterwards until it gets another command (motor.Send_Idle() to let go). Returns the position held, or None if
    the drive did not answer the position request.
    """
    if position is None:
        position = read_position(motor)
        if position is None:
            return None
    end = time.perf_counter() + seconds
    while True:
        motor.Send_data_pack_1(Position=int(position), Speed=0, Current=0)
        left = end - time.perf_counter()
        if left <= 0:
            return position
        drain(motor, min(period, left))


def wait_for_state(motor, param, busy_states=None, timeout=120.0, poll=0.4, on_change=None, silent_limit=15.0):
    """Follow a run on the drive until its state parameter leaves `busy_states`; return that state.

    For the encoder table (param motor.PARAM_ENCAL, 52), the anti-cogging identification or a combined run
    (motor.PARAM_COGCAL, 43) and the capture (motor.PARAM_CAPTURE_STATE, 31) `busy_states` has a default
    (BUSY_STATES); map the result with motor.ENCAL_STATES, motor.COGCAL_STATES or motor.CAPTURE_STATES. The state
    is asked every `poll` seconds; on_change(seconds, state) is called whenever it changes.

    While the drive saves to its EEPROM it may not answer for a moment: that is waited out for up to `silent_limit`
    seconds. When `timeout` (or that silence) runs out, the last state seen is returned (still busy, or None if
    the drive never answered).
    """
    if busy_states is None:
        if param not in BUSY_STATES:
            raise ValueError(f"no default busy states for parameter {param}: pass busy_states")
        busy_states = BUSY_STATES[param]
    t0 = time.perf_counter()
    last = state = None
    silent_since = None
    while time.perf_counter() - t0 < timeout:
        drain(motor, poll)
        state = get_parameter(motor, param)
        now = time.perf_counter()
        if state is None:
            state = last
            silent_since = silent_since or now
            if now - silent_since > silent_limit:
                break
            continue
        silent_since = None
        if state != last:
            if on_change is not None:
                on_change(now - t0, state)
            last = state
        if state not in busy_states:
            break
    return state


# --- buffers and the capture --------------------------------------------------------------------------------------

def read_buffer(motor, kind, first=0, count=None, chunk=64, retries=3):
    """Entries of a buffer on the drive (firmware 109): {index: value}, oldest first.

    kind: motor.BUFFER_CAPTURE (value: (A, B, C)), BUFFER_HISTOGRAM ((count, sum)), BUFFER_COGGING_HARMONICS
    ((harmonic, a mA, b mA)), BUFFER_COGGING_TABLE (mA) or BUFFER_ENCODER_TABLE (1/16 count). The drive sends one
    frame per entry (the tables: one per three) as fast as the bus takes them; asking in chunks of `chunk` frames
    keeps a slow USB adapter from losing frames, and entries still missing are asked for again, up to `retries`
    times. An entry that never arrived is missing from the result.
    """
    store = {motor.BUFFER_CAPTURE: motor.capture, motor.BUFFER_HISTOGRAM: motor.histogram,
             motor.BUFFER_COGGING_HARMONICS: motor.cogging_harmonics, motor.BUFFER_COGGING_TABLE: motor.cogging_table,
             motor.BUFFER_ENCODER_TABLE: motor.encoder_table}[kind]
    if count is None:
        count = motor.BUFFER_LENGTHS[kind] - first
    comm = motor.communication
    flush(motor)
    store.clear()
    wanted = range(first, first + count)
    step = 3 if kind in (motor.BUFFER_COGGING_TABLE, motor.BUFFER_ENCODER_TABLE) else 1
    for _ in range(retries + 1):
        missing = [i for i in wanted if i not in store]
        if not missing:
            break
        i = 0
        while i < len(missing):
            start = end = missing[i]
            while i < len(missing) and missing[i] - start < chunk * step:
                end = missing[i]
                i += 1
            n = end - start + 1
            motor.Send_Buffer_Read(kind, start, n)
            deadline = time.perf_counter() + 0.05 + 0.002 * n
            while time.perf_counter() < deadline and any(j not in store for j in range(start, end + 1)):
                message, unpacked = comm.receive_can_messages(timeout=0.02)
                if message is not None and unpacked.node_id == motor.node_id:
                    motor.UnpackData(message, unpacked)
    return {i: store[i] for i in wanted if i in store}


class CaptureResult(NamedTuple):
    """A capture read back from the drive."""
    signals: tuple                  # the three signal names (channels A, B, C), as in motor.SIGNALS
    channels: tuple                 # three lists of values (int16, the signal's units), oldest first
    trigger_index: int              # the sample the trigger fired at, -1 if it did not
    sample_period: Optional[float]  # seconds between samples (divider / loop rate); None if the product is unknown


def unwrap16(values):
    """A 16-bit signal that wraps (the 'position' signal: the position's low 16 bits) as a continuous one."""
    out, offset, prev = [], 0, None
    for v in values:
        if prev is not None:
            d = v - prev
            if d > 32767:
                offset -= 65536
            elif d < -32768:
                offset += 65536
        out.append(v + offset)
        prev = v
    return out


def _signal_name(motor, s):
    if isinstance(s, str):
        return s
    names = {v: k for k, v in motor.SIGNALS.items()}
    return names.get(s, str(s))


def start_capture(motor, signals=('velocity', 'iq', 'electrical_angle'), divider=1, trigger='now', level=0,
                  pretrigger=0):
    """Set up the drive's three-channel capture and arm it (firmware 109).

    signals: up to three names of motor.SIGNALS (or numbers); channel A is also the trigger channel. divider:
    control loops per sample (1 = every loop: 1024 samples last 0.16 s on a STEPFOC, 0.2 s on a Spectral Micro).
    trigger: 'now', 'error' (when an error rises; the watchdog's too), 'rising' or 'falling' (channel A through
    `level`, in its units). pretrigger: samples kept from before the trigger.
    """
    motor.Send_Capture_Setup(*_three(signals), divider=divider, pretrigger=pretrigger, level=level)
    motor.Send_Capture_Trigger(trigger)


def _three(signals):
    signals = list(signals) + ['none'] * (3 - len(signals))
    if len(signals) != 3:
        raise ValueError('a capture has three channels')
    return signals


def wait_capture(motor, timeout=5.0, while_waiting=None, period=0.01):
    """Wait until the armed capture is done; True if it is, False after `timeout` seconds.

    while_waiting(seconds), if given, is called about every `period` seconds while this waits, for instance to
    send the motion commands whose response is being captured. The drive's answers to them are unpacked.
    """
    t0 = time.perf_counter()
    next_poll = t0
    while True:
        now = time.perf_counter()
        if while_waiting is not None:
            while_waiting(now - t0)
        if now >= next_poll:
            if get_parameter(motor, motor.PARAM_CAPTURE_STATE, timeout=0.1, retries=0) == 3:
                return True
            next_poll = time.perf_counter() + 0.05
        if time.perf_counter() - t0 >= timeout:
            return False
        drain(motor, period)


def read_capture(motor, signals=None, max_missing=0.1):
    """Read the finished capture back: a CaptureResult, or None if nothing was recorded (or too much was lost).

    signals: the names to label the channels with; default: asked from the drive. A sample lost on the way (after
    the retries of read_buffer) is filled with the one before it; more than `max_missing` of them lost gives None.
    """
    count = get_parameter(motor, motor.PARAM_CAPTURE_COUNT)
    if not count:
        return None
    trigger = get_parameter(motor, motor.PARAM_CAPTURE_TRIGGER_INDEX)
    if signals is None:
        signals = [get_parameter(motor, n) for n in (motor.PARAM_CAPTURE_SOURCE_A, motor.PARAM_CAPTURE_SOURCE_B,
                                                     motor.PARAM_CAPTURE_SOURCE_C)]
    signals = tuple(_signal_name(motor, s) for s in signals)
    divider = get_parameter(motor, motor.PARAM_CAPTURE_DIVIDER)
    rate = loop_rate(motor)
    data = read_buffer(motor, motor.BUFFER_CAPTURE, 0, count)
    rows = [data.get(i) for i in range(count)]
    missing = sum(r is None for r in rows)
    if missing == count or missing > max_missing * count:
        return None
    if missing:
        last = next(r for r in rows if r is not None)
        for i, r in enumerate(rows):
            if r is None:
                rows[i] = last
            last = rows[i]
    channels = tuple([r[k] for r in rows] for k in range(3))
    period = divider / rate if divider and rate else None
    return CaptureResult(signals, channels, trigger if trigger is not None else -1, period)


def capture(motor, signals=('velocity', 'iq', 'electrical_angle'), divider=1, trigger='now', level=0, pretrigger=0,
            timeout=5.0, while_waiting=None):
    """Record three signals on the drive and read them back: start_capture, wait_capture and read_capture in one.

    Returns a CaptureResult, or None if the capture was not done within `timeout` seconds; it is then left armed:
    motor.Send_Capture_Trigger('stop') ends it, read_capture(motor) reads what it holds. See start_capture for the
    arguments and wait_capture for while_waiting.
    """
    signals = _three(signals)
    start_capture(motor, signals, divider, trigger, level, pretrigger)
    if not wait_capture(motor, timeout, while_waiting):
        return None
    return read_capture(motor, signals)


# --- sound --------------------------------------------------------------------------------------------------------
# The drive plays sound only while it holds or moves (position, velocity, current or PD mode); idle or an error stops
# it. With hold=True the helpers put each drive in position mode at the position it has and send that again every
# 0.2 s while they play (the motor is energised and resists a hand, but does not turn). With hold=False the drive
# must already be in such a mode, and your program keeps it there (from while_playing, for instance).

def _as_list(motors):
    return [motors] if isinstance(motors, SpectralCAN) else list(motors)


def _send_events(motor, events):
    for channel, freq, current, wait in events:
        motor.Send_Sound_Event(freq, current, wait, channel=channel)


def play_events(motors, events, hold=True, timeout=None, while_playing=None, period=0.04):
    """Play queued sound events on one drive or several at once, keeping each drive's queue filled.

    events: (channel 0-3, frequency Hz or 0 for silence, loudness mA, then wait ms) tuples, as Spectral_BLDC.music
    makes them (melody_events, lines_events). For one drive a list; for several a dict {node_id: list}, or a list
    of lists in the order of `motors`. A drive's queue holds 31 events: the first 31 are loaded, all drives are
    started together, and every `period` seconds parameter 80 says how many are still queued and the queue is
    topped up (and restarted if it ran dry before the refill).

    hold: see the note above. while_playing(seconds), if given, is called every `period` seconds. Returns True
    when every drive has played to the end; False after `timeout` seconds (default: the longest event list's
    length plus 3 s), when a drive stops answering for 2 s, or when a position could not be read for the hold.
    The drives keep holding afterwards.
    """
    motors = _as_list(motors)
    if isinstance(events, dict):
        lists = [list(events.get(m.node_id, [])) for m in motors]
    elif events and isinstance(events[0][0], (list, tuple)):
        lists = [list(e) for e in events]
    else:
        lists = [list(events)] + [[] for _ in motors[1:]]
    if len(lists) != len(motors):
        raise ValueError('one event list per motor')
    pending = {m.node_id: lst for m, lst in zip(motors, lists)}
    if timeout is None:
        timeout = max((sum(ev[3] for ev in lst) for lst in lists), default=0) / 1000 + 3.0
    positions = {}
    if hold:
        for m in motors:
            positions[m.node_id] = read_position(m, others=motors)
            if positions[m.node_id] is None:
                return False

    def hold_all():
        for m in motors:
            m.Send_data_pack_1(Position=int(positions[m.node_id]), Speed=0, Current=0)

    if hold:
        hold_all()
    free_slots = SpectralCAN.SOUND_QUEUE_FREE
    for m in motors:
        m.Send_Sound_Clear()
        _send_events(m, pending[m.node_id][:free_slots])
        pending[m.node_id] = pending[m.node_id][free_slots:]
    for m in motors:
        m.Send_Sound_Play()
    t0 = next_tick = last_hold = time.perf_counter()
    silent_since = {}
    while True:
        next_tick += period
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        now = time.perf_counter()
        if now - t0 > timeout:
            return False
        if while_playing is not None:
            while_playing(now - t0)
        if hold and now - last_hold > _HOLD_PERIOD:
            hold_all()
            last_hold = now
        done = True
        for m in motors:
            s = get_parameter(m, m.PARAM_SOUND_STATE, timeout=0.2, retries=0, others=motors)
            if s is None:
                done = False
                silent_since.setdefault(m.node_id, now)
                if now - silent_since[m.node_id] > 2.0:
                    return False
                continue
            silent_since.pop(m.node_id, None)
            queued, running = s & 0xFF, (s >> 25) & 1
            if pending[m.node_id]:
                done = False
                free = free_slots - queued
                if free > 0:
                    _send_events(m, pending[m.node_id][:free])
                    pending[m.node_id] = pending[m.node_id][free:]
                if not running:
                    m.Send_Sound_Play()       # it ran dry before the refill: go on
            elif running or queued:
                done = False
        if done:
            return True


def _sample_state(motor):
    """Parameter 80 as (buffered samples, playing, ran dry), or None."""
    s = get_parameter(motor, motor.PARAM_SOUND_STATE, timeout=0.1, retries=0)
    if s is None:
        return None
    return (s >> 8) & 0x7FF, bool((s >> 26) & 1), bool((s >> 27) & 1)


def stream_samples(motor, samples, rate, volume=1000, hold=True, while_playing=None, on_progress=None, lead=600,
                   prefill=800):
    """Stream sound samples (speech, music) to a drive while it plays them; return how often its buffer ran dry.

    samples: int8 values, as bytes (Spectral_BLDC.music.wav_samples makes them from a WAV file) or ints -128..127.
    rate: samples per second, up to the drive's loop rate (6250 STEPFOC, 5000 Spectral Micro; half of it is a good
    choice). volume: the voltage of full scale (+-127), mV; the drive limits it to parameter 81 (default 2000 mV).

    The drive's buffer holds 1023 samples. `prefill` are sent first, then playback starts, and the rest follow about
    `lead` samples ahead of where the drive plays. Every 0.5 s, and at once after this program was held up for
    more than 0.1 s, parameter 80 says how many are buffered: the estimate of where the drive plays is corrected
    from it, and if the buffer ran dry it is filled again and playback restarted. on_progress(seconds_sent,
    buffered, ran_dry) is called at each of those checks. hold and while_playing: as in play_events. Returns after
    the drive has played the last sample (or after the buffer's length plus 0.5 s); None if hold=True and the
    position could not be read. The drive keeps holding afterwards.
    """
    data = bytes(samples) if isinstance(samples, (bytes, bytearray)) else bytes(int(s) & 0xFF for s in samples)
    if not data:
        return 0
    position = None
    if hold:
        position = read_position(motor)
        if position is None:
            return None
        motor.Send_data_pack_1(Position=int(position), Speed=0, Current=0)
    prefill = min(prefill, SpectralCAN.SOUND_BUFFER)
    motor.Send_Samples_Stop()                      # an empty buffer to start from
    sent = min(prefill, len(data))
    motor.Send_Samples(data[:sent])
    motor.Send_Samples_Play(rate, volume)
    t_start = t_base = next_tick = prev = time.perf_counter()
    last_check = last_hold = t_start
    underruns = 0

    def keep_holding(now):
        nonlocal last_hold
        if hold and now - last_hold > _HOLD_PERIOD:
            motor.Send_data_pack_1(Position=int(position), Speed=0, Current=0)
            last_hold = now
        if while_playing is not None:
            while_playing(now - t_start)

    while sent < len(data):
        next_tick = max(next_tick + 0.005, time.perf_counter() - 0.005)   # no catching up after a stall
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        now = time.perf_counter()
        if now - last_check > 0.5 or now - prev > 0.1:
            last_check = now
            state = _sample_state(motor)
            if state is not None:
                buffered, playing, dry = state
                if dry and not playing:            # it ran dry: fill it again, then go on
                    underruns += 1
                    n = min(len(data) - sent, max(0, prefill - buffered))
                    motor.Send_Samples(data[sent:sent + n])
                    sent += n
                    buffered += n
                    motor.Send_Samples_Play(rate, volume)
                t_base = time.perf_counter() - (sent - buffered) / rate    # where the drive really is
                if on_progress is not None:
                    on_progress(sent / rate, buffered, dry)
            now = time.perf_counter()
        else:
            drain(motor, 0.001)
        prev = now
        want = min(len(data), int((now - t_base) * rate) + lead)
        n = want - sent
        if want < len(data):
            n -= n % 8                             # whole frames until the last one
        if n > 0:
            motor.Send_Samples(data[sent:sent + n])
            sent += n
        keep_holding(now)
    end = time.perf_counter() + (SpectralCAN.SOUND_BUFFER / rate) + 0.5
    next_check = time.perf_counter() + 0.1
    while True:                                    # let the buffer play out
        now = time.perf_counter()
        if now >= end:
            break
        if now >= next_check:
            state = _sample_state(motor)
            if state is not None and not state[1]:
                break
            next_check = time.perf_counter() + 0.1
        keep_holding(now)
        drain(motor, max(0.0, min(0.02, end - now)))
    return underruns
