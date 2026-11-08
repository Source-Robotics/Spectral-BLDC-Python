"""Music for the drives' sound (firmware 2026-11 batch 22): note names, ringtone (RTTTL) melodies, MIDI files and
the queue events the drive plays. Pure Python, no I/O: SpectralCAN sends the events (Send_Sound_Event,
Send_Sound_Play) and the samples (Send_Samples, Send_Samples_Play).

A drive plays up to 4 notes at once (channels 0-3); its queue holds 31 events. An event sets one channel's pitch and
loudness (frequency 0 silences it), then waits; a wait of 0 lets the next event start at the same moment, so a chord
is three events with wait 0 and one with the chord's length.
"""

import struct

NOTE_OFFSETS = {'c': 0, 'd': 2, 'e': 4, 'f': 5, 'g': 7, 'a': 9, 'b': 11, 'h': 11}


def midi_hz(note):
    """Frequency of a MIDI note number (69 = A4 = 440 Hz)."""
    return 440.0 * 2 ** ((note - 69) / 12)


def note_number(name):
    """MIDI number of a note name such as 'A4', 'C#5', 'Eb3'."""
    s = name.strip().lower()
    n = NOTE_OFFSETS[s[0]]
    i = 1
    while i < len(s) and s[i] in '#b':
        n += 1 if s[i] == '#' else -1
        i += 1
    octave = int(s[i:]) if i < len(s) else 4
    return 12 * (octave + 1) + n


def note_hz(name):
    """Frequency of a note name such as 'A4' (440 Hz)."""
    return midi_hz(note_number(name))


def parse_rtttl(text):
    """A ringtone in RTTTL ('name:d=4,o=5,b=100:8e6,8d6,4f#5,p,...') as (name, [(freq_hz or 0 for a rest, ms)])."""
    name, defaults, notes = text.split(':', 2)
    d, o, b = 4, 6, 63
    for item in defaults.split(','):
        if '=' in item:
            k, v = item.strip().split('=')
            if k == 'd':
                d = int(v)
            elif k == 'o':
                o = int(v)
            elif k == 'b':
                b = int(v)
    whole_ms = 4 * 60000 / b
    out = []
    for raw in notes.split(','):
        s = raw.strip().lower()
        if not s:
            continue
        i = 0
        while i < len(s) and s[i].isdigit():
            i += 1
        dur = int(s[:i]) if i else d
        note = s[i]
        i += 1
        sharp = i < len(s) and s[i] == '#'
        if sharp:
            i += 1
        dotted = False
        if i < len(s) and s[i] == '.':
            dotted = True
            i += 1
        octave = o
        if i < len(s) and s[i].isdigit():
            octave = int(s[i])
            i += 1
        if i < len(s) and s[i] == '.':
            dotted = True
        ms = whole_ms / dur * (1.5 if dotted else 1.0)
        if note == 'p':
            out.append((0, round(ms)))
        else:
            n = 12 * (octave + 1) + NOTE_OFFSETS[note] + (1 if sharp else 0)
            out.append((round(midi_hz(n)), round(ms)))
    return name, out


def melody_events(melody, current_mA=200, channel=0, gap_ms=15):
    """[(freq_hz, ms)] as queue events: each note sounds for ms - gap_ms, then a short silence separates it from the
    next (so repeated notes are heard as two)."""
    events = []
    for freq, ms in melody:
        if freq and ms > gap_ms:
            events.append((channel, int(freq), int(current_mA), int(ms - gap_ms)))
            events.append((channel, 0, 0, int(gap_ms)))
        else:
            events.append((channel, 0, 0, int(ms)))
    events.append((channel, 0, 0, 0))
    return events


def _vlq(data, i):
    v = 0
    while True:
        b = data[i]
        i += 1
        v = (v << 7) | (b & 0x7F)
        if not b & 0x80:
            return v, i


def read_midi(path):
    """A standard MIDI file (format 0 or 1) as a list of notes (start_s, end_s, midi_note, velocity, track),
    with the tempo changes applied. Drums (channel 10) are left out."""
    data = open(path, 'rb').read()
    if data[:4] != b'MThd':
        raise ValueError('not a MIDI file')
    hlen = struct.unpack('>I', data[4:8])[0]
    _fmt, ntrks, division = struct.unpack('>HHH', data[8:14])
    if division & 0x8000:
        raise ValueError('SMPTE time division is not supported')
    i = 8 + hlen
    tracks, tempos = [], [(0, 500000)]
    for t in range(ntrks):
        if data[i:i + 4] != b'MTrk':
            break
        tlen = struct.unpack('>I', data[i + 4:i + 8])[0]
        j, end = i + 8, i + 8 + tlen
        tick, status, events = 0, 0, []
        while j < end:
            delta, j = _vlq(data, j)
            tick += delta
            b = data[j]
            if b == 0xFF:
                kind = data[j + 1]
                length, k = _vlq(data, j + 2)
                if kind == 0x51 and length == 3:
                    tempos.append((tick, int.from_bytes(data[k:k + 3], 'big')))
                j = k + length
                continue
            if b in (0xF0, 0xF7):
                length, k = _vlq(data, j + 1)
                j = k + length
                continue
            if b & 0x80:
                status = b
                j += 1
            kind, chan = status & 0xF0, status & 0x0F
            if kind in (0x80, 0x90, 0xA0, 0xB0, 0xE0):
                a1, a2 = data[j], data[j + 1]
                j += 2
                if chan != 9 and kind == 0x90 and a2 > 0:
                    events.append((tick, 'on', a1, a2))
                elif chan != 9 and (kind == 0x80 or (kind == 0x90 and a2 == 0)):
                    events.append((tick, 'off', a1, 0))
            else:
                j += 1                               # program change, channel pressure
        tracks.append(events)
        i = end
    tempos.sort()

    def seconds(tick):
        s, last_tick, us = 0.0, 0, 500000
        for t_tick, t_us in tempos:
            if t_tick >= tick:
                break
            s += (t_tick - last_tick) * us / division / 1e6
            last_tick, us = t_tick, t_us
        return s + (tick - last_tick) * us / division / 1e6
    notes = []
    for tr, events in enumerate(tracks):
        held = {}
        for tick, what, note, vel in events:
            if what == 'on':
                held.setdefault(note, []).append((tick, vel))
            elif held.get(note):
                start, vel0 = held[note].pop(0)
                notes.append((seconds(start), seconds(tick), note, vel0, tr))
    notes.sort()
    return notes


def split_voices(notes, voices):
    """Notes (start_s, end_s, note, ...) into at most `voices` monophonic lines (lists of (start_s, end_s, note));
    a note that finds no free line takes the line that frees first; notes that still clash are dropped."""
    lines = [[] for _ in range(voices)]
    for n in sorted(notes, key=lambda x: (x[0], -x[2])):
        start, end, note = n[0], n[1], n[2]
        free = [k for k in range(voices) if not lines[k] or lines[k][-1][1] <= start + 1e-4]
        if free:
            lines[free[0]].append((start, end, note))
        else:
            k = min(range(voices), key=lambda v: lines[v][-1][1])
            last = lines[k][-1]
            if last[0] < start:
                lines[k][-1] = (last[0], start, last[2])   # cut the old note short
                lines[k].append((start, end, note))
    return lines


def lines_events(lines, current_mA=200, gap_ms=10):
    """Monophonic lines (one per channel, as split_voices makes) as one queue's events, in time order."""
    changes = []
    for ch, line in enumerate(lines):
        for start, end, note in line:
            changes.append((start, ch, round(midi_hz(note))))
            changes.append((max(start, end - gap_ms / 1000), ch, 0))
    changes.sort(key=lambda c: (c[0], c[2] != 0))
    events = []
    for k, (t, ch, freq) in enumerate(changes):
        nxt = changes[k + 1][0] if k + 1 < len(changes) else t
        events.append((ch, freq, int(current_mA) if freq else 0, max(0, int(round((nxt - t) * 1000)))))
    return events


def wav_samples(path, rate, coil_tau_s=0.0, clip_percentile=None):
    """A WAV file (8 or 16 bit, any channels) as int8 samples at `rate` Hz, mono, scaled to the full range.

    coil_tau_s: the coil's L / R (s). Above 1 / (2 pi tau) the coil passes less current the higher the pitch, which
    takes the consonants out of speech; this boosts the highs to match (y = x + tau dx/dt), so the current, and the
    sound, follow the recording. clip_percentile (e.g. 0.95): scale so that this share of the samples fits and
    soft-clip the rest; speech averages about a tenth of its peak, and this makes it about four times louder while
    staying intelligible. None: scale to the peak (music).
    """
    import wave
    with wave.open(path, 'rb') as w:
        nch, width, fr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width == 1:
        vals = [b - 128 for b in raw]
    elif width == 2:
        vals = list(struct.unpack('<%dh' % (len(raw) // 2), raw))
    else:
        raise ValueError('8 or 16-bit WAV only')
    mono = [sum(vals[i:i + nch]) / nch for i in range(0, len(vals) - nch + 1, nch)]
    out, step, pos = [], fr / rate, 0.0
    while pos < len(mono) - 1:
        k = int(pos)
        f = pos - k
        out.append(mono[k] * (1 - f) + mono[k + 1] * f)
        pos += step
    if coil_tau_s > 0 and out:
        k = coil_tau_s * rate
        out = [out[0]] + [out[i] + k * (out[i] - out[i - 1]) for i in range(1, len(out))]
    if clip_percentile:
        import math
        mags = sorted(abs(v) for v in out)
        ref = mags[int(clip_percentile * (len(mags) - 1))] if mags else 1
        ref = ref or 1
        out = [math.tanh(1.5 * v / ref) / math.tanh(1.5) for v in out]
        out = [max(-1.0, min(1.0, v)) for v in out]
    else:
        peak = max((abs(v) for v in out), default=1) or 1
        out = [v / peak for v in out]
    return bytes(int(round(v * 127)) & 0xFF for v in out)
