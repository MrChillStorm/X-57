#!/usr/bin/env python3
"""The X-57's sound: PyWopWop's bank of propeller and motor loops, wired into the two aircraft.

    python3 x57sound.py [bank]       # default ~/git/PyWopWop/out/x57; writes ../X-57/Sounds

The bank's section cross-fades each group of propellers over airspeed and rpm (the cruise propellers also over Ct). Four
things are done to it here.

Sources. SimGear has 128 sources for everything, and a looped sound holds one while its condition is true, however quiet.
So a loop's condition must be false whenever it can't be heard. The bank's conditions are its fades, with strict
limits, so a propeller loop stops where its volume reaches nil, and each view's loops only run in that view. But the governor
holds rpm on a grid point, and a spooling rpm only approaches it, so as the bank stands the neighbouring loops stay true
at next to nothing for the whole flight: 40 % of the sources held were under -60 dB. So each fade here has a flat top: within
FLAT of a grid point only that loop plays, and its neighbours are off. Adjacent ramps move together, so it stays equal
power. The motors' and
spinners' layers were gated at rpm > 0, but a spooling rpm only tends to 0, and they would hold a source for good once
the propellers stopped: here their gates rise to where they fall to -60 dB. Mod III's file leaves the high-lift groups out
altogether. A stopped propeller reads 0 rpm (genx57.py), so nothing runs on a parked aircraft. soundcheck.py tests all
of this on the XML itself.

Altitude. The loops are sea level's, and PropCondition's airspeed is true, so the fades run on true airspeed: at any height a
loop then plays at its own advance ratio, with rpm and Ct its own. Thrust goes as the density, but the level falls by less,
because the tip Mach number rises in the colder air. Volume carries sigma to a power, measured in PyWopWop by soundalt.py:
a good fit for the cruise propellers and outside, within about 0.5 dB to 15,000 ft; in the cockpit the high-lift
propellers scatter, 1.5 dB at 8,000 ft. The motors' tones aren't aerodynamic, and have none.

Level. The cockpit's and the outside's loops have different gains (manifest gain_db), so the outside's are brought to the
cockpit's scale by their volume. A full-scale sample at volume 1 is then 112.5 dB re 20 uPa, and every other sound plays at
the level it makes on that scale: PyWopWop's wind (mkwind.py), gear in the airflow (mkgear.py), tyres rolling (mktyres.py) and touchdowns (mktouchdown.py), and the airframe's own
sounds, which LEVELS sets with what each level rests on.

Place. The bank's positions are from the moment reference; SimGear's are from the model's origin, the VRP at station 0.

Doors. In the cockpit an open door lets the outside in: the outside's loops fade in on top of the cockpit's, by how far a
door is open, the rear door at half the main door's (about 1.8 m behind the pilot, across the cabin: an estimate). Only its
motors, and its propellers' loops of 0 kt, which fade out by 40 to 59 kt: a door opens only on the ground. Not the spinners,
30 dB under the propellers there; this keeps the sweep's worst case at the limit. The cockpit's own loops stay, as the walls
still carry them. Recorded at idle (FlightGear's own mix), the main door open lifts 3 to 8 kHz by 8 dB and leaves the motors'
low tone, so it sounds brighter, not louder. The outside's loops are PyWopWop's levels at 30 m; beside the door they would be
about 15 dB more, which needs louder copies of the loops (55 MB), so this is the outside as it is, crossfaded in by the door.
"""
import json, math, os, re, shutil, subprocess, sys

import numpy as np
import scipy.io.wavfile as wav

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'X-57', 'Sounds')
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
IN = 0.0254
RHO0 = 0.00237689
TAS = '/fdm/jsbsim/velocities/vtrue-kts'
SIGMA = '<div><property>/environment/density-slugft3</property><value>%.8g</value></div>' % RHO0
ENTRY = re.compile(r'\t\t<(prop|motor|hub)>\n(.*?)\t\t</\1>\n', re.S)
# sigma's power in each loop's level at the same true airspeed, rpm and Ct [soundalt.py]
ALT_POWER = dict(cruise=dict(cockpit=0.65, side=0.4), hl=dict(cockpit=0.8, side=0.85))
HEARD = 1e-3            # -60 dB: under this a layer's gate shuts
FLAT = (('vtrue-kts', 1.0), ('/rpm', 20.0), ('/ct', 0.002))     # half the width of a fade's flat top, by property


def full_scale_db(gain_db):
    return 20 * math.log10(1 / 2e-5) - gain_db


def entries(text):
    """(tag, body, group, view) of each loop"""
    for m in ENTRY.finditer(text):
        body = m.group(2)
        parts = re.search(r'<name>([^<]+)</name>', body).group(1).split('-')
        yield m.group(1), body, parts[-2], parts[-1]


def gate(tag, body):
    """a motor's or spinner's gate at the rpm where it falls to HEARD: the spinners' go as the cube of the rpm, the motors'
    (at least 0.15 of their level) as the rpm up to a fifth of their reference"""
    ref = 1.0 / float(re.search(r'<factor>([^<]+)</factor>', body).group(1))
    floor = ref * HEARD ** (1 / 3) if tag == 'hub' else ref * 0.2 * HEARD / 0.15
    body, n = re.subn(r'<value>0</value></greater-than>', '<value>%.4g</value></greater-than>' % floor, body)
    assert n == 1, body
    return body


def flat(prop):
    return next(d for k, d in FLAT if prop.endswith(k))


def flat_tops(body):
    """Every ramp of a propeller loop, from nil at z to full at o, moved in by FLAT at both ends, and every gate with it"""
    def ramp(m):
        prop, a, b = m.group(1), float(m.group(2)), float(m.group(3))
        z, o = round(-b / a, 3), round((1 - b) / a, 3)          # the grid points, without the bank's rounding
        d = flat(prop) * (1 if o > z else -1)
        z, o = z + d, o - d
        a2 = 1.0 / (o - z)
        return ('<clip><sum><product><property>%s</property><value>%.10g</value></product><value>%.10g</value></sum>'
                '<clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (prop, a2, -a2 * z))

    body, n = re.subn(r'<clip><sum><product><property>([^<]+)</property><value>([^<]+)</value></product><value>([^<]+)</value></sum>'
                      r'<clipMin>0</clipMin><clipMax>1</clipMax></clip>', ramp, body)
    assert n >= 2, body

    def lo(m):
        return '<greater-than><property>%s</property><value>%.6g</value></greater-than>' % (m.group(1), float(m.group(2)) + flat(m.group(1)))

    def hi(m):
        return '<less-than><property>%s</property><value>%.6g</value></less-than>' % (m.group(1), float(m.group(2)) - flat(m.group(1)))

    body = re.sub(r'<greater-than><property>([^<]+)</property><value>([^<]+)</value></greater-than>', lo, body)
    return re.sub(r'<less-than><property>([^<]+)</property><value>([^<]+)</value></less-than>', hi, body)


def convert(tag, body, group, view, scale):
    """one loop: true airspeed, the scale and the density in its volume, its gate, its place from the model's origin"""
    mrs = GEO['reference']['mrs_in']
    body = body.replace('/velocities/airspeed-kt', TAS)
    body = flat_tops(body) if tag == 'prop' else gate(tag, body)
    power = 0.0 if tag == 'motor' else ALT_POWER['cruise' if group.startswith('cruise') else 'hl'][view]

    # through a door: the outside's motors, and its propellers' loops of 0 kt, which fade out by 40 to 59 kt; a door opens only on the
    # ground. Not the spinners: outside they are 30 dB under the propellers on the ground. This keeps the sweep's worst case at the limit
    through = view != 'cockpit' and (tag == 'motor' or (tag == 'prop' and re.search(r'<name>pww-0kt_', body) is not None))

    def vol(m):
        k = '' if scale[view] == 1.0 else '<value>%.6g</value>' % scale[view]
        d = '<pow>%s<value>%g</value></pow>' % (SIGMA, power) if power else ''
        door = DOOR_FADE if through else ''
        return '<volume><expression><product>%s%s%s%s</product></expression></volume>' % (d, k, m.group(1), door)

    if through:
        body, n = re.subn(re.escape(NOT_INSIDE), DOOR_OPEN, body)
        assert n == 1, body

    body, n = re.subn(r'<volume><expression>(.*?)</expression></volume>', vol, body, flags=re.S)
    assert n == 1, body

    def place(m):
        x, y, z = (float(q) for q in m.groups())
        return ('<position>\n\t\t\t\t<x>%.3f</x>\n\t\t\t\t<y>%.3f</y>\n\t\t\t\t<z>%.3f</z>\n\t\t\t</position>'
                % (x + mrs[0] * IN, y, z + mrs[2] * IN))

    body, n = re.subn(r'<position>\s*<x>([^<]+)</x>\s*<y>([^<]+)</y>\s*<z>([^<]+)</z>\s*</position>', place, body)
    assert n == 1, body
    return '\t\t<%s>\n%s\t\t</%s>\n' % (tag, body, tag)


# The airframe's own sounds: the P2006T's samples (GPL v2) and FlightGear's gear lock, at the level each makes, on the bank's scale. No measurement of
# any of them exists on this aircraft, or in PyWopWop's documents, so every level below is an estimate, and LEVELS says from what.
# A file whose A-weighted rms (a click's: its loudest 125 ms) is L dB re its full scale plays at volume 10^((SPL - FULL - L)/20): the ASK 21's method.
# Each stays silent, so holds no source, unless it's happening. The gear in the airflow is computed (mkgear.py).
AIRFRAME = {'gear-p2006t.wav': 'gear.wav', 'flaps-p2006t.wav': 'Rotax912-Flaps.wav',
            'switch-rocker.wav': 'Rotax912-Click.wav',
            'door-open-p2006t.wav': 'Rotax912-Door_open.wav', 'door-shut-p2006t.wav': 'Rotax912-Door_close.wav'}
FG_ROOT = os.environ.get('FG_ROOT', os.path.expanduser('~/git/fgdata'))
LOCK = ('gear-lock.wav', 'Sounds/gear-lck.wav')
BUTTON = ('switch-push.wav', 'Sounds/click.wav')
WOW = '<or><property>gear/gear[0]/wow</property><property>gear/gear[1]/wow</property><property>gear/gear[2]/wow</property></or>'
STARTED = '<greater-than><property>sim/time/elapsed-sec</property><value>5</value></greater-than>'
INTERNAL = '<property>sim/current-view/internal</property>'
GEAR_POS = 'gear/gear[0]/position-norm'
KMH = 1 / 1.852                                             # kt per km/h
# The tyres rolling are PyWopWop's (its tyre.py; mktyres.py here): each tyre's rolling noise scaled from a measured truck tyre (Hoever 2014) by width,
# diameter, speed and asphalt texture, outside checked against one towed light aircraft on 5.00-5s, 60.5 dB(A) at 30 m against 60; in the cockpit through
# the structure, from a recorded landing in the P2006T's cabin, set against the wind bank. They replace the stock rumble, whose 72 dB(A) at 43 kt in the
# cockpit, from car-tyre data and a guessed cabin, sat well under the wind there: the recording has the roll near the wind's level
# The touchdowns are PyWopWop's too (mktouchdown.py): each wheel's squeal from the energy its spin-up loses, its share and spectrum from recorded landings,
# and a thump from its strut's force at its estimate, a firm one through a soft knee where it would pass full scale. Each plays once as the wheel's
# weight comes on, cross-faded over the slip (the ground speed less the rim's: a wheel still turning after a hop is quiet) and sink rate the flight
# model latches then (gear/unit[n]/touchdown-slip-kt, -sink-fps). They replace the stock contact sample, set from the ASK 21's estimate
# The aural alert: ISO 7731's 15 dB over the noise round it, the cockpit's 90 at 100 kt (wind and cruise propellers). The real one reaches a headset
# through the audio panel [ICD slide 40], whose setting isn't published; this cockpit's sound is what an open ear hears, so that is what it must clear
ALERT_DBA = 105.0
ROCKER_DBA, PUSH_DBA = 78.0, 72.0       # the loudest 125 ms of each: short sounds, so they peak well above this


MAIN = (172.0 * IN, 0.0, 0.7)
WING = (3.9, 0.0, 2.2)
# the doors' middles, from the Tecnam's door objects (interior frame x aft, y right, z up, less 4.93 m and 1.96 m to the VRP): the main door on the left
# beside the pilot, the rear door on the right over pack A. SimGear's y is to the left
MAIN_DOOR, REAR_DOOR = (2.67, 0.54, 1.63), (3.60, -0.56, 1.63)
# a door's sounds: doors.nas starts sim/model/x57/door/NAME at -1, then 1 as the door leaves its frame and 0 as it shuts, so neither plays at the start
DOORED = '<condition><equals><property>sim/model/x57/door/%s</property><value>%d</value></equals></condition>'
# the outside's loops in the cockpit through an open door (see Doors above): on while a door is open, faded by how far
NOT_INSIDE = '<not><property>sim/current-view/internal</property></not>'
DOOR_OPEN = ('<or>%s<greater-than><property>sim/model/door-left</property><value>0.01</value></greater-than>'
             '<greater-than><property>sim/model/door-right</property><value>0.01</value></greater-than></or>' % NOT_INSIDE)
REAR_WEIGHT = 0.5
DOOR_FADE = ('<sum><value>1</value><product><property>sim/current-view/internal</property><sum><value>-1</value><clip><sqrt><sum>'
             '<product><property>sim/model/door-left</property><property>sim/model/door-left</property></product>'
             '<product><value>%g</value><property>sim/model/door-right</property><property>sim/model/door-right</property></product>'
             '</sum></sqrt><clipMin>0</clipMin><clipMax>1</clipMax></clip></sum></product></sum>' % REAR_WEIGHT ** 2)
# Sounds that must say nothing about how things stood at the start, and SimGear's <internal>dt_stop</internal> can't do it: it reads a copy taken at load, always 0, so a sound
# multiplied by it is silent for good. So a click's property starts at -1, which neither of its two sounds plays on: a toggle takes it to 0, then 1. A lock's needs a gear that moved
TOGGLED = '<condition><equals><property>sim/model/x57/click/%s</property><value>%d</value></equals></condition>'
MOVED = '<condition><and>%s<property>sim/model/x57/gear-moved</property></and></condition>'
LEVELS = [
    # tag, name, the file, what plays it, dB(A) in the cockpit, dB(A) at 30 m (None: not heard outside), the amplitude's law (1 at the reference
    # state), a click, where it comes from, where the level comes from
    ('gear', 'gear', 'gear-p2006t.wav', '<mode>in-transit</mode><property>%s</property>' % GEAR_POS, 75.0, 30.0, None, False, MAIN,
     'the pump and jacks: the top of hydraulic power units\' published 58 to 75 dB(A) at 1 m; none is published for a small 28 V pack. Outside, less '
     '30 m\'s spreading and the walls\' 19 dB at 500 Hz, rounded up'),
    ('gear', 'gear-locked-down', LOCK[0], MOVED % ('<greater-than><property>%s</property><value>0.99</value></greater-than>' % GEAR_POS), 85.0, 55.0, None, True, MAIN, 'a leg locking down: the ASK 21\'s estimate for its airbrakes\' over-centre lock'),
    ('gear', 'gear-locked-up', LOCK[0], MOVED % ('<less-than><property>%s</property><value>0.01</value></less-than>' % GEAR_POS), 82.5, 52.5, None, True, MAIN, 'a leg reaching its up stop: as the down lock, less the stock sound file\'s 2.5 dB'),
    ('flap', 'flaps', 'flaps-p2006t.wav', '<mode>in-transit</mode><property>surface-positions/flap-pos-norm</property>', 65.0, 30.0, None, False, WING,
     'the flap motor: the middle of electric linear actuators\' published 55 to 70 dB(A), about 1 m off in the cabin\'s roof'),
    ('click', 'rocker-a', 'switch-rocker.wav', TOGGLED % ('rocker', 1), ROCKER_DBA, None, None, True, None,
     'a toggle or rocker switch flipped, about 0.6 m from the ear: estimate'),
    ('click', 'rocker-b', 'switch-rocker.wav', TOGGLED % ('rocker', 0), ROCKER_DBA, None, None, True, None,
     'the same, the other way'),
    ('click', 'push-a', BUTTON[0], TOGGLED % ('push', 1), PUSH_DBA, None, None, True, None,
     'a push button, breaker or knob: estimate, a lighter mechanism than a switch'),
    ('click', 'push-b', BUTTON[0], TOGGLED % ('push', 0), PUSH_DBA, None, None, True, None,
     'the same, on release'),
    ('door', 'door-main-open', 'door-open-p2006t.wav', DOORED % ('main', 1), 75.0, 40.0, None, True, MAIN_DOOR,
     'the main door unlatched and swung out, about 0.5 m from the ear: estimate, the shut\'s less 10 dB for a latch and its seal'),
    ('door', 'door-main-shut', 'door-shut-p2006t.wav', DOORED % ('main', 0), 85.0, 50.0, None, True, MAIN_DOOR,
     'the main door shut on its latch, about 0.5 m from the ear: estimate, a car door\'s firm close. Outside, 30 m\'s spreading from 0.5 m'),
    ('door', 'door-rear-open', 'door-open-p2006t.wav', DOORED % ('rear', 1), 65.0, 40.0, None, True, REAR_DOOR,
     'the rear door opening, about 1.8 m behind: the main door\'s less 10 dB, spreading with some of the cabin\'s echo'),
    ('door', 'door-rear-shut', 'door-shut-p2006t.wav', DOORED % ('rear', 0), 75.0, 50.0, None, True, REAR_DOOR,
     'the rear door shutting: the main door\'s less 10 dB'),
    ('alert', 'alert', 'x57-alert.wav', '<condition><property>sim/model/x57/alert-tone</property></condition>', ALERT_DBA, None, None, True, None,
     'the aural alert (see ALERT_DBA)'),
]


def a_weight_db(f):
    """IEC 61672 A-weighting, dB"""
    f2 = np.asarray(f, float) ** 2
    ra = 12194.0 ** 2 * f2 ** 2 / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return 20 * np.log10(np.maximum(ra, 1e-30)) + 2.0


def file_level(path, click=False):
    """a WAV's A-weighted level, dB re its full scale: its rms, or a click's loudest 125 ms"""
    fs, y = wav.read(path)
    if y.ndim > 1:
        y = y.mean(axis=1)
    y = (y.astype(np.float64) - (128.0 if y.dtype == np.uint8 else 0.0)) / (128.0 if y.dtype == np.uint8 else 32768.0)
    Y = np.fft.rfft(y - y.mean())
    ya = np.fft.irfft(Y * 10 ** (a_weight_db(np.maximum(np.fft.rfftfreq(len(y), 1 / fs), 1e-3)) / 20), len(y))
    if not click:
        return 10 * np.log10(np.mean(ya ** 2))
    n = max(1, int(0.125 * fs))
    return 10 * np.log10(np.max(np.convolve(ya ** 2, np.ones(n) / n, 'valid') if len(ya) > n else [np.mean(ya ** 2)]))


def view_volume(k_in, k_out, law):
    """the cockpit's level inside, the outside's outside, times the law"""
    view = ('<product>%s<value>%.6g</value></product>' % (INTERNAL, k_in) if k_out is None else
            '<sum><value>%.6g</value><product>%s<value>%.6g</value></product></sum>' % (k_out, INTERNAL, k_in - k_out))
    return '<volume><expression>%s</expression></volume>' % ('<product>%s%s</product>' % (view, law) if law else view)


def airframe(full):
    """LEVELS as sound entries, and a report"""
    out, rep = [], []
    for tag, name, fn, plays, l_in, l_out, law, click, at, note in LEVELS:
        lf = file_level(os.path.join(OUT, fn), click)
        k_in = 10 ** ((l_in - full - lf) / 20)
        k_out = None if l_out is None else 10 ** ((l_out - full - lf) / 20)
        if k_in > 1:
            sys.exit('%s would need a volume of %.2f: a louder file needed' % (name, k_in))
        dist = '<reference-dist>30</reference-dist><max-dist>1500</max-dist>' if l_out is not None else '<reference-dist>3</reference-dist><max-dist>10</max-dist>'
        place = '<position><x>%g</x><y>%g</y><z>%g</z></position>' % at if at else ''
        out.append('\t\t<%s>\n\t\t\t<name>%s</name><path>Sounds/%s</path>%s\n\t\t\t<!-- %s dB(A) in the cockpit%s: %s -->\n\t\t\t%s\n\t\t\t%s%s\n\t\t</%s>\n'
                   % (tag, name, fn, plays, '%g' % l_in, '' if l_out is None else ', %g at 30 m' % l_out, note.replace('--', '-'),
                      view_volume(k_in, k_out, law), place, dist, tag))
        rep.append('  %-18s %6.1f dB re its full scale: cockpit %5.1f dB(A), volume %.4f%s' % (
            name, lf, l_in, k_in, '' if l_out is None else '; 30 m %5.1f, volume %.4f' % (l_out, k_out)))
    return ''.join(out), rep


def wind(bank):
    """PyWopWop's cockpit wind, each loop gated to the speeds it's heard at: looped, it would hold a source all flight"""
    d = bank + '_wind'
    if not os.path.exists(os.path.join(d, 'pywopwop-wind.xml')):
        print('no wind at %s: python3 mkwind.py' % d)
        return '', []
    man = json.load(open(os.path.join(d, 'manifest.json')))
    v = [e['airspeed_kt'] for e in man['files']]
    out = re.findall(r'\t\t<wind>.*?</wind>\n', open(os.path.join(d, 'pywopwop-wind.xml')).read(), re.S)
    for i, e in enumerate(out):
        lo, hi = (0.75 * v[i - 1] if i else 10.0), (1.05 * v[i + 1] if i + 1 < len(v) else 1000.0)
        k = '<property>velocities/equivalent-kt</property>'
        c = ('<condition><and><property>sim/current-view/internal</property><greater-than>%s<value>%g</value></greater-than>'
             '<less-than>%s<value>%g</value></less-than></and></condition>' % (k, lo, k, hi))
        out[i] = re.sub(r'<condition>.*?</condition>', c, e, flags=re.S)
    return ''.join(out), [(os.path.join(d, x['file']), x['file']) for x in man['files']]


def gear_air(bank, scale):
    """PyWopWop's gear in the airflow, each view's loops by equal power over true airspeed and gated to the speeds they're heard at, times how far the
    gear is out. A Fink level goes as (rho c^2 M^3)^2, so the amplitude as the density at the same true airspeed"""
    d = bank + '_gear'
    if not os.path.exists(os.path.join(d, 'manifest.json')):
        print('no gear at %s: python3 mkgear.py' % d)
        return '', []
    man = json.load(open(os.path.join(d, 'manifest.json')))
    prop = '<property>%s</property>' % TAS
    out = []
    for view, k, seen in (('cockpit', 1.0, INTERNAL), ('side', scale['side'], '<not>%s</not>' % INTERNAL)):
        L = [e for e in man['files'] if e['view'] == view]
        v = [e['airspeed_kt'] for e in L]
        for i, e in enumerate(L):
            lo = v[i - 1] if i else 0.2 * v[0]
            hi = v[i + 1] if i + 1 < len(v) else None

            def ramp(a, b):
                s = 1.0 / (b - a)
                return ('<clip><sum><product>%s<value>%.8g</value></product><value>%.8g</value></sum><clipMin>0</clipMin><clipMax>1</clipMax></clip>'
                        % (prop, s, -a * s))
            w = ramp(lo, v[i]) if hi is None else '<product>%s%s</product>' % (ramp(lo, v[i]), ramp(hi, v[i]))
            cond = '%s<greater-than><property>%s</property><value>0.01</value></greater-than><greater-than>%s<value>%g</value></greater-than>' % (seen, GEAR_POS, prop, lo)
            if hi is not None:
                cond += '<less-than>%s<value>%g</value></less-than>' % (prop, hi)
            vol = '<product>%s<sqrt>%s</sqrt><property>%s</property><value>%.6g</value></product>' % (SIGMA, w, GEAR_POS, k)
            where = ('<reference-dist>30</reference-dist><max-dist>3000</max-dist>' if view == 'side' else '')
            out.append('\t\t<gearair>\n\t\t\t<name>pww-gear-%g-%s</name><mode>looped</mode><path>Sounds/pywopwop/%s</path>\n\t\t\t<condition><and>%s</and></condition>\n'
                       '\t\t\t<volume><expression>%s</expression></volume>\n\t\t\t<position><x>%g</x><y>0</y><z>0.7</z></position>%s\n\t\t</gearair>\n'
                       % (e['airspeed_kt'], view, e['file'], cond, vol, 172.0 * IN, where))
    return ''.join(out), [(os.path.join(d, e['file']), e['file']) for e in man['files']]


def tyres(bank, scale):
    """PyWopWop's tyres rolling: each wheel's loops by equal power over its own roll speed, gated on its weight and to the speeds each is heard at, fading
    as it lifts (the bank's own conditions and volumes). Here the outside's go to the cockpit's scale and each place from the model's origin"""
    d = bank + '_tyres'
    if not os.path.exists(os.path.join(d, 'pywopwop-tyres.xml')):
        print('no tyres at %s: python3 mktyres.py' % d)
        return '', []
    man = json.load(open(os.path.join(d, 'manifest.json')))
    mrs = GEO['reference']['mrs_in']
    out = []
    for e in re.findall(r'\t\t<tyre>.*?</tyre>\n', open(os.path.join(d, 'pywopwop-tyres.xml')).read(), re.S):
        if re.search(r'<name>[^<]*-side</name>', e):
            e, n = re.subn(r'<volume><expression>(.*?)</expression></volume>',
                           lambda m: '<volume><expression><product><value>%.6g</value>%s</product></expression></volume>' % (scale['side'], m.group(1)), e, flags=re.S)
            assert n == 1, e
        e, n = re.subn(r'<position>\s*<x>([^<]+)</x>\s*<y>([^<]+)</y>\s*<z>([^<]+)</z>\s*</position>',
                       lambda m: '<position>\n\t\t\t\t<x>%.3f</x>\n\t\t\t\t<y>%.3f</y>\n\t\t\t\t<z>%.3f</z>\n\t\t\t</position>'
                       % (float(m.group(1)) + mrs[0] * IN, float(m.group(2)), float(m.group(3)) + mrs[2] * IN), e)
        assert n == 1, e
        out.append(e)
    return ''.join(out), [(os.path.join(d, x['file']), x['file']) for x in man['files']]


def touchdown(bank, scale):
    """PyWopWop's touchdowns: each wheel's squeal and thump, once as its weight comes on, cross-faded over the slip and sink rate the flight model
    latched then (the bank's own conditions and volumes: up to four squeals and two thumps, one waveform under different envelopes). Here the
    outside's go to the cockpit's scale and each place from the model's origin"""
    d = bank + '_touchdown'
    if not os.path.exists(os.path.join(d, 'pywopwop-touchdown.xml')):
        print('no touchdowns at %s: python3 mktouchdown.py' % d)
        return '', []
    man = json.load(open(os.path.join(d, 'touchdown.json')))
    mrs = GEO['reference']['mrs_in']
    out = []
    for e in re.findall(r'\t\t<touchdown>.*?</touchdown>\n', open(os.path.join(d, 'pywopwop-touchdown.xml')).read(), re.S):
        if re.search(r'<name>[^<]*_side</name>', e):
            e, n = re.subn(r'<volume><expression>(.*?)</expression></volume>',
                           lambda m: '<volume><expression><product><value>%.6g</value>%s</product></expression></volume>' % (scale['side'], m.group(1)), e, flags=re.S)
            assert n == 1, e
        e, n = re.subn(r'<position>\s*<x>([^<]+)</x>\s*<y>([^<]+)</y>\s*<z>([^<]+)</z>\s*</position>',
                       lambda m: '<position>\n\t\t\t\t<x>%.3f</x>\n\t\t\t\t<y>%.3f</y>\n\t\t\t\t<z>%.3f</z>\n\t\t\t</position>'
                       % (float(m.group(1)) + mrs[0] * IN, float(m.group(2)), float(m.group(3)) + mrs[2] * IN), e)
        assert n == 1, e
        out.append(e)
    return ''.join(out), [(os.path.join(d, x['file']), x['file']) for x in man['files']]


def main():
    bank = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/git/PyWopWop/out/x57')
    if not os.path.exists(os.path.join(bank, 'manifest.json')):
        print('no sound bank at %s: Sounds left as they are' % bank)
        return
    man = json.load(open(os.path.join(bank, 'manifest.json')))
    text = open(os.path.join(bank, 'pywopwop-props.xml')).read()
    g = man['gain_db']
    scale = dict(cockpit=1.0, side=10 ** ((g['cockpit'] - g['exterior']) / 20))
    os.makedirs(os.path.join(OUT, 'pywopwop'), exist_ok=True)
    used = set()
    wind_xml, wind_files = wind(bank)
    gear_xml, gear_files = gear_air(bank, scale)
    tyre_xml, tyre_files = tyres(bank, scale)
    touch_xml, touch_files = touchdown(bank, scale)
    full = full_scale_db(g['cockpit'])
    stock = os.path.join(HERE, '..', 'refs', 'fgaddon-p2006t', 'Tecnam-P2006T', 'Sounds')
    for old in ('rumble-p2006t.wav', 'touch-p2006t.wav'):                  # the stock rumble and contact, which PyWopWop's tyres replaced
        if os.path.exists(os.path.join(OUT, old)):
            os.remove(os.path.join(OUT, old))
    for dst, src in AIRFRAME.items():
        shutil.copy(os.path.join(stock, src), os.path.join(OUT, dst))
    shutil.copy(os.path.join(FG_ROOT, LOCK[1]), os.path.join(OUT, LOCK[0]))
    shutil.copy(os.path.join(FG_ROOT, BUTTON[1]), os.path.join(OUT, BUTTON[0]))
    extra, report = airframe(full)
    for mod, keep in (('mod3', lambda grp: grp.startswith('cruise')), ('mod4', lambda grp: True)):
        out = []
        for tag, body, grp, view in entries(text):
            if keep(grp):
                out.append(convert(tag, body, grp, view, scale))
                used.add(re.search(r'<path>Sounds/pywopwop/([^<]+)</path>', body).group(1))
        with open(os.path.join(OUT, 'x57-%s-sound.xml' % mod), 'w') as f:
            f.write('<?xml version="1.0"?>\n<!-- generated by fdm/x57sound.py from PyWopWop\'s bank -->\n<PropertyList>\n\t<fx>\n'
                    + ''.join(out) + extra + wind_xml + gear_xml + tyre_xml + touch_xml + '\t</fx>\n</PropertyList>\n')
        print('%s: %d loops' % (mod, len(out)))
    fresh = {name for _, name in wind_files + gear_files + tyre_files + touch_files}
    for name in os.listdir(os.path.join(OUT, 'pywopwop')):          # a bank's files it no longer makes (renamed or dropped) leave the package
        if name.startswith(('touchdown_', 'tyre_')) and name not in fresh:
            os.remove(os.path.join(OUT, 'pywopwop', name))
    for src, name in wind_files + gear_files + tyre_files + touch_files:
        shutil.copy(src, os.path.join(OUT, 'pywopwop', name))
    for name in sorted(used):
        src, dst = os.path.join(bank, name), os.path.join(OUT, 'pywopwop', name)
        if not (os.path.exists(dst) and os.path.getsize(dst) == os.path.getsize(src)):
            if subprocess.run(['cp', '-c', src, dst], capture_output=True).returncode:     # a clone on APFS: no extra disk
                shutil.copy(src, dst)
    print('%d files in Sounds/pywopwop; a full-scale sample at volume 1 is %.2f dB; outside loops at %.3f' % (
        len(used), full, scale['side']))
    print('the airframe\'s sounds at their levels:\n' + '\n'.join(report))


if __name__ == '__main__':
    main()
