"""The X-57's avionics power and its switches, as the aircraft's [20240009560 fig. 5, 20240006533, ICD-CEPT-006 rev E].

The essential battery feeds the Essential Bus through ESS BATT. Two 13.8 V converters run off the traction batteries, upstream of the main contactors, each on its own DC Converter Bus; the essential bus
enables them. CROSS BUS A and B tie those buses to the Essential Bus, so the converters carry it and charge the battery. WING AVIONICS A and B take the Essential Bus up to 23 V for the motor controllers;
AVIONICS BUS A and B are the least critical and go first in a shed. The contactors want 28 V from the Essential Bus. A dead Essential Bus therefore drops traction: it is the last bus to lose.

Modelled as diodes and switches at fixed voltages: no sag. The bus loads are estimates, not published. They add up to 69 A with Mod IV's high-lift controllers on: 49 % of the two converters' 140 A, inside
NASA's 47 % for Mod II and III and 70 % for Mod IV, and under one converter's 70 A, since NASA sized each to carry the lot [20240006533]. The essential battery's capacity isn't published: 20 Ah assumed.
With every switch on and nothing failed this changes nothing in the flight model: the harness runs that way.
"""
import x57data as X
from xmlfun import S, M, D, Q, MAX, MIN, IFTHEN, GT, LT, T1, p, v

V_BUS, V_WING, V_28 = 13.8, 23.0, 28.0
V_ALIVE = 9.0                     # the controllers' minimum [20240009560]
V_CONV_IN = 310.0                 # a converter's input low limit [ICD slide 37]
ESS_AH = 20.0
ESS_SOC = (0.0, 0.1, 0.5, 1.0)
ESS_V = (11.0, 12.8, 13.1, 13.4)  # 12.8 V is where its annunciator calls it over-discharged [ICD slide 39]
LOAD_E, LOAD_A, LOAD_B, LOAD_AV_A, LOAD_AV_B = 20.0, 6.0, 5.0, 5.0, 17.0
SWITCHES = ('ess-batt', 'dc-conv-a', 'dc-conv-b', 'cross-bus-a', 'cross-bus-b', 'wing-av-a', 'wing-av-b', 'av-bus-a', 'av-bus-b',
            'tc-la', 'tc-lb', 'tc-ra', 'tc-rb', 'arm-la', 'arm-lb', 'arm-ra', 'arm-rb', 'hl-tc-a', 'hl-tc-b', 'cb-prop-l', 'cb-prop-r', 'cb-gear', 'cb-flap')
CMC = (('la', 'a'), ('lb', 'b'), ('ra', 'a'), ('rb', 'b'))     # controller, its bus: both for the high voltage and the wing avionics


def wing_load(hl):
    return 8.0 if hl else 4.0


def nominal(hl):
    """each converter's amps with everything on"""
    shared = (LOAD_E + 2 * wing_load(hl)) / 2
    return LOAD_A + LOAD_AV_A + shared, LOAD_B + LOAD_AV_B + shared


def declare(f, hl):
    for n in SWITCHES:
        f.declare('ctl/sw/' + n, 1)
    for s in 'lr':
        f.declare('ctl/pitch-man-' + s, 0)          # AUTO/MAN
        f.declare('ctl/pitch-sw-' + s, 0)           # START -1, FEATHER 1
    f.declare('ctl/cockpit', 0)                     # FlightGear: motors idle when armed, and the governor needs 200 rpm. The harness keeps the plain model
    f.declare('ctl/fail/conv-a', 0)
    f.declare('ctl/fail/conv-b', 0)
    f.declare('ctl/fail/ess-batt', 0)
    f.declare('elec/ess-soc', 1.0)
    f.declare('elec/ess-v', V_BUS)


def channel(f, hl):
    """run every frame, before anything that reads it"""
    f.channel('power')
    sw = lambda n: p('ctl/sw/' + n)
    alive = lambda x: GT(x, v(V_ALIVE))
    batt = f.fn('elec/ess-batt-v', M(T1('elec/ess-soc', ESS_SOC, ESS_V), D(v(1), p('ctl/fail/ess-batt'))))
    src = M(sw('ess-batt'), batt)
    conv = {}
    for b in 'ab':
        volts = p('bat/%s/volts' % b)
        low = f.fn('elec/conv-%s-low' % b, M(GT(volts, v(1)), LT(volts, v(V_CONV_IN))), desc='volts stand at 0 until the battery channel first runs')
        conv[b] = f.fn('elec/conv-%s-v' % b, M(v(V_BUS), sw('dc-conv-' + b), alive(p('elec/ess-v')), D(v(1), low), D(v(1), p('ctl/fail/conv-' + b))),
                       desc='the Essential Bus enables it, a frame old: once the cross bus carries that bus, it keeps itself going')
    ca, cb = sw('cross-bus-a'), sw('cross-bus-b')
    e = f.fn('elec/ess-bus-v', MAX(src, M(ca, conv['a']), M(cb, conv['b'])), out='elec/ess-v')
    bus = {'a': f.fn('elec/dc-a-v', MAX(conv['a'], M(ca, e))), 'b': f.fn('elec/dc-b-v', MAX(conv['b'], M(cb, e)))}
    av = {b: f.fn('elec/av-%s-v' % b, M(sw('av-bus-' + b), bus[b])) for b in 'ab'}
    wing = {b: f.fn('elec/wing-%s-v' % b, M(v(V_WING), sw('wing-av-' + b), alive(e))) for b in 'ab'}
    f.fn('elec/v28', M(v(V_28), alive(e)))
    for b in 'ab':
        f.fn('elec/bus-' + b, M(p('ctl/bus-' + b), alive(e)), desc='MAIN BATT: the contactor wants 28 V')
    for k, b in CMC:
        f.fn('elec/cmc-' + k, M(p('ctl/cmc-' + k), sw('tc-' + k), sw('arm-' + k), GT(wing[b], v(V_ALIVE))), desc='failed, its contactor open, disarmed or without its low voltage: it gives no torque')
    if hl:
        for b in 'ab':
            for s in 'lr':
                f.fn('elec/contactor-%s%s' % (b, s), M(p('ctl/contactor-%s%s' % (b, s)), sw('hl-tc-' + b), GT(wing[b], v(V_ALIVE))))
    for s in 'lr':
        f.fn('elec/hub-' + s, M(sw('cb-prop-' + s), alive(e)), desc='the pitch controller is on the Essential Bus: without it the blades stay')
    f.fn('elec/gear-power', M(sw('cb-gear'), alive(e)))
    f.fn('elec/flap-power', M(sw('cb-flap'), alive(e)))
    # the loads, amps at 13.8 V: what the converters carry
    la = f.fn('elec/load-a', M(alive(bus['a']), S(v(LOAD_A), M(v(LOAD_AV_A), alive(av['a'])))))
    lb = f.fn('elec/load-b', M(alive(bus['b']), S(v(LOAD_B), M(v(LOAD_AV_B), alive(av['b'])))))
    on_a, on_b = alive(conv['a']), alive(conv['b'])
    wl = wing_load(hl)
    tot = f.fn('elec/load-e', S(M(alive(e), S(v(LOAD_E), M(v(wl), GT(wing['a'], v(V_ALIVE))), M(v(wl), GT(wing['b'], v(V_ALIVE))))), M(D(v(1), on_a), ca, la), M(D(v(1), on_b), cb, lb)))
    na, nb = f.fn('elec/feed-a', M(ca, on_a)), f.fn('elec/feed-b', M(cb, on_b))
    n = S(na, nb)
    share = f.fn('elec/share', IFTHEN(GT(n, v(0.5)), Q(tot, MAX(v(1), n)), v(0)))
    na_amps, nb_amps = nominal(hl)
    f.fn('elec/conv-a-amps', S(M(on_a, la), M(na, share)))
    f.fn('elec/conv-b-amps', S(M(on_b, lb), M(nb, share)))
    f.fn('elec/ess-amps', M(sw('ess-batt'), IFTHEN(GT(n, v(0.5)), v(-2.0), tot)), desc='charging from the cross bus, carrying the bus without it')
    return na_amps, nb_amps


def battery(f):
    """in the battery's slow channel"""
    soc = p('elec/ess-soc')
    f.fn('elec/ess-soc-next', IFTHEN(GT(p('ctl/hold-now'), v(0.5)), soc, MIN(v(1), MAX(v(0), D(soc, M(p('elec/ess-amps'), p('simulation/channel-dt'), v(1 / (3600.0 * ESS_AH))))))), out='elec/ess-soc')
