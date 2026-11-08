"""Phase 7's cockpit power and propeller modes, against NASA's handbook [ICD-CEPT-006, 20240006533, 20240009560 fig. 5]. Limits set before the first run.

    python3 phase7_checks.py
"""
import harness as H
fails = []


def check(name, ok, got=''):
    if not ok:
        fails.append(name)
    print('  %-66s %-22s %s' % (name, got, 'ok' if ok else 'MISS'))


def run(f, n=30):
    for _ in range(n):
        f.run()


def fly(f):
    H.configure(f, flap=0, gear=0, rpm=2250, lever=0.5)
    H.air(f, 220.0, 3.0)
    H.freeze(f, False)
    f['ctl/hold-props'] = 0
    run(f, 60)


def reset(f):
    for n in ('ess-batt', 'dc-conv-a', 'dc-conv-b', 'cross-bus-a', 'cross-bus-b', 'wing-av-a', 'wing-av-b', 'av-bus-a', 'av-bus-b', 'tc-la', 'tc-lb', 'tc-ra', 'tc-rb', 'arm-la', 'arm-lb', 'arm-ra', 'arm-rb'):
        f['ctl/sw/' + n] = 1
    f['ctl/fail/conv-a'] = f['ctl/fail/conv-b'] = 0
    f['ctl/bus-a'] = f['ctl/bus-b'] = 1
    run(f, 60)


def main():
    f = H.load('mod4')
    fly(f)
    print('\nEverything on')
    check('buses at 13.8 V', all(abs(f['elec/%s' % k] - 13.8) < 0.01 for k in ('ess-bus-v', 'dc-a-v', 'dc-b-v', 'av-a-v', 'av-b-v')))
    check('wing avionics at 23 V', f['elec/wing-a-v'] == 23 and f['elec/wing-b-v'] == 23)
    a, b = f['elec/conv-a-amps'], f['elec/conv-b-amps']
    check('converters under 70 A, and 47 to 70 % of the 140 A together', a < 70 and b < 70 and 0.47 <= (a + b) / 140 <= 0.70, '%.0f + %.0f A' % (a, b))
    check('the essential battery charges', f['elec/ess-amps'] < 0, '%.0f A' % f['elec/ess-amps'])
    print('\nFailures and sheds')
    f['ctl/sw/ess-batt'] = 0
    run(f)
    check('ESS BATT off in flight: the converters hold the bus', abs(f['elec/ess-bus-v'] - 13.8) < 0.01 and f['elec/cmc-la'] == 1)
    f['ctl/sw/cross-bus-a'] = f['ctl/sw/cross-bus-b'] = 0
    run(f)
    check('and both cross buses open: Essential Bus dead', f['elec/ess-bus-v'] == 0)
    check('contactors drop without it', f['elec/bus-a'] == 0 and f['elec/bus-b'] == 0)
    check('no torque left', f['prop/l/cmd-nm'] == 0 and f['prop/r/cmd-nm'] == 0, '%.1f Nm' % f['prop/l/cmd-nm'])
    check('a converter can\'t start again without the bus', f['elec/conv-a-v'] == 0)
    reset(f)
    check('all switches back: traction back', f['elec/bus-a'] == 1 and f['prop/l/cmd-nm'] > 0, '%.1f Nm' % f['prop/l/cmd-nm'])
    f['ctl/fail/conv-a'] = 1
    run(f)
    check('converter A failed: B carries it all, within 70 A', f['elec/conv-a-amps'] == 0 and f['elec/conv-b-amps'] < 70, '%.0f A' % f['elec/conv-b-amps'])
    check('Bus A kept through the cross bus', abs(f['elec/dc-a-v'] - 13.8) < 0.01)
    f['ctl/fail/conv-b'] = 1
    run(f)
    check('both failed: the battery carries the Essential Bus', 12.8 <= f['elec/ess-bus-v'] < 13.5 and f['elec/ess-amps'] > 0, '%.1f V, %.0f A' % (f['elec/ess-bus-v'], f['elec/ess-amps']))
    check('traction stays on the battery\'s bus', f['elec/bus-a'] == 1)
    f['ctl/sw/av-bus-a'] = f['ctl/sw/av-bus-b'] = 0
    load = f['elec/ess-amps']
    run(f)
    check('shedding the avionics buses cuts the load', f['elec/ess-amps'] < load, '%.0f to %.0f A' % (load, f['elec/ess-amps']))
    reset(f)
    print('\nHigh voltage')
    full = f['prop/l/cmd-nm']
    f['ctl/sw/arm-la'] = 0
    run(f)
    half = f['prop/l/cmd-nm']
    check('one controller disarmed: half the torque', abs(half - full / 2) < 0.5, '%.0f of %.0f Nm' % (half, full))
    f['ctl/sw/arm-lb'] = 0
    run(f)
    check('both disarmed: none', f['prop/l/cmd-nm'] == 0)
    reset(f)
    f['ctl/sw/tc-ra'] = 0
    run(f)
    check('LA and RA are on Wing Avionics A', f['elec/cmc-ra'] == 0 and f['elec/cmc-la'] == 1)
    reset(f)
    f['ctl/sw/wing-av-a'] = 0
    run(f)
    check('Wing Avionics A off: LA and RA gone, LB and RB stay', f['elec/cmc-la'] == 0 and f['elec/cmc-ra'] == 0 and f['elec/cmc-lb'] == 1 and f['elec/cmc-rb'] == 1)
    reset(f)
    print('\nPropeller modes')
    f['ctl/pitch-man-l'] = 1
    run(f, 10)
    p0 = f['prop/l/pitch-deg']
    run(f, 120)
    check('MAN: blades stay without a switch', abs(f['prop/l/pitch-deg'] - p0) < 0.01, '%.2f deg' % f['prop/l/pitch-deg'])
    f['ctl/pitch-sw-l'] = 1
    run(f, 120)
    up = f['prop/l/pitch-deg']
    check('FEATHER held: coarser', up > p0 + 0.5, '%.1f to %.1f deg' % (p0, up))
    f['ctl/pitch-sw-l'] = -1
    run(f, 120)
    check('START held: finer', f['prop/l/pitch-deg'] < up - 0.5, '%.1f deg' % f['prop/l/pitch-deg'])
    f['ctl/pitch-sw-l'] = -1            # still held towards START
    p1 = f['prop/l/pitch-deg']
    f['ctl/feather-l'] = 1
    run(f, 120)
    check('the lever\'s aft stop overrides MAN', f['prop/l/pitch-deg'] > p1 + 0.5, '%.1f to %.1f deg' % (p1, f['prop/l/pitch-deg']))
    f['ctl/pitch-sw-l'] = 0
    f['ctl/feather-l'] = 0
    f['ctl/pitch-man-l'] = 0
    f['ctl/sw/cb-prop-r'] = 0
    f['ctl/pitch-sw-r'] = 1
    f['ctl/pitch-man-r'] = 1
    p0 = f['prop/r/pitch-deg']
    run(f, 120)
    check('PROP CONT. breaker pulled: the hub stays', abs(f['prop/r/pitch-deg'] - p0) < 0.01)
    reset(f)
    f['ctl/sw/cb-prop-r'] = 1
    f['ctl/pitch-sw-r'] = 0
    f['ctl/pitch-man-r'] = 0
    f['ctl/cruise-rpm-l'] = 1800
    f['ctl/cruise-rpm-r'] = 2700
    run(f, 4)
    check('each propeller lever sets its own speed', abs(f['prop/l/rpm-cmd'] - 1800) < 1 and abs(f['prop/r/rpm-cmd'] - 2700) < 1, '%.0f, %.0f rpm' % (f['prop/l/rpm-cmd'], f['prop/r/rpm-cmd']))
    print('\nMotor response')
    f['ctl/cruise-rpm-l'] = f['ctl/cruise-rpm-r'] = 0
    reset(f)
    for k in ('la', 'lb'):
        f['ctl/sw/arm-' + k] = 0
    run(f, 120)
    check('both controllers disarmed: no torque, even with the rotor slow', f['prop/l/torque-nm'] <= 0, '%.1f Nm' % f['prop/l/torque-nm'])
    reset(f)
    f['ctl/cockpit'] = 1
    run(f, 240)
    full = f['prop/l/cmd-nm']
    f['/controls/engines/engine[0]/throttle'] = 0
    run(f, 30)                          # from half lever, a quarter of a second is half way to idle
    half = f['prop/l/cmd-nm']
    run(f, 120)
    check('in the cockpit a lever cut ramps, 127.5 Nm/s a controller', 0.2 * full < half < 0.8 * full and f['prop/l/cmd-nm'] < 0.2 * full, '%.0f, %.0f, %.0f Nm' % (full, half, f['prop/l/cmd-nm']))
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else '%d misses: %s' % (len(fails), '; '.join(fails))))


if __name__ == '__main__':
    main()
