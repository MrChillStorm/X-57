#!/usr/bin/env python3
"""Does it fly? Time-domain flights through JSBSim itself, with a simple pilot in the loop.

    python3 fly.py [mod3|mod4]

A takeoff from rest, a climb, gear and flaps away, level cruise left alone, a banked turn, then for Mod IV the high-lift
system on for a slow approach. These are checks that nothing diverges or misbehaves, not comparisons with NASA.
"""
import math, os, sys
import numpy as np
os.environ.setdefault('X57_BUILT', '1')      # what ships; X57_BUILT= flies the table model
import harness as H


def settle(f, seconds):
    for _ in range(int(seconds * H.HZ)):
        f.run()


class Pilot:
    """holds a pitch attitude with the stabilator, wings level or a bank with the ailerons, the ball with the rudder"""

    def __init__(s, f):
        s.f, s.i, s.p = f, 0.0, 0.0

    def step(s, theta=None, phi=0.0, speed=None):
        f = s.f
        if theta is not None:
            e = f['attitude/theta-deg'] - theta
            s.i = float(np.clip(s.i + 0.02 * e / H.HZ, -0.8, 0.8))
            f['fcs/elevator-cmd-norm'] = float(np.clip(0.06 * e + 0.05 * f['velocities/q-rad_sec'] * 57.3 + s.i, -1, 1))
        # a hand doesn't answer at the wing's 3.5 Hz. Raw roll rate on the ailerons does, and pumps the antisymmetric bending up
        s.p += (f['velocities/p-rad_sec'] * 57.3 - s.p) / (H.HZ * 0.25)
        f['fcs/aileron-cmd-norm'] = float(np.clip(0.05 * (phi - f['attitude/phi-deg']) - 0.03 * s.p, -1, 1))
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.03 * f['aero/beta-deg'], -0.5, 0.5))


def fly(mod):
    f = H.load(mod)
    H.freeze(f, False)
    f['flex/enable'] = 1
    f['ctl/instant'] = 1
    f['ctl/hold-props'] = 0
    H.configure(f, flap=10, gear=1, rpm=2700, lever=0.0, stab=-5, hl_mode=1 if mod == 'mod4' else 0)
    f['ic/terrain-elevation-ft'] = 2300.0
    f['ic/h-agl-ft'] = 5.0
    f['ic/vt-fps'] = 0.0
    f['ic/theta-deg'] = 0.0
    f['ic/phi-deg'] = 0.0
    f['ic/psi-true-deg'] = 0.0
    f['ic/u-fps'] = 0.0
    f['ic/v-fps'] = 0.0
    f['ic/w-fps'] = 0.0
    f.run_ic()
    settle(f, 6)
    print('%s on the wheels: pitch %.2f deg, height of the centre of gravity %.2f ft, weight on wheels %d' % (
        mod, f['attitude/theta-deg'], f['position/h-agl-ft'], f['gear/wow']))
    for e in (0, 1):
        f['/controls/engines/engine[%d]/throttle' % e] = 1.0
    vr = 77.0 if mod == 'mod4' else 88.0
    p = Pilot(f)
    t, x, liftoff = 0.0, 0.0, None
    x0 = f['position/distance-from-start-mag-mt']
    while t < 150:
        v = f['velocities/vc-kts']
        p.step(theta=9.0 if v > vr else 0.0)
        f['fcs/steer-cmd-norm'] = 0.0
        f.run()
        t += 1 / H.HZ
        if liftoff is None and not f['gear/wow']:
            liftoff = (t, v, (f['position/distance-from-start-mag-mt'] - x0) / 0.3048)
            print('  lift-off after %.1f s at %.0f KCAS, in %.0f ft' % liftoff)
        if f['position/h-agl-ft'] > 500:
            break
    assert liftoff and f['position/h-agl-ft'] > 400, 'never climbed: %.0f ft, %.0f KCAS' % (f['position/h-agl-ft'], f['velocities/vc-kts'])
    print('  500 ft after %.1f s at %.0f KCAS, %.0f ft/min' % (t, f['velocities/vc-kts'], f['velocities/h-dot-fps'] * 60))
    f['gear/gear-cmd-norm'] = 0
    f['ctl/flap-cmd-deg'] = 0
    if mod == 'mod4':
        f['ctl/hl-mode'] = 0
    # cruise, trimmed by the model's own levers, then left alone
    f['ctl/hold-props'] = 1
    f['flex/enable'] = 0
    t = H.trim(f, 133.0, flap=0, gear=0, hp=f['position/h-sl-ft'])
    H.freeze(f, False)
    f['flex/enable'] = 1
    H.configure(f, flap=0, gear=0, lever=t['lever'], stab=t['stab'])
    a = math.radians(t['alpha'])
    f['ic/theta-rad'] = a
    f['ic/phi-rad'] = 0.0
    f['ic/u-fps'] = t['V'] * math.cos(a)
    f['ic/v-fps'] = 0.0
    f['ic/w-fps'] = t['V'] * math.sin(a)
    f['ic/p-rad_sec'] = 0.0
    f['ic/q-rad_sec'] = 0.0
    f['ic/r-rad_sec'] = 0.0
    f.run_ic()
    f.run()
    f['ctl/hold-props'] = 0
    # the centre of gravity is 1.19 in left of the centreline: a few seconds wings level find the roll and yaw trim
    p = Pilot(f)
    for i in range(int(H.HZ * 15)):
        p.step()
        f.run()
    ail, rud = f['fcs/aileron-cmd-norm'], f['fcs/rudder-cmd-norm']
    h0 = f['position/h-sl-ft']
    settle(f, 60)
    print('  cruise, trimmed (aileron %.3f, rudder %.3f), hands off for 60 s: %.1f KCAS, height changed %.0f ft, bank %.1f deg, pitch %.1f deg' % (
        ail, rud, f['velocities/vc-kts'], f['position/h-sl-ft'] - h0, f['attitude/phi-deg'], f['attitude/theta-deg']))
    # a 30 degree banked turn
    p = Pilot(f)
    p.i = f['fcs/elevator-cmd-norm']
    th = f['attitude/theta-deg']
    psi0 = f['attitude/psi-deg']
    for i in range(int(H.HZ * 25)):
        p.step(theta=th + 1.0, phi=30.0)
        f.run()
    print('  30 deg bank for 25 s: heading moved %.0f deg, %.2f g, %.1f KCAS, sideslip %.1f deg' % (
        (f['attitude/psi-deg'] - psi0) % 360, f['accelerations/Nz'], f['velocities/vc-kts'], f['aero/beta-deg']))
    for i in range(int(H.HZ * 10)):
        p.step(theta=th, phi=0.0)
        f.run()
    print('  wings level again: bank %.1f deg, %.1f KCAS' % (f['attitude/phi-deg'], f['velocities/vc-kts']))

    def hold(speed, seconds, stop=None):
        """a pilot flying the speed on the pitch attitude"""
        t = 0.0
        while t < seconds:
            th = float(np.clip(2.0 + 0.6 * (f['velocities/vc-kts'] - speed), -8, 12))
            p.step(theta=th)
            f.run()
            t += 1 / H.HZ
            if stop and stop():
                break

    for e in (0, 1):
        f['/controls/engines/engine[%d]/throttle' % e] = 0.1
    hold(100.0, 60, lambda: f['velocities/vc-kts'] < 102)
    f['ctl/flap-cmd-deg'] = 10
    f['gear/gear-cmd-norm'] = 1
    if mod == 'mod4':
        speed = 75.0
        f['ctl/hl-mode'] = 2
        hold(90.0, 30, lambda: f['velocities/vc-kts'] < 92)
        f['ctl/flap-cmd-deg'] = 30
    else:
        speed = 94.0
        f['ctl/flap-cmd-deg'] = 30
    h0 = f['position/h-sl-ft']
    hold(speed, 45)
    f0 = f['position/h-sl-ft']
    hold(speed, 20)
    rate = (f['position/h-sl-ft'] - f0) * 3
    hl = (', high lift %.0f rpm, %.1f kW a disc' % (f['prop/hl/rpm'], f['prop/hl/power-each-kw'])) if mod == 'mod4' else ''
    print('  approach, flaps 30: %.1f KCAS (aiming %.0f), alpha %.1f, stabilator %.1f, %.0f ft/min%s' % (
        f['velocities/vc-kts'], speed, f['aero/alpha-deg'], f['ctl/stab-deg'], rate, hl))


if __name__ == '__main__':
    for m in sys.argv[1:] or ('mod3', 'mod4'):
        fly(m)
