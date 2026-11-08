"""The tyres' side force, and the nose wheel's play, for the X-57 and the P2006T alike (genx57.py, genp2006t.py).

Side force. JSBSim's own curve is Pacejka's magic formula with fixed constants: its slope at small slip is 0.121 of the wheel's load a
degree for every tyre, and it peaks at 10.5 deg. Smiley and Horne's measured cornering power [NACA TN 4110 eq. 82, within about 20 %]
for these tyres, from Goodyear's data book [GOODYEAR] and the P2006T's pressures [AFM 8, 4.3], is 0.10-0.12 for the nose's 5.00-5 but
0.055-0.067 for the mains' 6.00-6, which carry 85 % of the weight; and the side force of two large tyres measured peaked at 14-18 deg of yaw [NACA TN 3235].
Here each tyre keeps JSBSim's shape and peak, 0.72 of its load, with the slope Smiley and Horne give it at its load (the mean of a
4-ply and a 6-ply, since the P2006T's plies aren't published). The mains' then peak near 24 deg, the nose's near 13.

The nose wheel's play. The pedals steer it in proportion, 20 deg at full travel [20220005440, 'from Tecnam's specifications'], with slack
in the linkage that small, quick pedal movements are lost in [same]. Within that play the wheel trails like a castor, taking no side
force, so near the centre the mains alone keep the aeroplane straight. Rigid, the nose tyre's force 8 ft ahead of the centre of gravity
outweighed the mains' 1.4 ft behind it: with the nose wheel held straight, a 15 kt quartering tailwind swung the aeroplane into the wind
and rolled it onto a tip propeller from 40 kt, where NASA's simulator, built on Tecnam's data, holds it straight with about 3 to 4 deg of
nose wheel [20220005440 fig. 6].

The play isn't published. PLAY_DEG is fitted to that case, NASA's simulator's: the nose-wheel angle a slow heading hold settles on in a 15 kt
wind from 45 deg behind, Mod III at takeoff flaps, comes to 2.5, 3.0, 3.2, 3.3, 3.3, 2.9, 2.0 deg from 20 to 80 kt against NASA's 3.7, 3.2,
3.2, 3.2, 3.2, 3.0, 2.0 (0.46 rms; 1.9 rigid). It isn't the whole story: with landing flaps 3 to 4 deg fit better, and the P2006T's own
model at Mod II's mass, NASA's aeroplane, needs only 0.4 to 1.9 deg rigid and matches at no play. Past the play the wheel is rigid, and a pedal input of 5 deg or
more from 20 kt still leans the aeroplane onto a tip propeller; NASA's simulator steers far more softly than geometry and these tyres allow,
for reasons its paper doesn't give.
"""
import math, os
from xmlfun import *

# [GOODYEAR] Goodyear Aviation Data Book 2022, section 4A: outside diameter, section width, static loaded radius at rated load and
# pressure (in); rated load (lb) and pressure (psi) by ply. [AFM 8, 4.3]: nose 1.7 bar, mains 2.3 bar
TYRES = {'6.00-6': dict(d=17.5, w=6.3, r_static=6.9, rated={4: (1150.0, 29.0), 6: (1750.0, 42.0)}, psi=33.0),
         '5.00-5': dict(d=14.2, w=4.95, r_static=5.7, rated={4: (800.0, 31.0), 6: (1285.0, 50.0)}, psi=24.0)}
PEAK = 0.72                      # the tyres' static friction on dry asphalt, as the brakes have it
SHAPE, CURVE = 2.8, 1.03         # JSBSim's own magic-formula shape and curvature
MAX_STEER_DEG = 20.0             # [AFM 4: the wheel's stop; 20220005440: +-20]
PLAY_DEG = float(os.environ.get('X57_PLAY', 2.0))      # deg each way of the pedals' angle, fitted to NASA's straight line (above)


def cornering(size, load_lb):
    """Smiley and Horne's cornering power, a degree of yaw, over the wheel's load: the mean of a 4-ply and a 6-ply at the AFM's pressure.
    N = Cc (p + 0.44 pr) w^2 f(delta/d), Cc = 1.2 lb/deg for type III tyres; the deflection from the loaded radius at rated load and pressure,
    in proportion to load and inversely to the pressure with the carcass's part (0.08 pr) added"""
    t = TYRES[size]
    out = []
    for load_r, pr in t['rated'].values():
        delta = (t['d'] / 2 - t['r_static']) * (load_lb / load_r) * (1.08 * pr) / (t['psi'] + 0.08 * pr)
        x = delta / t['d']
        f = 1.2 * x - 8.8 * x * x if x <= 0.0875 else 0.0674 - 0.34 * x
        out.append(1.2 * (t['psi'] + 0.44 * pr) * t['w'] ** 2 * f / load_lb)
    return sum(out) / len(out)


def table_xml(size, load_lb):
    """JSBSim's CORNERING_COEFF table: side force over load against slip (deg), the magic formula at this tyre's slope"""
    B = cornering(size, load_lb) / (PEAK * SHAPE)
    rows = []
    for s in [0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 30, 40, 60, 90]:
        x = B * s
        v = PEAK * math.sin(SHAPE * math.atan(x - CURVE * (x - math.atan(x))))
        rows += [(s, v)] + ([(-s, -v)] if s else [])
    rows.sort()
    return ('<table name="CORNERING_COEFF" type="internal"><tableData>\n' + '\n'.join('    %g %.4f' % r for r in rows) +
            '\n  </tableData></table>')


def nose_steering(f, x_nose_in, cmd='fcs/steer-cmd-norm'):
    """The nose wheel's angle (deg, + right) into fcs/steer-pos-deg, which the gear reads: the pedals' angle, and within PLAY_DEG either
    side of it the angle the wheel runs at, so its tyre slips none. f is the FCS being written (xmlfun)"""
    want = f.fn('gear/nws-pedal-deg', M(v(MAX_STEER_DEG), p(cmd)), desc='where the pedals put the nose wheel')
    arm = D(Q(p('inertia/cg-x-in'), v(12.0)), v(x_nose_in / 12.0))          # ft: the nose wheel ahead of the centre of gravity
    side = S(p('velocities/v-fps'), M(p('velocities/r-rad_sec'), arm))      # its sideways speed over the ground
    track = f.fn('gear/nws-track-deg', M(v(180 / math.pi), ATAN2(side, MAX(p('velocities/u-fps'), v(1.0)))), desc='where it would run, trailing')
    if PLAY_DEG:
        return f.fn('gear/nws-deg', MIN(S(want, v(PLAY_DEG)), MAX(D(want, v(PLAY_DEG)), track)), out='fcs/steer-pos-deg',
                    desc='trailing within the play either side of the pedals\' angle')
    return f.fn('gear/nws-deg', want, out='fcs/steer-pos-deg')
