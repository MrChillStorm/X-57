#!/usr/bin/env python3
"""The touchdowns, from PyWopWop (its tyre.py, make_touchdown_bank): each wheel's squeal and thump, at the propeller bank's loudness. x57sound.py merges it.

    python3 mktouchdown.py [bank]        default ~/git/PyWopWop/out/x57; writes <bank>_touchdown

Squeal: a wheel touches turning slower than the ground goes by and slides until friction spins it up, losing I (slip/r)^2 / 2 whatever the load;
the load comes through the strut, its damping on the sink rate, then its spring, so a firm touchdown squeals short and a gentle one long, of the same
energy. The sound's share of that loss and its spectrum come from recorded landings: the 6.00-6's from a P2006T's, the 5.00-5's from a glider's.
Thump: the strut's damping force on the airframe at contact, from the same P2006T landing, where it was at the edge of hearing; written at its
estimate, a firm one through a soft knee where it would pass the cockpit's full scale (the manifest's thump_knee_db, thump_drive), so firmer always
thumps harder. Estimates both. Each wheel's files are one waveform under different envelopes, cross-faded by amplitude over the slip and sink rate.

The struts as genx57.py sets them (each sits 0.25 ft down at NASA's simulator's 3075 lb, damped at 0.6 of critical on its load, twice), the tyres'
grip its 0.5 sliding, the loads at the 3000 lb design weight. The flight model latches each wheel's sink rate and ground speed as its weight comes
on (genx57.py, gear/unit[n]/touchdown-sink-fps and touchdown-slip-kt), which the sounds read. The slip is the ground speed less the wheel's
own rim speed: it rolls on the ground and spins down in the air (x57data.WHEELS, PyWopWop's inertia and radius, checked here), so a wheel that
hops for a second touches down still turning and doesn't squeal. Squeals go by the spin-up's energy, I (slip / r)^2 / 2, thumps by the sink
rate squared: each file's volume follows that law between its window's neighbours, so no edge steps.
"""
import json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
bank = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/git/PyWopWop/out/x57')
sys.path.insert(0, os.path.join(os.path.dirname(bank), '..'))
sys.path.insert(0, HERE)
from pywopwop import tyre as T
from pywopwop.propeller import load_prop_aircraft
from x57data import GEAR, MASS, WHEELS
import mktyres

LBF, LBFT = 4.4482216, 14.5939029          # N a lbf; N/m a lbf/ft (and N s/m a lbf s/ft)


def strut(share):
    """spring (lbf/ft), damping (lbf s/ft) and load at the design weight (N) of a strut carrying this share of the weight"""
    load = MASS['weight_lb'] * share
    k = load / 0.25
    return k, 0.6 * math.sqrt(k) * math.sqrt(load / 32.174) * 2, MASS['design_weight_lb'] * share * LBF


main, nose = GEAR['main_load_share'] / 2, 1 - GEAR['main_load_share']
for size, w in WHEELS.items():           # the flight model spins the wheels down with these: the same wheels as the squeal's spin-up
    assert abs(w['inertia_kgm2'] - T.wheel_inertia(size)) < 1e-4 and abs(w['radius_m'] - T.TYRE_TYPES[size]['rolling_radius_m']) < 1e-6, size
at = {t['gear']: t['position'] for t in mktyres.tyres}
groups = []
for name, size, share, gears in (('main', '6.00-6', main, (1, 2)), ('nose', '5.00-5', nose, (0,))):
    k, c, w = strut(share)
    groups.append(dict(name=name, tyre_type=size, damping=c * LBFT, spring=k * LBFT, static_load_n=w, wheels=[dict(gear=g, position=at[g]) for g in gears]))

if __name__ == '__main__':
    gain = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
    m = T.make_touchdown_bank(load_prop_aircraft('x57'), bank + '_touchdown', groups, gain, 'p2006t')
    print('%d files in %s_touchdown' % (len(m['files']), bank))
