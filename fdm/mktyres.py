#!/usr/bin/env python3
"""The tyres rolling, from PyWopWop (its tyre.py), at the propeller bank's loudness, so the two are in proportion. x57sound.py merges it.

    python3 mktyres.py [bank]        default ~/git/PyWopWop/out/x57; writes <bank>_tyres

Tyres: the P2006T's flight manual's 6.00-6 mains and 5.00-5 nose, sized by Goodyear's Aviation Databook (2022): 0.4445 m across and 0.16 wide,
0.3607 and 0.1257. Each sounds at its contact patch. Static compression: genx57.py sets each strut to sit 0.25 ft down at NASA's simulator's
3075 lb; FlightGear flies the 3000 lb design weight. The cockpit's transfer is the P2006T's ('p2006t'), from a recorded landing in that cabin:
the X-57 kept its fuselage and gear. Outside it is the tyre's airborne sound, scaled from a measured truck tyre; asphalt only.
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
bank = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/git/PyWopWop/out/x57')
sys.path.insert(0, os.path.join(os.path.dirname(bank), '..'))
sys.path.insert(0, HERE)
from pywopwop import tyre as T
from pywopwop.propeller import load_prop_aircraft
from x57data import GEAR, MASS

IN = 0.0254
MAIN = dict(diameter_m=0.4445, width_m=0.16)
NOSE = dict(diameter_m=0.3607, width_m=0.1257)
mrs = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))['reference']['mrs_in']
sag = 0.25 * MASS['design_weight_lb'] / MASS['weight_lb']          # ft: each strut's static compression at the design weight


def at(station, right):
    """a contact patch in the aircraft's frame (x forward, y left, z up, m from the moment reference)"""
    return (round((mrs[0] - station) * IN, 3), round(-right * IN, 3), round((GEAR['ground_z_in'] - mrs[2]) * IN, 3))


half = GEAR['track_in'] / 2
tyres = [dict(name='nose', gear=0, position=at(GEAR['main_x_in'] - GEAR['wheelbase_in'], 0.0), static_compression_ft=round(sag, 4), **NOSE),
         dict(name='left_main', gear=1, position=at(GEAR['main_x_in'], -half), static_compression_ft=round(sag, 4), **MAIN),
         dict(name='right_main', gear=2, position=at(GEAR['main_x_in'], half), static_compression_ft=round(sag, 4), **MAIN)]

if __name__ == '__main__':
    gain = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
    m = T.make_tyre_bank(load_prop_aircraft('x57'), bank + '_tyres', tyres, gain, 'p2006t')
    print('%d files in %s_tyres; below 5 kt each falls as speed to %s' % (len(m['files']), bank, m['low_speed_power']))
