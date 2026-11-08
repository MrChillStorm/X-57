#!/usr/bin/env python3
"""The gear in the airflow, from PyWopWop: Fink's wheels and struts, at the propeller bank's loudness, so the two are in
proportion. x57sound.py merges it.

    python3 mkgear.py [bank]        default ~/git/PyWopWop/out/x57; writes <bank>_gear

Tyres: the P2006T's flight manual's 6.00-6 main and 5.00-5 nose, about 0.44 and 0.355 m across. The legs' size is an estimate:
no drawing of them is published.
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
bank = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/git/PyWopWop/out/x57')
sys.path.insert(0, os.path.join(os.path.dirname(bank), '..'))
sys.path.insert(0, HERE)
from pywopwop.gear import make_gear_bank
from pywopwop.propeller import load_prop_aircraft
from x57data import GEAR

IN = 0.0254
MAIN_D, NOSE_D, STRUT_D = 0.44, 0.355, 0.06
MAIN_LEG, NOSE_LEG = 0.40, 0.35
mrs = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))['reference']['mrs_in']


def leg(station, right, wheel, length):
    """a leg, its wheel's centre in the aircraft's frame (x forward, y left, z up, from the moment reference)"""
    z = GEAR['ground_z_in'] * IN + wheel / 2
    return dict(position=((mrs[0] - station) * IN, -right * IN, z - mrs[2] * IN), wheel_m=wheel, strut_m=STRUT_D, strut_length_m=length)


half = GEAR['track_in'] / 2
legs = [leg(GEAR['main_x_in'], -half, MAIN_D, MAIN_LEG), leg(GEAR['main_x_in'], half, MAIN_D, MAIN_LEG),
        leg(GEAR['main_x_in'] - GEAR['wheelbase_in'], 0.0, NOSE_D, NOSE_LEG)]
gain = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
make_gear_bank(load_prop_aircraft('x57'), bank + '_gear', legs, gain)
