#!/usr/bin/env python3
"""The cockpit's wind, from PyWopWop: the boundary layer on the windscreen through the glass, at the propeller bank's loudness, so the two
are in proportion. x57sound.py merges it.

    python3 mkwind.py [bank]        default ~/git/PyWopWop/out/x57; writes <bank>_wind
"""
import json, os, sys
bank = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/git/PyWopWop/out/x57')
sys.path.insert(0, os.path.join(os.path.dirname(bank), '..'))
from pywopwop.propeller import load_prop_aircraft
from pywopwop.wind import make_wind_bank

g = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']['cockpit']
make_wind_bank(load_prop_aircraft('x57'), bank + '_wind', speeds=(30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 160.0, 200.0), gain_db=g)
