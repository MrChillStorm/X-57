#!/usr/bin/env python3
"""Time histories for the report: the left motor lost with no pilot, on the built-up and the table model, into results/upset.json.
The rest of the report's numbers are the check logs, copied here from a run of check_all.sh.

    python3 mkresults.py
"""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
os.environ['X57_ROOT_TAG'] = '-rep'
import harness as H
import phase5_checks as C5


def run(built):
    if built:
        os.environ['X57_BUILT'] = '1'
    else:
        os.environ.pop('X57_BUILT', None)
    f = H.load('mod3')
    C5.clear(f)
    t = H.trim(f, 95, flap=0, gear=0, hp=6000.0, rpm=2500.0)
    lat = C5.asym(f, 95, 0, 0, 6000.0, 2500.0, t['lever'], 0, 0.0)
    a = math.radians(t['alpha'])
    H.freeze(f, False)
    f['ic/h-sl-ft'], f['ic/terrain-elevation-ft'] = 6000.0, 0.0
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, a, 0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = t['V'] * math.cos(a), 0, t['V'] * math.sin(a)
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
    H.configure(f, 0, 0, 0, 2500.0, t['lever'], stab=t['stab'], ail=lat['ail'], rud=lat['rud'])
    f.run_ic()
    f['ctl/hold-props'] = 0
    for _ in range(int(2.5 * H.HZ)):
        f.run()
    h0 = f['attitude/psi-deg']
    f['ctl/motor-on-l'] = 0
    out = {k: [] for k in ('t', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'alpha', 'beta', 'kcas', 'sink')}
    for i in range(int(3 * H.HZ) + 1):
        if i % 6 == 0:
            for k, v in (('t', i / H.HZ), ('p', math.degrees(f['velocities/p-aero-rad_sec'])), ('q', math.degrees(f['velocities/q-aero-rad_sec'])),
                         ('r', math.degrees(f['velocities/r-aero-rad_sec'])), ('phi', f['attitude/phi-deg']), ('theta', f['attitude/theta-deg']),
                         ('psi', (f['attitude/psi-deg'] - h0 + 180) % 360 - 180), ('alpha', f['aero/alpha-deg']), ('beta', f['aero/beta-deg']),
                         ('kcas', f['velocities/vc-kts']), ('sink', f['velocities/v-down-fps'])):
                out[k].append(round(float(v), 3))
        f.run()
    return out


which = sys.argv[1] if len(sys.argv) > 1 else 'built'
p = os.path.join(HERE, 'results', 'upset.json')
d = json.load(open(p)) if os.path.exists(p) else {}
d[which] = run(which == 'built')
json.dump(d, open(p, 'w'))
print('wrote', which)
