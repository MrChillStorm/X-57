#!/usr/bin/env python3
"""Is what FlightGear loads the aircraft the checks test? Reads taken out of the graph must change nothing.

    python3 readcheck.py

FlightGear's build has functions dropped and written into their readers (genx57.py --fg), and the high-lift channels rest while the propellers are
stopped and disarmed. This flies that build beside one with none of it (X57_NO_OPT, X57_NO_GATE), through the same inputs: a cruise, flaps, the
high-lift propellers armed, spun up and stopped again, and a roll, and compares them step by step. It counts the property reads in each, too.
"""
import os, re, subprocess, sys
import numpy as np
import jsbsim

HERE = os.path.dirname(os.path.abspath(__file__))
TOL = 1e-9
WATCH = ['velocities/u-aero-fps', 'velocities/v-aero-fps', 'velocities/w-aero-fps', 'velocities/p-aero-rad_sec', 'velocities/q-aero-rad_sec', 'velocities/r-aero-rad_sec',
         'attitude/phi-rad', 'attitude/theta-rad', 'attitude/psi-rad', 'position/h-sl-ft', 'bu/axis/X', 'bu/axis/Y', 'bu/axis/Z', 'bu/axis/L', 'bu/axis/M', 'bu/axis/N',
         'x57/cruise[0]/rpm', 'x57/cruise[1]/rpm']
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'FAIL', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def build(mod, tag, env):
    root = os.path.join(HERE, 'root-read-%s-%s' % (mod, tag))
    d = os.path.join(root, 'aircraft', 'x57-' + mod)
    subprocess.run([sys.executable, os.path.join(HERE, 'genx57.py'), d, '--' + mod, '--fg', '--built'], check=True, stdout=subprocess.DEVNULL, env=dict(os.environ, **env))
    # the harness has no FlightGear behind it: its engines and the like come from the same folder as the checks'
    return root


def reads(root, mod):
    n = 0
    for part in ('aero', 'propulsion', 'flex', 'controls'):
        t = open(os.path.join(root, 'aircraft', 'x57-' + mod, 'Systems', 'x57-%s-%s.xml' % (mod, part)), errors='replace').read()
        t = re.sub(r'<!--.*?-->', '', t, flags=re.S)
        for attrs, body in re.findall(r'<channel\b([^>]*)>(.*?)</channel>', t, re.S):
            c = len(re.findall(r'<property[^>]*>[^<]+</property>|<independentVar|<input>', body))
            rate = re.search(r'execrate="(\d+)"', attrs)
            yield c / (int(rate.group(1)) if rate else 1), 'execute=' in attrs


def fly(root, mod):
    f = jsbsim.FGFDMExec(root, None)
    f.set_debug_level(0)
    f.set_dt(1 / 120.0)
    f.load_model('x57-' + mod)
    f['ic/h-sl-ft'], f['ic/vc-kts'], f['ic/gamma-deg'], f['ic/psi-true-deg'] = 3000.0, 110.0, 0.0, 90.0
    f.run_ic()
    f['/controls/engines/engine[0]/throttle'] = f['/controls/engines/engine[1]/throttle'] = 0.7
    out, live = [], []
    for k in range(int(36 * 120)):
        t = k / 120.0
        f['/controls/flight/flaps'] = 0.33 if 4 <= t < 26 else 0.0
        f['fcs/aileron-cmd-norm'] = 0.15 if 8 <= t < 10 else -0.15 if 12 <= t < 14 else 0.0
        if mod == 'mod4':
            f['ctl/hl-mode'] = 1 if 6 <= t < 20 else 0
        f.run()
        if k % 12 == 0:
            out.append([f[w] for w in WATCH])
            if mod == 'mod4':
                live.append(f['prop/hl/live'])
    return np.array(out), np.array(live)


def main():
    for mod in ('mod3', 'mod4'):
        a = build(mod, 'full', {'X57_NO_OPT': '1', 'X57_NO_GATE': '1'})
        b = build(mod, 'opt', {})
        rf = list(reads(a, mod))
        ro = list(reads(b, mod))
        full, opt = sum(c for c, _ in rf), sum(c for c, _ in ro)
        rest = sum(c for c, g in ro if not g)
        print('%s: %.0f reads a step unoptimised, %.0f optimised, %.0f with the high-lift channels at rest' % (mod, full, opt, rest))
        A, _ = fly(a, mod)
        B, live = fly(b, mod)
        check('%s: no NaN' % mod, np.isfinite(A).all() and np.isfinite(B).all())
        scale = np.maximum(np.abs(A).max(axis=0), 1.0)
        worst = float((np.abs(A - B) / scale).max())
        j = int(np.argmax((np.abs(A - B) / scale).max(axis=0)))
        check('%s: the two fly the same through 36 s' % mod, worst < TOL, 'worst %.2e of %s' % (worst, WATCH[j]))
        if mod == 'mod4':
            check('mod4: the high-lift channels ran and rested', live.max() > 0.5 and live[0] < 0.5 and live[-1] < 0.5, 'on for %.1f s of 36' % (live.mean() * 36))
        subprocess.run(['rm', '-rf', a, b])
    print('\n%s' % ('ALL CHECKS PASS' if not fails else 'FAILED: ' + '; '.join(fails)))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
