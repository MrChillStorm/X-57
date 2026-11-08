#!/usr/bin/env python3
"""Phase 6: the wing's bending, against NASA's vibration test and in JSBSim. Limits in docs/PLAN.md.

    python3 phase6_checks.py [beam ring flight]
"""
import math, os, sys
os.environ['X57_BUILT'] = '1'
import numpy as np
import harness as H
import x57data as X
import x57flex as FX

fails = []


def check(name, val, ref, tol, fmt='%.2f', rel=True):
    t = tol * abs(ref) if rel else tol
    ok = abs(val - ref) <= t
    if not ok:
        fails.append(name)
    print('  %-52s %9s   target %-8s %s' % (name, fmt % val, fmt % ref, 'ok' if ok else 'MISS (limit %s)' % (('%.0f %%' % (100 * tol)) if rel else tol)))


def beam():
    print('\nThe beam against the vibration test [20240011864 tables 5, 9] and NASA\'s whole aircraft [20240007439 table 4]')
    c = FX.coefficients()
    A, B, Kr = c['A'], c['B'], c['Kr']
    tip = FX.TIP_LB * FX.LB
    s, a = FX.sym(A, B, tip), FX.anti(A, B, Kr, tip)
    s0, a0 = FX.sym(A, B), FX.anti(A, B, Kr)
    print('  fitted: symmetric %.2f (2.26), clean %.2f (6.55), antisymmetric %.2f (2.14)' % (s[0], s0[0], a[0]))
    check('clean, antisymmetric first bending, Hz', a0[0], 5.76, 0.05)
    check('tip masses, symmetric second bending, Hz', s[1], 15.76, 0.10)
    check('tip masses, antisymmetric second bending, Hz', a[1], 14.82, 0.10)
    check('clean, symmetric second bending, Hz', s0[1], 26.36, 0.10)
    check('clean, antisymmetric second bending, Hz', a0[1], 23.90, 0.10)
    f3, f4 = FX.free_sym(A, B, tip, []), FX.free_sym(A, B, tip, FX.hl_discs())
    print('  free in the air, antisymmetric: %.2f, %.2f Hz. NASA\'s table lists one first bending mode, without saying which; it sits nearest the symmetric' % (FX.free_anti(A, B, Kr, tip, []), FX.free_anti(A, B, Kr, tip, FX.hl_discs())))
    check('Mod III, free in the air, Hz', f3, 2.31, 0.05)
    check('Mod IV, free in the air, Hz', f4, 2.29, 0.05)


def clear_air(f, hp, kcas=None, u=300.0):
    f['flex/enable'] = 1
    H.freeze(f, False)
    f['ic/terrain-elevation-ft'] = 0.0
    f['ic/h-sl-ft'] = hp
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, 0, 0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = u, 0, 0
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0


def rate(t, y):
    """frequency, Hz, and damping ratio from a decaying record"""
    y = np.array(y) - np.mean(y[len(y) // 2:])
    z = [i for i in range(1, len(y)) if y[i - 1] < 0 <= y[i]]
    if len(z) < 3:
        return float('nan'), float('nan')
    f = (len(z) - 1) / (t[z[-1]] - t[z[0]])
    pk = [np.abs(y[a:b]).max() for a, b in zip(z, z[1:])]
    d = math.log(pk[0] / pk[-1]) / (len(pk) - 1)
    return f, d / math.sqrt(4 * math.pi ** 2 + d * d)


def ring(f3, f4):
    print('\nRinging with the air nearly gone: 300,000 ft, propellers parked')
    c = FX.coefficients()
    tip = FX.TIP_LB * FX.LB
    for f, name, discs in ((f3, 'Mod III', []), (f4, 'Mod IV', FX.hl_discs())):
        for kind, sign, want in (('symmetric', 1, FX.free_sym(c['A'], c['B'], tip, discs)), ('antisymmetric', -1, FX.free_anti(c['A'], c['B'], c['Kr'], tip, discs))):
            H.configure(f, 0, 0, 0, 2250.0, 0.0, stab=0.0)
            f['ctl/hold-props'] = 0
            clear_air(f, 300000.0)
            f.run_ic()
            for _ in range(30):
                f.run()
            f['flex/qL-ft'], f['flex/qR-ft'] = 0.3 * sign, 0.3
            t, y = [], []
            for i in range(int(12 * H.HZ)):
                f.run()
                t.append(i / H.HZ)
                y.append(f['flex/qR-ft'])
            fr, z = rate(t, y)
            print('  %s %s: %.3f Hz (beam %.3f), damping %.4f, kept %.0f %% after 12 s' % (name, kind, fr, want, z, 100 * abs(y[-1]) / 0.3))
            check('%s, %s, Hz' % (name, kind), fr, want, 0.03, '%.3f')
            check('%s, %s, damping' % (name, kind), z, FX.ZETA, 0.2, '%.4f')
            f['ctl/hold-props'] = 1
            f['flex/enable'] = 0
            H.freeze(f)


def flight(f3, f4):
    print('\nIn flight, trimmed level, 6,000 ft, a kick to each mode')
    for f, name, flap, mode, speeds in ((f3, 'Mod III', 0, 0, (100, 135, 175)), (f4, 'Mod IV', 0, 0, (100, 135, 175)), (f4, 'Mod IV, high lift', 30, 1, (70,))):
        for k in speeds:
            H.freeze(f)
            t = H.trim(f, k, flap=flap, gear=0, hl_mode=mode, hp=6000.0, rpm=2250.0)
            a = math.radians(t['alpha'])
            H.configure(f, flap, 0, mode, 2250.0, t['lever'], stab=t['stab'])
            clear_air(f, 6000.0)
            f['ic/theta-rad'] = a
            f['ic/u-fps'], f['ic/w-fps'] = t['V'] * math.cos(a), t['V'] * math.sin(a)
            f.run_ic()
            f['ctl/hold-props'] = 0
            for _ in range(int(0.5 * H.HZ)):
                f.run()
            q0 = 0.5 * (f['flex/qL-ft'] + f['flex/qR-ft'])
            out = []
            for sign in (1, -1):
                base = (f['flex/qL-ft'], f['flex/qR-ft'])
                f['flex/qL-ft'], f['flex/qR-ft'] = base[0] + 0.2 * sign, base[1] + 0.2
                tt, y = [], []
                for i in range(int(4 * H.HZ)):
                    f.run()
                    tt.append(i / H.HZ)
                    y.append(f['flex/qR-ft'] + sign * f['flex/qL-ft'])
                w = int(0.5 * H.HZ)                      # what's left after a half second's average: the wing's ringing, not the airplane's slower motion
                y = np.array(y)
                hp = y[w:-w] - np.convolve(y, np.ones(2 * w + 1) / (2 * w + 1), 'valid')
                out.append(rate(tt[w:-w], hp))
            good = all(z > 0 for _, z in out)
            if not good:
                fails.append('%s at %d KCAS' % (name, k))
            print('  %-18s %3d KCAS: alpha %.1f, tip %+.2f ft up; symmetric %.2f Hz, damping %.3f; antisymmetric %.2f Hz, %.3f  %s' % (name, k, t['alpha'], q0, out[0][0], out[0][1], out[1][0], out[1][1], 'ok' if good else 'MISS'))
            f['ctl/hold-props'] = 1
            f['flex/enable'] = 0
            H.freeze(f)
    print('\nDihedral effect, Mod III, 100 KCAS, flaps 0, bent to its 1 g shape against rigid')
    f = f3
    t = H.trim(f, 100, flap=0, gear=0, hp=6000.0, rpm=2250.0)
    a = math.radians(t['alpha'])
    out = {}
    for on in (0, 1):
        f['flex/enable'] = on
        c = {}
        for b in (-2.0, 2.0):
            H.configure(f, 0, 0, 0, 2250.0, t['lever'], stab=t['stab'])
            br = math.radians(b)
            H.state(f, t['V'] * math.cos(a) * math.cos(br), t['V'] * math.sin(br), t['V'] * math.sin(a) * math.cos(br), theta=a, hp=6000.0)
            for _ in range(6):
                f.run()
            c[b] = (f['aero/coef/Cl'], f['aero/coef/Cn'], f['flex/qR-ft'], f['flex/qL-ft'])
        out[on] = ((c[2.0][0] - c[-2.0][0]) / math.radians(4), (c[2.0][1] - c[-2.0][1]) / math.radians(4), 0.5 * (c[2.0][2] + c[2.0][3]))
    f['flex/enable'] = 0
    print('  Cl beta %.4f rigid, %.4f bent; Cn beta %.4f, %.4f; tip %.2f ft' % (out[0][0], out[1][0], out[0][1], out[1][1], out[1][2]))


def ground():
    """on the gear with the wings bending. JSBSim's tyres hold a standing airplane by friction that changes side each frame until the wheel stops: the wing's answer mustn't keep it moving"""
    print('\nOn the gear, wings bending, nothing running')
    f = H.load('mod4', rebuild=False)
    f['flex/enable'] = 1
    f['ctl/hold-props'] = 0
    H.configure(f, flap=0, gear=1, rpm=2250, lever=0, stab=0)
    keys = ('accelerations/n-pilot-z-norm', 'accelerations/Ny', 'flex/qL-ft')

    def spread(h, T, phi=0.0, u=0.0):
        f['ic/terrain-elevation-ft'] = 0
        f['ic/h-agl-ft'] = h
        f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, 0.011, phi
        f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = u, 0, 0
        f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
        H.freeze(f, False)
        f.run_ic()
        n = int(T * H.HZ)
        L = []
        for i in range(n):
            f.run()
            L.append([f[k] for k in keys])
        return np.ptp(np.array(L[n // 2:]), axis=0)
    for name, s in (('standing', spread(3.994, 30)), ('dropped 0.2 ft with 1 deg of bank', spread(4.2, 40, 0.02)), ('rolling at 10 kt', spread(3.994, 20, 0.0, 17.0))):
        check('%s, the pilot\'s g, spread' % name, float(s[0]), 0.0, 0.005, '%.4f', False)
        check('%s, side load factor, spread' % name, float(s[1]), 0.0, 0.005, '%.4f', False)
        check('%s, wing tip, ft, spread' % name, float(s[2]), 0.0, 0.002, '%.4f', False)
    spread(3.994, 8)
    f['ctl/instant'] = 0
    f['gear/gear-cmd-norm'] = 0
    for _ in range(int(4 * H.HZ)):
        f.run()
    check('gear lever up on the ground: the gear stays down', f['gear/gear-pos-norm'], 1.0, 0.001, '%.3f', False)
    f['gear/gear-cmd-norm'] = 1
    f['ctl/instant'] = 1


def main():
    what = sys.argv[1:] or ['beam', 'ring', 'flight', 'ground']
    if 'beam' in what:
        beam()
    if set(what) & {'ring', 'flight'}:
        f3, f4 = H.load('mod3'), H.load('mod4')
        if 'ring' in what:
            ring(f3, f4)
        if 'flight' in what:
            flight(f3, f4)
    if 'ground' in what:
        ground()
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else '%d misses: %s' % (len(fails), '; '.join(fails))))


if __name__ == '__main__':
    main()
