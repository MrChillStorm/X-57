#!/usr/bin/env python3
"""Phase 4: the built-up model of Mod III and Mod IV against the CFD. Limits are the plan's, from the solvers' own disagreement. phase1_checks.py and phase2_checks.py, with X57_BUILT=1, run the same model against the rest.

    python3 phase4_checks.py [poweroff stall controls highlift failure]

Flaps 0, 10 and 30 are fitted, so their neutral curves are no test. Flaps 20, the controls, sideslip and everything with power aren't.
"""
import json, math, os, sys
os.environ['X57_BUILT'] = '1'
import numpy as np
import harness as H
import phase1_checks as C

HERE = os.path.dirname(os.path.abspath(__file__))
T = json.load(open(os.path.join(HERE, 'data', 'aero_tables.json')))
SOL = C.H_SOLVERS
LIM = {0.0: (0.044, 0.058), 10.0: (0.093, 0.087), 30.0: (0.060, 0.054)}      # CL, Cm: phase0_checks.py's spread, linear range
STALL = {0.0: 15.0, 10.0: 12.0, 20.0: 12.0, 30.0: 12.0}
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'MISS', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def air(f, r, V=200.0):
    C.setrow(f, r, V)
    return C.coef(f)


def poweroff(f):
    print('Power off, neutral, against the solvers\' mean. Linear range')
    rows = [r for r in C.L('poweroff')['rows'] if not r['tab'] and not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))]
    for flap in (0.0, 10.0, 20.0, 30.0):
        e = {'CL': [], 'Cm': [], 'CD': []}
        for r in rows:
            if r['flap'] == flap and r['alpha'] <= STALL[flap] - 2:
                m = air(f, r)
                for c in e:
                    v = [r[s][c] for s in SOL if s in r and c in r[s]]
                    if v:
                        e[c].append(m[c] - float(np.mean(v)))
        if flap == 20.0:
            print('  flaps 20, not fitted, between 10 and 30: worst CL %.3f, Cm %.3f, CD %.4f over %d points' % (*[np.abs(e[c]).max() for c in ('CL', 'Cm', 'CD')], len(e['CL'])))
            check('flaps 20: within the widest limits, 0.09 and 0.09', np.abs(e['CL']).max() <= 0.093 and np.abs(e['Cm']).max() <= 0.087)
            continue
        lc, lm = LIM[flap]
        check('flaps %2g: CL within %.3f, Cm within %.3f (fitted)' % (flap, lc, lm), np.abs(e['CL']).max() <= lc and np.abs(e['Cm']).max() <= lm, 'worst %.3f, %.3f, CD %.4f' % tuple(np.abs(e[c]).max() for c in ('CL', 'Cm', 'CD')))


def stall(f):
    print('Past stall: shape only. The solvers differ by 0.2-0.4 in CL here')
    A = np.array(T['alpha'])
    for flap in (0.0, 10.0, 30.0):
        i = T['flaps'].index(flap)
        cur = []
        for a in np.arange(8.0, 24.1, 1.0):
            H.configure(f, flap=flap, gear=0, lever=0.0, stab=0.0)
            H.air(f, 200.0, float(a), hp=2500.0)
            cur.append((a, f['aero/coef/CL'], np.interp(a, A, T['base']['CL'][i])))
        cur = np.array(cur)
        am, at = cur[np.argmax(cur[:, 1]), 0], cur[np.argmax(cur[:, 2]), 0]
        print('  flaps %2g: peak CL %.2f at %g deg (the table: %.2f at %g); past it, worst difference %.2f' % (flap, cur[:, 1].max(), am, cur[:, 2].max(), at, np.abs(cur[:, 1] - cur[:, 2]).max()))
        check('flaps %g: the peak within 1 deg of the table\'s' % flap, abs(am - at) <= 1.0)


def controls(f):
    print('Each input\'s increment, below stall, against the same solver\'s neutral case: RMS error (RMS of the increment itself). None of these is fitted but the aileron\'s gain')
    rows = [r for r in C.L('poweroff')['rows'] if r['flap'] in (0, 10, 30) and not r['tab'] and r['alpha'] <= STALL[r['flap']] - 2]
    base = {(r['flap'], r['alpha']): r for r in rows if not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))}
    show = dict(beta=('CY', 'Cn', 'Cl'), stab=('CL', 'Cm'), ail=('Cl', 'CL', 'Cm'), rud=('CY', 'Cn', 'Cl'))
    res = {}
    for r in rows:
        ks = [k for k in show if r[k]]
        if len(ks) != 1:
            continue
        k = ks[0]
        m, m0 = air(f, r), air(f, dict(r, beta=0.0, stab=0.0, ail=0.0, rud=0.0))
        b = base.get((r['flap'], r['alpha']))
        for c in show[k]:
            odd = c in ('CY', 'Cl', 'Cn')
            d = [r[s][c] - (b[s][c] if b and s in b and c in b[s] else 0.0) for s in SOL if s in r and c in r[s] and (odd or (b and s in b and c in b[s]))]
            if d:
                res.setdefault((k, c), []).append((m[c] - m0[c] - float(np.mean(d)), float(np.mean(d))))
    for (k, c), v in res.items():
        e, d = zip(*v)
        rel = rms(e) / max(rms(d), 1e-6)
        check('%-5s %-3s' % (k, c), rel <= 0.25 or rms(e) <= 0.005, 'error %.4f of %.4f: %.0f %%' % (rms(e), rms(d), 100 * rel))


def highlift(f):
    print('Mod IV with the CFD\'s own thrust through the slipstreams')
    sets = {s['kcas']: s for s in C.L('highlift_settings')}
    rows = [r for r in C.L('highlift') if not r['tab'] and not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))]

    def on(r):
        k = r['kcas']
        f['ctl/hl-rpm-test'] = sets[k]['rpm']
        f['bu/test/hl-thrust'] = sets[k]['T_lbf']
        f['bu/test/hl-torque'] = sets[k]['Q_lbft']
        C.setrow(f, r, sets[k]['v_fts'], hl_mode=3)
        return C.coef(f)

    def off(r):
        f['bu/test/hl-thrust'] = -999
        C.setrow(f, r, sets[r['kcas']]['v_fts'], hl_mode=0)
        return C.coef(f)
    st = {}
    for r in sorted(rows, key=lambda r: r['alpha']):
        if r['kcas'] == 58.0 and r['flap'] == 30.0:
            st[r['alpha']] = r
    cl = [(on(r)['CL'], a) for a, r in st.items()]
    best = max(cl)
    check('58 KCAS, flaps 30: CLmax 4.40-4.58', 4.40 <= best[0] <= 4.58, 'CL %.2f at %g deg' % best)
    star = {a: r['star'] for a, r in st.items() if 'star' in r and 'on' in r['star'] and 'off' in r['star']}
    dcl, dcm = [], []
    # the model's power-off curve follows the time-accurate solvers past stall, and STAR-CCM+ isn't one, so the increment is taken from STAR-CCM+'s own power-off case
    for a in (2.0, 4.0, 8.0, 10.0, 12.0, 14.0, 15.0):
        if a in star:
            m = on(st[a])
            dcl.append((a, m['CL'] - star[a]['off']['CL'], star[a]['on']['CL'] - star[a]['off']['CL']))
            dcm.append((a, m['Cm'] - star[a]['off']['Cm'], star[a]['on']['Cm'] - star[a]['off']['Cm']))
    print('  dCL, model (STAR-CCM+): ' + ' '.join('%g: %.2f (%.2f)' % x for x in dcl))
    print('  dCm, model (STAR-CCM+): ' + ' '.join('%g: %+.2f (%+.2f)' % x for x in dcm))
    v = [x[1] for x in dcl]
    check('dCL +1.1 to +1.7 between 2 and 15 deg', abs(v[0] - 1.1) <= 0.15 and abs(v[-1] - 1.7) <= 0.3, 'from %.2f to %.2f' % (v[0], v[-1]))
    w = [x[1] for x in dcm]
    check('dCm +0.6 falling to +0.1', abs(w[0] - 0.6) <= 0.15 and abs(w[-1] - 0.1) <= 0.25, 'from %+.2f to %+.2f' % (w[0], w[-1]))
    e = {}
    for r in rows:
        if r['alpha'] > STALL[r['flap']] - 2 or r['kcas'] > 75:
            continue
        m, m0 = on(r), off(r)
        for c in ('CL', 'Cm', 'CD'):
            d = [r[s]['on'][c] - r[s]['off'][c] for s in SOL if s in r and 'on' in r[s] and 'off' in r[s] and c in r[s]['on'] and c in r[s]['off']]
            if d:
                e.setdefault(c, []).append((m[c] - m0[c] - float(np.mean(d)), max(d) - min(d)))
    for c, v in e.items():
        x, sp = zip(*v)
        check('the props\' %s, 35-75 KCAS, below stall: inside the solvers\' spread' % c, rms(x) <= max(float(np.mean(sp)), 0.02) * 1.5, 'error %.3f, spread %.3f' % (rms(x), float(np.mean(sp))))
    # controls with the propellers on, at the CFD's thrust
    allr = [r for r in C.L('highlift') if not r['tab'] and r['alpha'] <= STALL[r['flap']] - 2 and r['kcas'] <= 90]
    base = {(r['kcas'], r['flap'], r['alpha']): r for r in allr if not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))}
    show = dict(beta=('CY', 'Cn', 'Cl'), stab=('CL', 'Cm'), ail=('Cl',), rud=('CY', 'Cn'))
    res = {}
    for r in allr:
        ks = [k for k in show if r[k]]
        if len(ks) != 1:
            continue
        k = ks[0]
        m, m0 = on(r), on(dict(r, beta=0.0, stab=0.0, ail=0.0, rud=0.0))
        b = base.get((r['kcas'], r['flap'], r['alpha']))
        for c in show[k]:
            odd = c in ('CY', 'Cl', 'Cn')
            d = []
            for s in SOL:
                x = r.get(s, {}).get('on', {})
                y = (b or {}).get(s, {}).get('on', {})
                if c in x and c in y:
                    d.append(x[c] - y[c])
                elif c in x and odd:
                    d.append(x[c])
            # one slip in the file: 90 KCAS, flaps 10, alpha 8, stabilator -12 and -15 have the wrong sign
            if k == 'stab' and d and float(np.mean(d)) * r['stab'] * (-1 if c == 'Cm' else 1) < 0:
                continue
            if d:
                res.setdefault((k, c), []).append((m[c] - m0[c] - float(np.mean(d)), float(np.mean(d))))
    for (k, c), v in res.items():
        e, d = zip(*v)
        rel = rms(e) / max(rms(d), 1e-6)
        check('power on: %-5s %-3s' % (k, c), rel <= 0.25 or rms(e) <= 0.005, 'error %.4f of %.4f: %.0f %%' % (rms(e), rms(d), 100 * rel))
    f['bu/test/hl-thrust'] = -999


def main():
    what = sys.argv[1:] or ['poweroff', 'stall', 'controls', 'highlift']
    f = H.load('mod3')
    f4 = None
    for w in what:
        print()
        if w == 'highlift':
            f4 = f4 or H.load('mod4')
            highlift(f4)
        else:
            globals()[w](f)
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else '%d misses: %s' % (len(fails), '; '.join(fails))))


if __name__ == '__main__':
    main()
