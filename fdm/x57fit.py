#!/usr/bin/env python3
"""The wing's section corrections, fitted to the CFD's power-off curves, into data/x57_section_fit.json.

    python3 x57fit.py [hl]       hl: the high-lift moment alone

NeuralFoil's clean section and a generic slotted flap can't hold what twelve nacelles, a cove and the flap's slot do to the wing. So for flaps 0, 10 and 30 this finds what must be added to the section's lift,
moment and drag, by the section's own angle, for the strips and the line to give the CFD's curves on the table model. Flaps add to the clean wing's. Flaps 20, the controls, sideslip, rates and everything with power stay
out of the fit, as checks.
"""
import json, os, sys
os.environ['X57_BUILT'] = '1'
import numpy as np
import harness as H
import x57built as B

HERE = os.path.dirname(os.path.abspath(__file__))
T = json.load(open(os.path.join(HERE, 'data', 'aero_tables.json')))
GRID = np.arange(-12.0, 34.0, 1.0)
ALPHAS = np.arange(-6.0, 25.0, 1.0)
PROBE = 'bu/wR6/a'           # a strip mid-way along the flap: the section's angle to file the error under


def errors(f, flap):
    i = T['flaps'].index(flap)
    A = np.array(T['alpha'])
    out = []
    for a in ALPHAS:
        H.configure(f, flap=flap, gear=0, lever=0.0, stab=0.0)
        H.air(f, 200.0, float(a), hp=2500.0)
        out.append((f[PROBE], *[np.interp(a, A, T['base'][k][i]) - f['aero/coef/' + k] for k in ('CL', 'Cm', 'CD')]))
    return np.array(out)


def main():
    if os.path.exists(B.FIT_FILE):
        os.remove(B.FIT_FILE)
    flaps = (0.0, 10.0, 30.0)
    C = {str(fl): dict(a=GRID.tolist(), dcl=[0.0] * len(GRID), dcm=[0.0] * len(GRID), dcd=[0.0] * len(GRID)) for fl in flaps}
    C['body'] = dict(alpha=ALPHAS.tolist(), flaps=list(flaps), Cm=[[0.0] * 3 for _ in ALPHAS])
    share = {0.0: 0.8, 10.0: 0.55, 30.0: 0.55}          # the share of the wing each fit reaches, less what the line takes back
    SPLIT = 8.0                                         # section moments below this section angle; above it, the body's
    for it in range(int(sys.argv[1]) if len(sys.argv) > 1 else 9):
        worst = {}
        for j, flap in enumerate(flaps):
            f = H.load('mod3')
            e = errors(f, flap)
            o = np.argsort(e[:, 0])
            c = C[str(flap)]
            for name, col, k in (('dcl', 1, 0.8 / share[flap]), ('dcm', 2, 0.6 / share[flap]), ('dcd', 3, 0.7 / share[flap])):
                d = np.interp(GRID, e[o, 0], e[o, col]) * k
                if name == 'dcm':
                    d = d * (GRID <= SPLIT)
                c[name] = (np.array(c[name]) + d).tolist()
            # past the split what's left of the moment goes on the body's incidence
            late = e[:, 0] > SPLIT
            for i in range(len(ALPHAS)):
                if late[i]:
                    C['body']['Cm'][i][j] += 0.7 * e[i, 2]
            worst[flap] = np.abs(e[(ALPHAS >= -2) & (ALPHAS <= 12), 1:]).max(axis=0)
            with open(B.FIT_FILE, 'w') as g:
                json.dump(C, g)
        print('pass %d: worst CL, Cm, CD up to 12 deg: ' % it + '; '.join('flaps %g %.3f %.3f %.4f' % (k, *v) for k, v in worst.items()))
    C['rates'] = rates()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['hl'] = hl_moment()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['ground'] = ground_fit()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['ground_hl'] = ground_hl_fit()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['beta'] = beta_fit()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['cruise'] = cruise_fit()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)
    C['rud'] = rud_fit()
    with open(B.FIT_FILE, 'w') as g:
        json.dump(C, g)


def ground_at(f, h, flap, a, V=170.0, mode=0, rpm=0):
    import math
    H.configure(f, flap=flap, gear=0, lever=0.0, stab=0.0, hl_mode=mode)
    if mode:
        f['ctl/hl-rpm-test'] = rpm
    f['ctl/one-aileron'] = 0
    al = math.radians(a)
    f['ic/h-sl-ft'] = 2500.0
    f['ic/terrain-elevation-ft'] = 2500.0 - (h - (87.5 - f['inertia/cg-z-in']) / 12)
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, al, 0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = V * math.cos(al), 0, V * math.sin(al)
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
    H.freeze(f)
    f.run_ic()
    f.run()
    f.run()
    return f['aero/coef/CL'], f['aero/coef/Cm'], f['bu/h-b']


def ground_table(pts, hb):
    """on height over span, incidence and flap, nil at 1.5 spans. The CFD has fewer incidences at some heights"""
    flaps = [10.0, 30.0]
    hs = sorted(hb)
    al = sorted({k[2] for k in pts})
    x = np.array([hb[h] for h in hs])
    out = dict(hb=[float(v) for v in x] + [1.5], alpha=al, flaps=flaps, CL=[], Cm=[])
    for c, n in (('CL', 0), ('Cm', 1)):
        for fl in flaps:
            T = np.full((len(hs), len(al)), np.nan)
            for (f_, h, a), e in pts.items():
                v = [q[n] for q in e if q[n] is not None]
                if f_ == fl and v:
                    T[hs.index(h), al.index(a)] = np.mean(v)
            for j in range(len(al)):
                ok = np.isfinite(T[:, j])
                T[:, j] = np.interp(x, x[ok], T[ok, j]) if ok.any() else np.nan
            for i in range(len(hs)):
                ok = np.isfinite(T[i])
                T[i] = np.interp(al, np.array(al)[ok], T[i, ok]) if ok.any() else 0.0
            out[c].append(np.r_[T, np.zeros((1, len(al)))].tolist())
    return out


def ground_fit():
    """The wake's image gets the wing's ground effect and the tail's relief about right on average, but the CFD's changes grow faster near the
    ground, and differently by flap. What is left, on wing height, incidence and flap, from the CFD's power-off ground planes"""
    import phase1_checks as K
    f = H.load('mod3')
    ground_at(f, 900.0, 10, 0.0)
    pts, hb = {}, {}
    for r in K.L('poweroff_ground'):
        if r['alpha'] > 10 or r['flap'] not in (10, 30):
            continue
        d = {c: [r[s]['delta'][c] for s in ('star', 'lava') if s in r and 'delta' in r[s] and (abs(r[s]['delta'][c]) < 0.5 or c == 'Cm')] for c in ('CL', 'Cm')}
        l, m, h = ground_at(f, r['height_ft'], r['flap'], r['alpha'])
        l0, m0, _ = ground_at(f, 900.0, r['flap'], r['alpha'])
        pts.setdefault((float(r['flap']), r['height_ft'], r['alpha']), []).append((float(np.mean(d['CL'])) - (l - l0) if d['CL'] else None, float(np.mean(d['Cm'])) - (m - m0) if d['Cm'] else None))
        hb[r['height_ft']] = h
    out = ground_table(pts, hb)
    print('ground left to the table: heights over span %s, worst CL %.3f, Cm %.3f' % (np.round(out['hb'], 3), np.abs(out['CL']).max(), np.abs(out['Cm']).max()))
    return out


def ground_hl_fit():
    """The same with the high-lift propellers on, over what the power-off fit already gives: LAVA's ground planes less its free air, speeds averaged"""
    import phase1_checks as K
    f = H.load('mod4')
    sets = {s['rpm']: s for s in K.L('highlift_settings')}
    free = {(r['kcas'], r['flap'], r['alpha']): r for r in K.L('highlift') if not any(r[k] for k in ('beta', 'ail', 'stab', 'rud', 'tab'))}
    pts, hb = {}, {}
    for r in K.L('highlift_ground'):
        if any(r[k] for k in ('beta', 'ail', 'stab', 'rud', 'tab')) or r['alpha'] > 10 or r['rpm'] not in sets or r['flap'] not in (10, 30):
            continue
        s = sets[r['rpm']]
        b = free.get((s['kcas'], r['flap'], r['alpha']))
        if not b or 'lava' not in b or 'on' not in b['lava'] or 'lava' not in r or 'on' not in r['lava']:
            continue
        f['bu/test/hl-thrust'], f['bu/test/hl-torque'] = s['T_lbf'], s['Q_lbft']
        l, m, h = ground_at(f, r['height_ft'], r['flap'], r['alpha'], s['v_fts'], 3, r['rpm'])
        l0, m0, _ = ground_at(f, 900.0, r['flap'], r['alpha'], s['v_fts'], 3, r['rpm'])
        pts.setdefault((float(r['flap']), r['height_ft'], r['alpha']), []).append((r['lava']['on']['CL'] - b['lava']['on']['CL'] - (l - l0), r['lava']['on']['Cm'] - b['lava']['on']['Cm'] - (m - m0)))
        hb[r['height_ft']] = h
    f['bu/test/hl-thrust'] = -999
    out = ground_table(pts, hb)
    print('ground left with the propellers on: worst CL %.3f, Cm %.3f' % (np.abs(out['CL']).max(), np.abs(out['Cm']).max()))
    return out


def hl_moment():
    """With the propellers on, the CFD's pitching moment breaks at the same body incidence as with them off, though a blown section is well short of stall. Lift and drag follow the strips; what they leave of
    the moment goes on J, incidence and flap, as the CFD gives it with its own thrust, scaled by the share of discs working"""
    import phase1_checks as K
    SOL = K.H_SOLVERS
    f = H.load('mod4')
    sets = sorted(K.L('highlift_settings'), key=lambda s: s['J'])
    rows = [r for r in K.L('highlift') if not r['tab'] and not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))]
    A = np.array([-2.0, 2.0, 4.0, 6.0, 8.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0])
    flaps = [0.0, 10.0, 30.0]
    Cm = np.zeros((3, len(sets), len(A)))
    for k, flap in enumerate(flaps[1:], 1):
        for i, s in enumerate(sets):
            pts = {r['alpha']: r for r in rows if r['kcas'] == s['kcas'] and r['flap'] == flap}
            al, e = [], []
            for a, r in sorted(pts.items()):
                v = [r[q]['on']['Cm'] for q in SOL if q in r and 'on' in r[q] and 'Cm' in r[q]['on']]
                if not v:
                    continue
                f['ctl/hl-rpm-test'] = s['rpm']
                f['bu/test/hl-thrust'] = s['T_lbf']
                f['bu/test/hl-torque'] = s['Q_lbft']
                K.setrow(f, r, s['v_fts'], hl_mode=3)
                al.append(a)
                e.append(float(np.mean(v)) - K.coef(f)['Cm'])
            if al:
                Cm[k, i] = np.interp(A, al, e)
    print('moment left to the table, 58 KCAS, flaps 30: ' + ' '.join('%g: %+.2f' % (a, v) for a, v in zip(A, Cm[2, [s['kcas'] for s in sets].index(58.0)])))
    return dict(J=[s['J'] for s in sets], alpha=A.tolist(), flaps=flaps, Cm=Cm.tolist())


def rates():
    """The damping the strips give against the table model's, on alpha and flap. The tail's lag is in the sum for pitch, as a forced oscillation has it. NASA's modes fit the tunnel's rates, which the table
    holds; the strips land on the CFD's. So what's added here carries them to the tunnel, as the table model is"""
    f = H.load('mod3')
    R = T['rates']
    A = [a for a in R['alpha'] if -5 <= a <= 18]
    V, e, c, b = 170.0, 0.05, 2.13, 31.623
    import math
    out = dict(alpha=A, flaps=[0.0, 10.0, 30.0], **{k: np.zeros((len(A), 3)) for k in ('Cmq', 'Clp', 'Clr', 'Cnp', 'Cnr')})

    def at(a, flap, p=0.0, q=0.0, r=0.0, ad=0.0):
        H.configure(f, flap=flap, gear=0, lever=0.0, stab=-3.0)
        f['bu/test/alphadot'] = ad
        al = math.radians(a)
        H.state(f, V * math.cos(al), 0.0, V * math.sin(al), p, q, r, 0.0, al, hp=2500.0)
        return {k: f['aero/coef/' + k] for k in ('Cm', 'Cl', 'Cn')}
    for j, flap in enumerate(out['flaps']):
        for i, a in enumerate(A):
            ra = list(R['alpha']).index(a)
            d = lambda x, y, k, h: (y[k] - x[k]) / h
            hq, hb = 2 * e * c / (2 * V), 2 * e * b / (2 * V)
            q0, q1, a0, a1 = at(a, flap, q=-e), at(a, flap, q=e), at(a, flap, ad=-0.1), at(a, flap, ad=0.1)
            p0, p1, r0, r1 = at(a, flap, p=-e), at(a, flap, p=e), at(a, flap, r=-e), at(a, flap, r=e)
            own = dict(Cmq=d(q0, q1, 'Cm', hq) + d(a0, a1, 'Cm', 0.2 * c / (2 * V)), Clp=d(p0, p1, 'Cl', hb), Cnp=d(p0, p1, 'Cn', hb), Clr=d(r0, r1, 'Cl', hb), Cnr=d(r0, r1, 'Cn', hb))
            for k in own:
                out[k][i, j] = R[k][j][ra] - own[k]
    print('rate corrections at alpha 2, flaps 0: ' + ' '.join('%s %+.3f' % (k, out[k][A.index(2.0), 0]) for k in ('Cmq', 'Clp', 'Clr', 'Cnp', 'Cnr')))
    return {k: (v.tolist() if hasattr(v, 'tolist') else v) for k, v in out.items()}


def beta_fit():
    """Sideslip's roll and yaw, on sideslip, incidence and flap. The strips match the CFD at low incidence, but near stall their
    windward lift goes and the roll with it, and the fin's yaw stays, where the CFD keeps the roll and loses the yaw. What's left over
    the solvers' mean goes here. The CFD has 5 to 20 deg of sideslip at 2 deg and from 8 to past stall: odd in sideslip, held past both ends"""
    import phase1_checks as K
    SOL = K.H_SOLVERS
    f = H.load('mod3')
    rows = [r for r in K.L('poweroff')['rows'] if r['flap'] in (0, 10, 30) and not r['tab']]
    base = {(r['flap'], r['alpha']): r for r in rows if not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))}
    A, Bb, flaps = [2.0, 8.0, 10.0, 12.0, 14.0, 16.0], [5.0, 10.0, 15.0, 20.0], [0.0, 10.0, 30.0]
    out = dict(beta=[-b for b in Bb[::-1]] + [0.0] + Bb, alpha=A, flaps=flaps)
    for c in ('Cl', 'Cn'):
        out[c] = []
        for flap in flaps:
            pts = {}
            for r in rows:
                if r['flap'] != flap or not r['beta'] or any(r[k] for k in ('stab', 'ail', 'rud')):
                    continue
                b = base.get((flap, r['alpha']))
                d = [r[s][c] - (b[s][c] if b and s in b and c in b[s] else 0.0) for s in SOL if s in r and c in r[s]]
                if d:
                    K.setrow(f, r, 200.0)
                    m = K.coef(f)[c]
                    K.setrow(f, dict(r, beta=0.0), 200.0)
                    sign = 1.0 if r['beta'] > 0 else -1.0
                    pts.setdefault(r['alpha'], {})[abs(r['beta'])] = sign * (float(np.mean(d)) - (m - K.coef(f)[c]))
            al = sorted(pts)
            on_b = np.array([np.interp(Bb, [0.0] + sorted(pts[a]), [0.0] + [pts[a][x] for x in sorted(pts[a])]) for a in al])
            grid = np.array([np.interp(A, al, on_b[:, j]) for j in range(len(Bb))])      # [beta][alpha]
            out[c].append(np.vstack([-grid[::-1], np.zeros(len(A)), grid]).tolist())
    print('sideslip left to the table, flaps 0, 10 deg of sideslip, by alpha: Cl ' + ' '.join('%+.4f' % v for v in out['Cl'][0][6]) + ', Cn ' + ' '.join('%+.4f' % v for v in out['Cn'][0][6]))
    return out


def rud_fit():
    """The rudder. The fin's section table runs out of lift towards full travel, where the CFD goes on nearly straight to 28 deg: a quarter
    of its yaw and side force at full rudder, which is what sets the minimum control speed. What's left over the solvers' mean goes here"""
    import phase1_checks as K
    SOL = K.H_SOLVERS
    f = H.load('mod3')
    rows = [r for r in K.L('poweroff')['rows'] if r['flap'] in (0, 10, 30) and not r['tab']]
    base = {(r['flap'], r['alpha']): r for r in rows if not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))}
    A, Rr, flaps = [2.0, 8.0, 10.0, 12.0, 14.0, 16.0], [5.0, 10.0, 18.0, 28.0], [0.0, 10.0, 30.0]
    K.setrow(f, dict(next(r for r in rows if r['rud'] == -28.0)), 200.0)
    full = abs(f['ctl/rud-deg'])
    sense = f['ctl/rud-deg'] / -28.0 / abs(f['ctl/rud-deg'] / -28.0)          # the model's sign for the CFD's
    out = dict(rud=[sense * -x * full / 28.0 for x in Rr[::-1]] + [0.0] + [sense * x * full / 28.0 for x in Rr], alpha=A, flaps=flaps)
    if out['rud'][0] > out['rud'][-1]:
        out['rud'] = out['rud'][::-1]
    up = out['rud'][-1] > 0
    for c in ('CY', 'Cn'):
        out[c] = []
        for flap in flaps:
            pts = {}
            for r in rows:
                if r['flap'] != flap or not r['rud'] or any(r[k] for k in ('stab', 'ail', 'beta')):
                    continue
                b = base.get((flap, r['alpha']))
                d = [r[s][c] - (b[s][c] if b and s in b and c in b[s] else 0.0) for s in SOL if s in r and c in r[s]]
                if d:
                    K.setrow(f, r, 200.0)
                    m = K.coef(f)[c]
                    K.setrow(f, dict(r, rud=0.0), 200.0)
                    pts.setdefault(r['alpha'], {})[abs(r['rud'])] = float(np.mean(d)) - (m - K.coef(f)[c])     # at the CFD's negative rudder
            al = sorted(pts)
            on_r = np.array([np.interp(Rr, [0.0] + sorted(pts[a]), [0.0] + [pts[a][x] for x in sorted(pts[a])]) for a in al])
            grid = np.array([np.interp(A, al, on_r[:, j]) for j in range(len(Rr))])      # [rud][alpha], for the CFD's negative rudder
            neg_is_low = sense > 0                                                       # the CFD's negative rudder sits at the table's low end
            low, high = (grid[::-1], -grid) if neg_is_low else (-grid[::-1], grid)
            out[c].append(np.vstack([low, np.zeros(len(A)), high]).tolist())
    print('rudder left to the table, flaps 0, alpha 8, full travel: Cn %+.4f CY %+.4f, rows %s' % (out['Cn'][0][0][1], out['CY'][0][0][1], [round(x, 1) for x in out['rud']]))
    return out


def cruise_fit():
    """One propeller's roll. NASA's tables give it on J and blade pitch, and they go on past zero thrust: a windmilling propeller takes lift off its
    own wing, which the strips, fed only by thrust, don't. So the tables go in whole and the strips' own share, which goes with thrust over dynamic
    pressure, comes out. With the swirl across the radius the strips' own roll comes within 17 % of the CFD's (phase4_checks.py cruise), where it was
    0.7 of it: what the tables add now is mostly the windmilling. Nil while the two sides match"""
    import math
    import phase1_checks as K
    f = H.load('mod3')
    V, a, ks = 175.0, math.radians(8.0), []
    for lever in (0.5, 0.8):
        H.configure(f, 0, 0, 0, 2500.0, lever, stab=-5.0)
        c = []
        for on in (1, 0):
            f['ctl/motor-on-l'] = on
            H.state(f, V * math.cos(a), 0.0, V * math.sin(a), 0, 0, 0, 0, a, hp=6000.0)
            c.append((K.coef(f)['Cl'], f['prop/l/thrust-lbs'], f['aero/qbar-psf']))
        f['ctl/motor-on-l'] = 1
        ks.append((c[0][0] - c[1][0]) * c[0][2] / c[0][1])
    print('the strips\' own roll for one propeller, Cl per lbf per psf: ' + ' '.join('%.5f' % k for k in ks))
    return dict(k=float(np.mean(ks)))


if __name__ == '__main__':
    if sys.argv[1:] in (['hl'], ['ground'], ['ground_hl'], ['beta'], ['cruise'], ['rud']):          # a part alone
        d = json.load(open(B.FIT_FILE))
        k = sys.argv[1]
        d.pop(k, None)
        with open(B.FIT_FILE, 'w') as g:
            json.dump(d, g)
        d[k] = dict(hl=hl_moment, ground=ground_fit, ground_hl=ground_hl_fit, beta=beta_fit, cruise=cruise_fit, rud=rud_fit)[k]()
        with open(B.FIT_FILE, 'w') as g:
            json.dump(d, g)
    else:
        main()
