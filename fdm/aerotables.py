#!/usr/bin/env python3
"""The table model's aerodynamics: NASA's CFD (data/cfd) into data/aero_tables.json.

    python3 aerotables.py

The structure is NASA's [TM fig. 68]: a power-off baseline, cruise propeller increments and high-lift increments.

Baseline. C(alpha, flap) with beta and every control neutral, plus increments for sideslip, stabilator, aileron and
rudder, each from the neutral case at the same alpha, within one solver and then averaged. So cross terms are left out
by construction, and phase1_checks.py measures what that costs against the CFD's combined cases.

Solvers. Up to stall the four agree (CL within 0.04-0.09) and are averaged, as NASA preferred [TM 6.1]. Past stall they
split into two families, and averaging them would draw a curve neither predicts. Kestrel and USM3D were run time
accurate, and a stalled wing's flow is unsteady, so past stall those two are used. The blend runs over the 2 deg after
STALL_ALPHA, where the families part. Kestrel and USM3D also stall earlier and lower, which goes the way of NASA's
stall speeds, as the trimmed comparison in phase1_checks.py shows.

Outside the CFD (alpha below -2 or above 24 deg) the values are estimates, a flat plate blended in: the model flies
there, but nothing here is NASA's.

Mirror images: sideslip was run positive, the rudder negative, and only the right aileron moved. CL, CD and Cm are even
under the mirror, CY, Cl and Cn odd; genx57.py applies that.
"""
import json, os
import numpy as np
from scipy.interpolate import RBFInterpolator

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, 'data')
load = lambda n: json.load(open(os.path.join(D, 'cfd', n + '.json')))
ALL = ('star', 'lava', 'usm3d', 'kestrel')
TIME_ACCURATE = ('usm3d', 'kestrel')
SIX = ('CD', 'CL', 'CY', 'Cl', 'Cm', 'Cn')
THREE = ('CD', 'CL', 'Cm')
FLAPS = (0.0, 10.0, 30.0)
RATE_SOURCE = os.environ.get('X57_RATES', 'tunnel')
STALL_ALPHA = {0.0: 15.0, 10.0: 12.0, 30.0: 12.0}     # where the solvers' two families part
BLEND_DEG = 2.0
CLEAN = ('stab', 'tab', 'ail', 'rud')
GRID = np.array([-90, -60, -40, -30, -24, -20, -16, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 11, 12, 13, 14, 15, 16, 17,
                 18, 19, 20, 22, 24, 27, 30, 35, 40, 50, 60, 90], float)


def neutral(r, but=()):
    return not any(r[k] for k in CLEAN if k not in but)


def mean_of(rows, coef, solvers=ALL):
    v = [r[s][coef] for r in rows for s in solvers if s in r and coef in r[s]]
    return float(np.mean(v)) if v else None


def flat_plate(a_deg):
    """lift and drag of a flat plate, the estimate far from the data"""
    a = np.radians(a_deg)
    cn = 1.15 * np.sin(a)
    return cn * np.cos(a), 0.05 + cn * np.sin(a)


def base_curves(rows):
    """{flap: {coef: values on GRID}} for CL, CD, Cm, and the raw alphas used"""
    out = {}
    for f in FLAPS:
        R = [r for r in rows if r['flap'] == f and r['beta'] == 0 and neutral(r)]
        al = sorted({r['alpha'] for r in R})
        cur = {}
        for c in THREE:
            everyone = np.array([mean_of([r for r in R if r['alpha'] == a], c) for a in al])
            late = [mean_of([r for r in R if r['alpha'] == a], c, TIME_ACCURATE) for a in al]
            # where they stop, the others carry on from the same offset, so the curve doesn't jump between families
            gap = 0.0
            for i, t in enumerate(late):
                if t is None:
                    late[i] = everyone[i] - gap
                else:
                    gap = everyone[i] - t
            late = np.array(late)
            w = np.clip((np.array(al) - STALL_ALPHA[f]) / BLEND_DEG, 0, 1)
            w = w * w * (3 - 2 * w)
            cur[c] = (1 - w) * everyone + w * late
        out[f] = extend(np.array(al), cur)
    return out


def extend(al, cur):
    """the CFD's curves onto GRID, with estimates beyond them"""
    cl, cd, cm = cur['CL'], cur['CD'], cur['Cm']
    res = {c: np.interp(GRID, al, cur[c]) for c in THREE}
    # below the data: the slope carries on to a negative stall near -12 deg, then the flat plate
    s_cl = (cl[1] - cl[0]) / (al[1] - al[0])
    s_cm = (cm[1] - cm[0]) / (al[1] - al[0])
    k = (cd[1] - cd[0]) / max(cl[1] ** 2 - cl[0] ** 2, 1e-6)
    cd0 = cd[0] - k * cl[0] ** 2
    for i, a in enumerate(GRID):
        if a < al[0]:
            lin = cl[0] + s_cl * (a - al[0])
            pl, pd = flat_plate(a)
            w = np.clip((-12 - a) / 8, 0, 1)
            res['CL'][i] = (1 - w) * lin + w * pl
            res['CD'][i] = (1 - w) * (cd0 + k * lin ** 2) + w * max(pd, cd0)
            res['Cm'][i] = cm[0] + s_cm * (max(a, -16) - al[0])
        elif a > al[-1]:
            pl, pd = flat_plate(a)
            w = np.clip((a - al[-1]) / 12, 0, 1)
            res['CL'][i] = (1 - w) * cl[-1] + w * pl
            res['CD'][i] = max((1 - w) * cd[-1] + w * pd, cd[-1])
            res['Cm'][i] = cm[-1] - 0.02 * (min(a, 45) - al[-1])
    return {c: v.tolist() for c, v in res.items()}


def increments(rows, field, coefs, values, alphas, extra=lambda r: True):
    """increment of each coefficient from the neutral case at that alpha and flap, within a solver, then averaged.
    Returns {coef: [flap][alpha node][value]}, with value 0 as zero. Nodes without data take their neighbours' values"""
    out = {c: np.zeros((len(FLAPS), len(alphas), len(values))) for c in coefs}
    for fi, f in enumerate(FLAPS):
        Rf = [r for r in rows if r['flap'] == f]
        base = {r['alpha']: r for r in Rf if r['beta'] == 0 and neutral(r)}
        have = sorted({r['alpha'] for r in Rf if r[field] and neutral(r, (field,)) and (field == 'beta' or r['beta'] == 0) and extra(r)})
        for c in coefs:
            for vi, val in enumerate(values):
                if val == 0:
                    continue
                pts = []
                for a in have:
                    d = []
                    for r in Rf:
                        if r['alpha'] != a or r[field] != val or not neutral(r, (field,)) or (field != 'beta' and r['beta'] != 0):
                            continue
                        for s in ALL:
                            if s in r and c in r[s]:
                                b = base.get(a, {}).get(s, {}).get(c)
                                if b is not None:
                                    d.append(r[s][c] - b)
                                elif c in ('CY', 'Cl', 'Cn'):
                                    d.append(r[s][c])        # zero at neutral, by symmetry; the sheets leave it blank
                    if d:
                        pts.append((a, float(np.mean(d))))
                if pts:
                    x, y = zip(*pts)
                    out[c][fi, :, vi] = np.interp(alphas, x, y)
            # a deflection that wasn't run at all: between the ones that were
            for ai in range(len(alphas)):
                row = out[c][fi, ai]
                good = [i for i, v in enumerate(values) if v == 0 or row[i] != 0]
                row[:] = np.interp(values, [values[i] for i in good], row[good])
    return {c: v.tolist() for c, v in out.items()}


def rates(R):
    """The damping derivatives on alpha and flap, forced oscillation, so Cmq here holds Cmq and Cm-alpha-dot together.

    RATE_SOURCE 'tunnel': the Langley tunnel's curves, measured flaps up, with the CFD's change between flap settings on
    top. 'cfd': the CFD alone. The two differ most in Cmq, about -140 against -195 near trim. NASA's published modes
    (figs. 18-24) fit the tunnel's values and not the CFD's: see phase1_checks.py."""
    alphas = [-5.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 18.0]
    out = {c: np.zeros((len(FLAPS), len(alphas))) for c in ('Cmq', 'Clp', 'Cnp', 'Clr', 'Cnr')}
    for fi, f in enumerate(FLAPS):
        for c in out:
            pts = {}
            for r in R:
                if r['flap'] != f:
                    continue
                v = [r[s][c] for s in ('star', 'lava', 'kestrel') if s in r and c in r[s]]
                if v:
                    pts.setdefault(r['alpha'], []).extend(v)
            x = sorted(pts)
            out[c][fi] = np.interp(alphas, x, [np.mean(pts[a]) for a in x])
    if RATE_SOURCE == 'tunnel':
        t = json.load(open(os.path.join(D, 'digitised', 'tunnel_rates.json')))
        for c in out:
            w = np.interp(alphas, t['alpha'], t[c])
            out[c][0] = w
            for fi in (1, 2):
                out[c][fi] = w + flap_change(R, c, FLAPS[fi], alphas)
    return dict(alpha=alphas, source=RATE_SOURCE, **{c: v.tolist() for c, v in out.items()})


def flap_change(R, c, f, alphas):
    """What a flap setting changes in a derivative, from the CFD: within one solver at one alpha, so that a solver's
    bias doesn't read as a flap effect, and as a straight line in alpha, since the solvers scatter more than the flaps
    move it. Below 10 deg: above that the oscillation reaches into the stall [TM 5.2.1]"""
    x, y = [], []
    for s in ('star', 'lava', 'kestrel'):
        cl = {r['alpha']: r[s][c] for r in R if r['flap'] == 0 and s in r and c in r[s]}
        for r in R:
            if r['flap'] == f and s in r and c in r[s] and r['alpha'] in cl and r['alpha'] <= 10:
                x.append(r['alpha'])
                y.append(r[s][c] - cl[r['alpha']])
    if len(x) < 3:
        return np.zeros(len(alphas))
    a = np.polyfit(x, y, 1)
    return np.polyval(a, np.clip(alphas, min(x), max(x)))


def cruise_tables():
    """Cruise propeller: Ct and Cp on J and blade pitch, as NASA's blade-element model gave them to the CFD, and each
    coefficient's increment for one motor, on the right. Their points lie along the operating band; the surfaces are
    smooth, so a thin plate spline fills the grid. Off the band the values are extrapolations, and never used in flight"""
    rows = load('cruise')
    base = {(r['hp'], r['kcas'], r['alpha'], r['beta'], r['ail'], r['flap']): r for r in rows if r['prop'] is None}
    pts = {}
    for r in rows:
        p = r['prop']
        if not p or r['alpha'] != 2 or r['beta'] or r['ail'] or r['flap']:
            continue
        k = (round(p['J'], 3), p['pitch'])
        e = pts.setdefault(k, dict(Ct=p['Ct'], Cp=p['Cp'], d={c: [] for c in SIX}))
        b = base.get((r['hp'], r['kcas'], r['alpha'], r['beta'], r['ail'], r['flap']))
        for s in ('star', 'lava', 'usm3d'):
            if b and s in r and s in b:
                for c in SIX:
                    if c in r[s] and c in b[s]:
                        e['d'][c].append(r[s][c] - b[s][c])
    keys = sorted(pts)
    X = np.array(keys, float)
    Js = [0.0, 0.1, 0.25, 0.4, 0.6, 0.8, 1.0, 1.2, 1.3, 1.4, 1.6, 1.8, 2.0]
    Bs = [10.0, 14.0, 18.0, 22.0, 26.0, 30.0, 34.0, 38.0, 42.0, 46.0]
    G = np.array([(j, b) for j in Js for b in Bs])
    scale = np.array([1.0, 30.0])

    def surf(y):
        return RBFInterpolator(X / scale, y, kernel='thin_plate_spline', smoothing=1e-4)(G / scale).reshape(len(Js), len(Bs))

    ct = surf(np.array([pts[k]['Ct'] for k in keys]))
    cp = surf(np.array([pts[k]['Cp'] for k in keys]))
    # blade pitch and thrust from J and power: each J's pitch curve inverted, for the motor's torque to set the pitch
    Cps = [-0.15, -0.1, -0.05, -0.02, 0.0, 0.02, 0.05, 0.08, 0.11, 0.14, 0.17, 0.2, 0.24, 0.28, 0.32]
    pitch_of = np.zeros((len(Js), len(Cps)))
    ct_of = np.zeros((len(Js), len(Cps)))
    bf = np.linspace(Bs[0], Bs[-1], 400)
    for i in range(len(Js)):
        cf = np.interp(bf, Bs, cp[i])
        tf = np.interp(bf, Bs, ct[i])
        o = np.argsort(cf)
        pitch_of[i] = np.interp(Cps, cf[o], bf[o])
        ct_of[i] = np.interp(Cps, cf[o], tf[o])
    out = dict(J=Js, pitch=Bs, Ct=ct.tolist(), Cp=cp.tolist(), Cp_axis=Cps, pitch_of=pitch_of.tolist(), ct_of=ct_of.tolist(),
               points=len(keys))
    # increments grow as T/q, so as Ct/J^2: at J = 0 the coefficient is unbounded while q is nil. The table stops at the
    # J = 0.1 of the lowest CFD case, whose value holds below it
    for c in SIX:
        y = np.array([np.mean(pts[k]['d'][c]) if pts[k]['d'][c] else np.nan for k in keys])
        ok = np.isfinite(y)
        f = RBFInterpolator(X[ok] / scale, y[ok], kernel='thin_plate_spline', smoothing=1e-3)
        d = f(G / scale).reshape(len(Js), len(Bs))
        d[0] = d[1]
        out['d' + c] = d.tolist()
    return out


def highlift_tables(po):
    """High lift: increment of each coefficient against J, with all twelve running on the schedule, power on minus
    power off within a solver, then averaged. NASA's cases follow the schedule at 2,500 ft, so J stands for the speed. Fixed
    pitch propellers have one Ct for each J, so the increments hold across speeds at the same J [20240006496]."""
    rows = [r for r in load('highlift') if r['alpha'] is not None]
    sets = load('highlift_settings')
    Js = [s['J'] for s in sets]
    kc = [s['kcas'] for s in sets]
    alphas = [-2.0, 2.0, 4.0, 6.0, 8.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0]
    flaps = (10.0, 30.0)

    def delta(r, c):
        v = []
        for s in ALL:
            x = r.get(s)
            if x and 'on' in x and 'off' in x and c in x['on'] and c in x['off']:
                v.append(x['on'][c] - x['off'][c])
        return float(np.mean(v)) if v else None

    def clean(r):
        return not any(r[k] for k in ('ail', 'stab', 'rud', 'tab'))

    out = dict(J=Js, kcas=kc, alpha=alphas, flaps=flaps, Ct=[s['Ct'] for s in sets], Cp=[s['Cp'] for s in sets])
    for c in THREE:
        T = np.zeros((len(flaps), len(Js), len(alphas)))
        for fi, f in enumerate(flaps):
            for ji, k in enumerate(kc):
                R = [r for r in rows if r['flap'] == f and r['kcas'] == k and r['beta'] == 0 and clean(r)]
                p = sorted((r['alpha'], delta(r, c)) for r in R if delta(r, c) is not None)
                x, y = zip(*p)
                T[fi, ji] = np.interp(alphas, x, y)
        out['d' + c] = T.tolist()
    # sideslip, and the controls' change of effect in the propellers' wash, at the speeds with a full set of cases
    full = (58.0, 75.0, 90.0)
    out['full_J'] = [Js[kc.index(k)] for k in full]

    def on(r, s, c):
        x = r.get(s, {}).get('on', {})
        return x.get(c)

    def extra(field, values, coefs, nodes):
        """A control or sideslip with the propellers running: its increment from the neutral case, power on, within a
        solver. Stored less the same input's power-off increment, which the baseline already adds. Worked from the powered
        runs alone, because many of them have no power-off partner"""
        res = {c: np.zeros((len(flaps), len(full), len(nodes), len(values))) for c in coefs}
        pb = po[field]
        for fi, f in enumerate(flaps):
            pf = po['flaps'].index(f)
            for ki, k in enumerate(full):
                Rk = [r for r in rows if r['flap'] == f and r['kcas'] == k]
                b = {r['alpha']: r for r in Rk if r['beta'] == 0 and clean(r)}
                for c in coefs:
                    odd = c in ('CY', 'Cl', 'Cn')
                    for vi, val in enumerate(values):
                        if val == 0:
                            continue
                        pts = []
                        for a in sorted({r['alpha'] for r in Rk}):
                            d = []
                            for r in Rk:
                                others = [q for q in ('beta', 'ail', 'stab', 'rud', 'tab') if q != field]
                                if r['alpha'] != a or r[field] != val or any(r[q] for q in others):
                                    continue
                                for s in ALL:
                                    x = on(r, s, c)
                                    y = on(b.get(a, {}), s, c)
                                    if x is None:
                                        continue
                                    if y is not None:
                                        d.append(x - y)
                                    elif odd:
                                        d.append(x)
                            if d:
                                # the power-off increment at this alpha, from the baseline's table
                                node = po['nodes']
                                vals = po[field]['values']
                                col = [float(np.interp(a, node, [pb[c][pf][i][j] for i in range(len(node))])) for j in range(len(vals))] if c in pb else None
                                base = float(np.interp(val, vals, col)) if col else 0.0
                                pts.append((a, float(np.mean(d)) - base))
                        if pts:
                            x, y = zip(*pts)
                            res[c][fi, ki, :, vi] = np.interp(nodes, x, y)
                    for ai in range(len(nodes)):
                        row = res[c][fi, ki, ai]
                        good = [i for i, v in enumerate(values) if v == 0 or row[i] != 0]
                        row[:] = np.interp(values, [values[i] for i in good], row[good])
        return {c: v.tolist() for c, v in res.items()}

    nodes = [2.0, 8.0, 12.0, 15.0]
    out['nodes'] = nodes
    out['beta'] = dict(values=[0.0, 5.0, 10.0, 15.0, 20.0], **extra('beta', [0.0, 5.0, 10.0, 15.0, 20.0], SIX, nodes))
    out['stab'] = dict(values=[-15.0, -12.0, -5.0, 0.0, 5.0], **extra('stab', [-15.0, -12.0, -5.0, 0.0, 5.0], THREE, nodes))
    out['ail'] = dict(values=[-25.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 18.0],
                      **extra('ail', [-25.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 18.0], SIX, nodes))
    out['rud'] = dict(values=[-28.0, -18.0, -10.0, -5.0, 0.0], **extra('rud', [-28.0, -18.0, -10.0, -5.0, 0.0], SIX, nodes))
    R = load('rates')
    # the rate derivatives with high lift on, at 58 and 75 KCAS: their change from power off at the same alpha and flap
    out['rates'] = dict(J=[0.686, 1.052, 1.61], alpha=[-2.0, 0.0, 4.0, 8.0, 10.0, 12.0], flaps=flaps)
    off, on = R['power_off'], R['high_lift']
    for c in ('Cmq', 'Clp', 'Cnp', 'Clr', 'Cnr'):
        T = np.zeros((len(flaps), 3, 6))
        for fi, f in enumerate(flaps):
            for ji, k in enumerate((58.0, 75.0)):
                pts = {}
                for r in on:
                    if r['flap'] == f and r['kcas'] == k:
                        v = [r[s][c] for s in ('star', 'kestrel') if s in r and c in r[s]]
                        if v:
                            pts[r['alpha']] = float(np.mean(v))
                base = {}
                for r in off:
                    if r['flap'] == f:
                        v = [r[s][c] for s in ('star', 'lava', 'kestrel') if s in r and c in r[s]]
                        if v:
                            base.setdefault(r['alpha'], []).extend(v)
                bx = sorted(base)
                by = [float(np.mean(base[a])) for a in bx]
                if pts:
                    x = sorted(pts)
                    T[fi, ji] = np.interp(out['rates']['alpha'], x, [pts[a] - np.interp(a, bx, by) for a in x])
        out['rates'][c] = T.tolist()
    return out, R


def build():
    rows = load('poweroff')['rows']
    rows = [r for r in rows if r['flap'] in FLAPS]
    b = base_curves(rows)
    nodes = [2.0, 8.0, 12.0, 14.0, 15.0, 16.0]
    bet = [0.0, 5.0, 10.0, 15.0, 20.0]
    stb = [-15.0, -12.0, -5.0, 0.0, 5.0]
    ail = [-25.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 18.0]
    rud = [-28.0, -18.0, -10.0, -5.0, 0.0]
    t = dict(alpha=GRID.tolist(), flaps=FLAPS, base={c: [b[f][c] for f in FLAPS] for c in THREE}, nodes=nodes,
             beta=dict(values=bet, **increments(rows, 'beta', SIX, bet, nodes)),
             stab=dict(values=stb, **increments(rows, 'stab', THREE, stb, nodes)),
             ail=dict(values=ail, **increments(rows, 'ail', SIX, ail, nodes)),
             rud=dict(values=rud, **increments(rows, 'rud', SIX, rud, nodes)))
    t['highlift'], R = highlift_tables(t)
    t.update(rates=rates(R['power_off']), cruise=cruise_tables())
    return t


if __name__ == '__main__':
    t = build()
    with open(os.path.join(D, 'aero_tables.json'), 'w') as f:
        json.dump(t, f, separators=(',', ':'))
    a = np.array(t['alpha'])
    for i, fl in enumerate(FLAPS):
        cl = np.array(t['base']['CL'][i])
        m = (a >= -2) & (a <= 24)
        print('flaps %2g: CL peak %.2f at %g deg; CL0 %.3f, CLa %.4f /deg; CD at CL 0.75 %.4f; Cm(0) %.3f' % (
            fl, cl[m].max(), a[m][cl[m].argmax()], np.interp(0, a, cl), (np.interp(8, a, cl) - np.interp(0, a, cl)) / 8,
            np.interp(0.75, cl[(a > -3) & (a < 12)], np.array(t['base']['CD'][i])[(a > -3) & (a < 12)]), np.interp(0, a, t['base']['Cm'][i])))
    b = t['beta']
    print('beta 5 at alpha 2, flaps 0: CY %.4f Cl %.4f Cn %.4f' % tuple(b[c][0][0][1] for c in ('CY', 'Cl', 'Cn')))
    s = t['stab']
    print('stab -5 at alpha 2, flaps 0: dCm %.3f dCL %.3f' % (s['Cm'][0][0][2], s['CL'][0][0][2]))
    al = t['ail']
    print('right aileron -15 at alpha 2, flaps 0: dCl %.4f dCn %.4f dCL %.4f' % (al['Cl'][0][0][1], al['Cn'][0][0][1], al['CL'][0][0][1]))
    r = t['rud']
    print('rudder -18 at alpha 2, flaps 0: dCn %.4f dCY %.4f dCl %.4f' % (r['Cn'][0][0][1], r['CY'][0][0][1], r['Cl'][0][0][1]))
    q = t['rates']
    print('rates alpha', q['alpha'])
    for c in ('Cmq', 'Clp', 'Cnp', 'Clr', 'Cnr'):
        print('  %s' % c, [round(x, 2) for x in q[c][1]])
    c = t['cruise']
    print('cruise: %d points; at J 1.3, pitch 30: Ct %.3f Cp %.3f dCD %.4f dCL %.3f dCm %.3f (TM: dCD about -0.0014 at J 1.35)' % (
        c['points'], c['Ct'][8][5], c['Cp'][8][5], c['dCD'][8][5], c['dCL'][8][5], c['dCm'][8][5]))
    h = t['highlift']
    print('high lift J', [round(x, 3) for x in h['J']])
    print(' flaps 30, J 0.686, dCL', [round(x, 2) for x in h['dCL'][1][2]])
    print(' flaps 30, J 0.686, dCm', [round(x, 2) for x in h['dCm'][1][2]])
    print('file size', os.path.getsize(os.path.join(D, 'aero_tables.json')))
