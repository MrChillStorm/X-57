#!/usr/bin/env python3
"""Phase 0 checks: is what we read from NASA's files what NASA meant?

    python3 phase0_checks.py

Each check compares the data with something independent of the reader: NASA's reference quantities, the physics of a
symmetric airplane, the sheets' own arithmetic, the definitions of the propeller coefficients, the TM's tables. Slips
in NASA's sheets are listed as findings and don't fail a check: known ones are in KNOWN, with what we do about them.
"""
import json, math, os, statistics as st, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, 'data')
load = lambda n: json.load(open(os.path.join(D, n + '.json') if n == 'geometry' else os.path.join(D, 'cfd', n + '.json')))
SOLVERS = ('star', 'lava', 'usm3d', 'kestrel')
HP_FTLB = 737.56            # the sheets' kW to ft-lbf/s
KESTREL_DELTA = ('kestrel', 58.0, 10.0)      # this block's delta columns came from another power-off run
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'FAIL', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def mass():
    """the hand-entered mass against what a body must satisfy, and the wing it hangs on"""
    import x57data as X
    m = X.MASS
    a, b, c = m['Ixx_lbin2'], m['Iyy_lbin2'], m['Izz_lbin2']
    check('inertias are physical: each no more than the sum of the others', a <= b + c and b <= a + c and c <= a + b)
    g = load('geometry')
    w = g['wing']
    p = w['panels'][0]
    cr, ct, sb = p['c0'], p['c1'], p['y1']
    lam = ct / cr
    ymac = sb / 3 * (1 + 2 * lam) / (1 + lam)
    # the panel's sweep is at its own location: the leading edge moves back by (sweep location) x (root less local chord)
    cm = cr + (ct - cr) * ymac / sb
    xle = w['origin_in'][0] + ymac * math.tan(math.radians(p['sweep'])) + p['loc'] * (cr - cm)
    cg = xle + m['cg_mac'] * w['mac_in']
    mrs = g['reference']['mrs_in']
    print('     mean chord leading edge x %.2f in, CG x %.2f in, moment reference x %.2f in (%.1f %% MAC)' % (xle, cg, mrs[0], 100 * (mrs[0] - xle) / w['mac_in']))
    check('the CG lies on the wing, close to the moment reference', abs(cg - mrs[0]) < 6, 'CG %.2f in ahead of it' % (mrs[0] - cg))
    zcg = g['reference']['wing_le_in'][2] - m['cg_below_wing_le_in']
    check('CG height close to the moment reference', abs(zcg - mrs[2]) < 4, 'CG z %.2f, reference z %.2f in' % (zcg, mrs[2]))
    print('     Ixx %.0f, Iyy %.0f, Izz %.0f slug ft2' % (a / X.SLUG_FT2, b / X.SLUG_FT2, c / X.SLUG_FT2))
    r = load('poweroff')['rows']
    u = lambda k: (min(x[k] for x in r), max(x[k] for x in r))
    s = X.SURFACES
    check('entered surface limits are the CFD\'s', u('stab') == s['stabilator'] and u('ail') == s['aileron'] and u('tab') == s['tab'] and u('rud')[0] == s['rudder'][0])


def geometry():
    g = load('geometry')
    w, R = g['wing'], g['reference']
    check('wing area', abs(w['area_in2'] / 144 - R['S_ft2']) < 0.005, '%.3f ft2' % (w['area_in2'] / 144))
    check('wing span', abs(w['span_in'] / 12 - R['b_ft']) < 0.002, '%.3f ft' % (w['span_in'] / 12))
    check('mean chord', abs(w['mac_in'] / 12 - R['c_ft']) < 0.005, '%.3f ft' % (w['mac_in'] / 12))
    check('aspect ratio 15 [TM table 1]', abs(w['aspect_ratio'] - 15) < 0.01, '%.2f' % w['aspect_ratio'])
    h, c = g['high_lift'], g['cruise']
    check('high-lift D 22.67 in, 5 blades, 27.4 deg [tunnel test table 1]',
          abs(h['diameter_in'] - 22.67) < 0.01 and h['blades'] == 5 and abs(h['beta34_deg'] - 27.4) < 0.1)
    check('cruise D 60 in (5 ft), 3 blades [CFD notes; MTV-7-152/64]', c['diameter_in'] == 60 and c['blades'] == 3)
    b = h['blade']
    check('high-lift blade angle at 0.75 R is beta34', abs(np.interp(0.75, b['r_R'], b['beta_deg']) - h['beta34_deg']) < 0.2,
          '%.2f deg' % np.interp(0.75, b['r_R'], b['beta_deg']))
    b = c['blade']
    check('cruise blade angle at 0.75 R is beta34', abs(np.interp(0.75, b['r_R'], b['beta_deg']) - c['beta34_deg']) < 0.2)
    s = g['sections']
    check('MH 114 thickness 13.02 %', abs(s['hl_blade']['thickness'] - 0.1302) < 0.002, '%.2f %%' % (100 * s['hl_blade']['thickness']))
    check('wing about 15 % thick [airfoil paper]', abs(s['wing_root']['thickness'] - 0.15) < 0.005, '%.2f %%' % (100 * s['wing_root']['thickness']))
    y = [p[1] for p in h['hubs_in']]
    disc = h['diameter_in']
    x = [p[0] for p in h['hubs_in']]
    gap = [math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]) - disc for i in range(len(y) - 1)]
    check('high-lift discs clear each other', min(gap) > 0, 'smallest gap %.3f in' % min(gap))


def symmetry():
    """an airplane mirrored: beta -> -beta keeps CL, CD, Cm and turns CY, Cl, Cn over"""
    rows = load('poweroff')['rows']
    key = lambda r: (r['alpha'], r['stab'], r['tab'], r['ail'], r['rud'], r['flap'])
    m = {}
    for r in rows:
        m.setdefault(key(r), {})[r['beta']] = r
    pairs = 0
    worst = {}
    for k, v in m.items():
        if k[3] or k[4]:                # a deflected aileron or rudder isn't symmetric
            continue
        for b in v:
            if b > 0 and -b in v:
                for s in SOLVERS:
                    p, q = v[b].get(s), v[-b].get(s)
                    if not p or not q:
                        continue
                    pairs += 1
                    for c, sign in (('CL', 1), ('CD', 1), ('Cm', 1), ('CY', -1), ('Cl', -1), ('Cn', -1)):
                        if c in p and c in q:
                            worst[c] = max(worst.get(c, 0), abs(p[c] - sign * q[c]))
    if pairs:
        check('sideslip mirrors', max(worst.values()) < 0.05, 'pairs %d, worst %s' % (pairs, {k: round(v, 4) for k, v in worst.items()}))
    else:
        print('     no clean +-beta pairs: the only negative sideslip has a control deflected, so the mirror is untested')


def signs():
    """a positive right aileron lowers its trailing edge: right wing up, roll left, Cl falls. Rudder trailing edge
    right (negative): the tail goes left, the nose right, Cn rises. Stabilator trailing edge down: nose down, Cm falls.
    Positive sideslip (wind from the right): a stable airplane turns into it, Cn rises, and it rolls away, Cl falls."""
    rows = load('poweroff')['rows']

    def base(r):
        return next((b for b in rows if b['alpha'] == r['alpha'] and b['flap'] == r['flap'] and b['beta'] == r['beta']
                     and not any(b[k] for k in ('stab', 'tab', 'ail', 'rud'))), None)

    def slopes(field, coef):
        out = []
        for r in rows:
            if r[field] and not any(r[k] for k in ('stab', 'tab', 'ail', 'rud') if k != field):
                b = base(r)
                if b and 'star' in r and 'star' in b and coef in r['star'] and coef in b['star']:
                    out.append((r['star'][coef] - b['star'][coef]) / r[field])
        return out

    a, s, d = slopes('ail', 'Cl'), slopes('stab', 'Cm'), slopes('rud', 'Cn')
    check('aileron: Cl falls with a positive right aileron', a and st.median(a) < 0, 'median %.5f /deg, n %d' % (st.median(a), len(a)))
    check('stabilator: Cm falls with trailing edge down', s and st.median(s) < 0, 'median %.4f /deg, n %d' % (st.median(s), len(s)))
    check('rudder: Cn falls as positive deflection grows', d and st.median(d) < 0, 'median %.5f /deg, n %d' % (st.median(d), len(d)))
    bn = [(r['star']['Cn'] / r['beta'], r['star']['Cl'] / r['beta']) for r in rows
          if r['beta'] > 0 and not any(r[k] for k in ('stab', 'tab', 'ail', 'rud')) and 'star' in r and 'Cn' in r['star']]
    check('weathercock stability: Cn rises with beta', st.median([b[0] for b in bn]) > 0, 'median %.5f /deg' % st.median([b[0] for b in bn]))
    check('dihedral effect: Cl falls with beta', st.median([b[1] for b in bn]) < 0, 'median %.5f /deg' % st.median([b[1] for b in bn]))
    cm = [(r['alpha'], r['star']['Cm']) for r in rows if r['flap'] == 0 and r['beta'] == 0 and r['alpha'] <= 12
          and not any(r[k] for k in ('stab', 'tab', 'ail', 'rud')) and 'star' in r]
    x, y = zip(*sorted(cm))
    check('pitch stability: Cm falls with alpha, flaps 0', np.polyfit(x, y, 1)[0] < 0, 'slope %.4f /deg' % np.polyfit(x, y, 1)[0])


def deltas():
    """the sheets' delta columns are power on minus power off"""
    rows = load('highlift')
    res, n = 0.0, 0
    bad, known = [], []
    for r in rows:
        for s in SOLVERS:
            x = r.get(s)
            if not x or 'delta' not in x:
                continue
            for c, v in x['delta'].items():
                if c in x.get('on', {}) and c in x.get('off', {}):
                    e = abs(x['on'][c] - x['off'][c] - v)
                    n += 1
                    if e <= 1e-3:
                        res = max(res, e)
                    elif (s, r['kcas'], r['flap']) == KESTREL_DELTA:
                        known.append(e)
                    else:
                        bad.append((r['kcas'], r['flap'], r['alpha'], r['beta'], s, c, round(e, 4)))
    check('Mod-IV deltas are on minus off', not bad, '%d values, worst %.5f' % (n, res) + ('; ' + str(bad[:6]) if bad else ''))
    print('     known slip: Kestrel, 58 KCAS, flaps 10: %d values differ, by up to %.3f. Use on minus off, not that column'
          % (len(known), max(known)))


def double_rows():
    """the rows that read '4,6': each solver's lift must now rise steadily through 2, 4, 6 and 8 deg"""
    rows = [r for r in load('highlift') if r['flap'] == 10 and r['beta'] == 0 and not any(r[k] for k in ('ail', 'stab', 'rud', 'tab'))]
    ok, n = True, 0
    for v in (58.0, 75.0):
        for s in SOLVERS:
            for state in ('on', 'off'):
                p = sorted((r['alpha'], r[s][state]['CL']) for r in rows if r['kcas'] == v and s in r and state in r[s]
                           and 'CL' in r[s][state] and 2 <= r['alpha'] <= 8)
                if len(p) >= 3:
                    n += 1
                    ok &= all(b[1] > a[1] for a, b in zip(p, p[1:]))
    star6 = [r for r in rows if r['kcas'] == 58 and r['alpha'] == 6]
    ok &= len(star6) == 1 and 'delta' not in star6[0]['star']
    check('rows reading 4,6 resolved: lift rises through them', ok, '%d curves' % n)


def prop_identities():
    """the definitions, on every row that states a propeller setting"""
    found = []

    def one(tag, D_ft, p, v, rho, torque_nm=False):
        n = p['rpm'] / 60
        tests = {'J': v / (n * D_ft)}
        if 'T_lbf' in p:
            tests['Ct'] = p['T_lbf'] / (rho * n * n * D_ft ** 4)
        if 'P_kW' in p:
            tests['Cp'] = p['P_kW'] * HP_FTLB / (rho * n ** 3 * D_ft ** 5)
        if 'Cp' in p and p.get('Cq') is not None:
            tests['Cq'] = p['Cp'] / (2 * math.pi)
        for k, want in tests.items():
            got = p.get(k)
            if got is None:
                continue
            if abs(got - want) > max(0.01 * abs(want), 0.002):
                found.append('%s %s reads %.4f, definition gives %.4f' % (tag, k, got, want))

    for r in load('highlift_settings'):
        one('high-lift %g KCAS' % r['kcas'], 1.89, r, r['v_fts'], r['rho'])
    seen = set()
    for r in load('cruise'):
        if r['prop'] and (r['kcas'], r['prop']['pitch'], r['prop']['rpm']) not in seen:
            seen.add((r['kcas'], r['prop']['pitch'], r['prop']['rpm']))
            one('cruise %g KCAS beta %g' % (r['kcas'], r['prop']['pitch']), 5.0, r['prop'], r['v_fts'], r['rho'])
    R = load('rates')
    seen = set()
    for r in R['cruise']:
        p = r['prop']
        if p and (r['kcas'], r['hp'], p['pitch']) not in seen:
            seen.add((r['kcas'], r['hp'], p['pitch']))
            one('rate cruise %g ft %g KCAS beta %g' % (r['hp'], r['kcas'], p['pitch']), 5.0, p, r['v_fts'], r['rho'])
    for r in R['high_lift']:
        p = r['prop']
        one('rate high-lift %g KCAS' % r['kcas'], 1.89, p, r['v_fts'], r['rho'])
    return found


def reynolds():
    """Re per foot goes with rho V at one altitude: the high-lift table's rows should agree"""
    rows = load('highlift_settings')
    k = [r['re_ft'] / r['v_fts'] for r in rows]
    med = st.median(k)
    return ['high-lift %g KCAS: Re/ft %g, expected %.0f' % (r['kcas'], r['re_ft'], med * r['v_fts'])
            for r, x in zip(rows, k) if abs(x / med - 1) > 0.02]


def tm_tables():
    """TM-20250001715 table 6 against the spreadsheet's settings"""
    tm = {35: (3545, .5490, .3125, .3134, 30.7, 4.7), 50: (4661, .5966, .3058, .3152, 52.0, 10.7), 58: (4702, .6860, .2848, .3075, 49.3, 10.7),
          65: (4379, .8250, .2473, .2874, 37.1, 8.1), 75: (3962, 1.0521, .1788, .2367, 22.0, 4.9), 90: (3428, 1.4595, .0366, .0809, 3.4, 1.1),
          100: (3451, 1.6098, -.0198, .0018, -1.8, 0.0)}
    ok = True
    for r in load('highlift_settings'):
        t = tm[int(r['kcas'])]
        got = (r['rpm'], r['J'], r['Ct'], r['Cp'], r['T_lbf'], r['P_kW'])
        ok &= all(abs(a - b) <= 0.06 * max(abs(b), 1e-3) + 0.051 * (i >= 4) + 0.0001 for i, (a, b) in enumerate(zip(got, t)))
    check('high-lift settings match TM table 6', ok)
    c = load('poweroff')['conditions']
    check('power-off conditions match TM table 2', [(x['alt_ft'], x['kcas'], x['flap']) for x in c] == [(2500, 88, 30), (2500, 94, 10), (8000, 133, 0)])


def digitised():
    """NASA's flying-qualities figures: the eigenvalue plots and the damping plots were read separately, so the damping
    worked out from an eigenvalue must match the damping plot at that speed"""
    f = json.load(open(os.path.join(D, 'digitised', 'flying_qualities.json')))
    want = dict(mod3=5, mod4_fixed=6, mod4_airspeed=4)
    for k, d in f['vs_kcas'].items():
        check('%s: every speed read' % k, all(len(d[s]) == n for s, n in want.items()), str({s: len(v) for s, v in d.items()}))
    worst, n = 0.0, 0
    for mode, zk in (('phugoid', 'zeta_phugoid'), ('short_period', 'zeta_short_period'), ('dutch_roll', 'zeta_dutch_roll')):
        for s, v in f['eigenvalues'][mode].items():
            for kcas, e in v.items():
                worst = max(worst, abs(e['zeta'] - f['vs_kcas'][zk][s][kcas]))
                n += 1
    print('     eigenvalues read: %s' % {m: {s: len(x) for s, x in v.items()} for m, v in f['eigenvalues'].items()})
    check('eigenvalues agree with the damping plots', worst < 0.012, '%d points, worst difference in damping ratio %.4f' % (n, worst))
    # Mod III, which needed no help from the speed grid to be found, is the cleaner test of the digitising
    m3 = [abs(e['zeta'] - f['vs_kcas'][zk]['mod3'][k]) for mode, zk in (('phugoid', 'zeta_phugoid'), ('short_period', 'zeta_short_period'),
          ('dutch_roll', 'zeta_dutch_roll')) for k, e in f['eigenvalues'][mode]['mod3'].items()]
    check('Mod III eigenvalues agree closely', max(m3) < 0.003, 'worst %.4f over %d points' % (max(m3), len(m3)))
    p = [2 * math.pi / e['im'] for e in f['eigenvalues']['phugoid']['mod3'].values()]
    check('phugoid period sensible with eigenvalues in rad/s, not Hz', 15 < min(p) and max(p) < 40, '%.0f to %.0f s' % (min(p), max(p)))


def coverage():
    rows = load('poweroff')['rows']
    print('\nCoverage')
    for f in (0, 10, 30):
        R = [r for r in rows if r['flap'] == f]
        u = lambda k: sorted({r[k] for r in R})
        print('  power off, flaps %2g: %3d rows, alpha %g..%g, beta %s, tab %s' % (f, len(R), u('alpha')[0], u('alpha')[-1], u('beta'), u('tab')))
    print('  other flaps in power off:', sorted({r['flap'] for r in rows} - {0, 10, 30}), len([r for r in rows if r['flap'] not in (0, 10, 30)]), 'rows')
    c = load('cruise')
    print('  cruise power: %d rows, flaps %s, %d baseline rows' % (len(c), sorted({r['flap'] for r in c}), sum(1 for r in c if r['prop'] is None)))
    h = load('highlift')
    for f in (10, 30):
        R = [r for r in h if r['flap'] == f]
        print('  high lift, flaps %g: %d rows, KCAS %s' % (f, len(R), sorted({r['kcas'] for r in R})))
    both = [r for r in h if 'star' in r and 'on' in r['star'] and 'off' not in r['star']]
    print('  high-lift rows with no power-off partner in STAR-CCM+:', len(both))


def spread():
    """how far the solvers disagree, power off, clean: the acceptance limits of the plan"""
    rows = [r for r in load('poweroff')['rows'] if r['beta'] == 0 and not any(r[k] for k in ('stab', 'tab', 'ail', 'rud'))]
    print('\nSolver spread, power off, beta 0 (max - min over the solvers at each alpha)')
    out = {}
    for f, lin in ((0, 12), (10, 10), (30, 9)):
        R = sorted([r for r in rows if r['flap'] == f], key=lambda r: r['alpha'])

        def sp(r, c):
            v = [r[s][c] for s in SOLVERS if s in r and c in r[s]]
            return max(v) - min(v) if len(v) > 1 else None
        res = {}
        for tag, sel in (('linear', lambda a: a <= lin), ('stall', lambda a: a > lin)):
            for c in ('CL', 'CD', 'Cm'):
                v = [x for x in (sp(r, c) for r in R if sel(r['alpha'])) if x is not None]
                res[tag, c] = (st.mean(v), max(v))
        print('  flaps %2g  linear CL %.3f (max %.3f)  Cm %.3f  CD %.4f | past stall CL %.3f (max %.3f)  Cm %.3f' % (
            f, *res['linear', 'CL'], res['linear', 'Cm'][0], res['linear', 'CD'][0], *res['stall', 'CL'], res['stall', 'Cm'][0]))
        peak = {s: max((r[s]['CL'], r['alpha']) for r in R if s in r and 'CL' in r[s]) for s in SOLVERS}
        print('           peak CL (at alpha):', {k: (round(v[0], 2), v[1]) for k, v in peak.items()})
        out[f] = peak
    h = [r for r in load('highlift') if r['beta'] == 0 and not any(r[k] for k in ('ail', 'stab', 'rud', 'tab'))]
    print('Mod IV, high lift on, beta 0')
    for f, v in ((30, 58), (10, 58), (30, 75)):
        R = sorted([r for r in h if r['flap'] == f and r['kcas'] == v], key=lambda r: r['alpha'])
        peak = {s: max((r[s]['on']['CL'], r['alpha']) for r in R if s in r and 'on' in r[s] and 'CL' in r[s]['on']) for s in SOLVERS
                if any(s in r and 'on' in r[s] for r in R)}
        print('  flaps %g, %g KCAS, peak CL:' % (f, v), {k: (round(x[0], 2), x[1]) for k, x in peak.items()})
    R = sorted([r for r in h if r['flap'] == 30 and r['kcas'] == 58], key=lambda r: r['alpha'])
    print('  STAR-CCM+ flaps 30, 58 KCAS, dCL:', ' '.join('%g:%.2f' % (r['alpha'], r['star']['delta']['CL']) for r in R if 'star' in r and 'delta' in r['star']))
    print('  STAR-CCM+ flaps 30, 58 KCAS, dCm:', ' '.join('%g:%+.2f' % (r['alpha'], r['star']['delta']['Cm']) for r in R if 'star' in r and 'delta' in r['star']))
    W, S = 3000.0, 66.667
    print('NASA stall speeds, CL they imply at 3,000 and 3,075 lb:')
    for tag, V in (('VS0 73 KCAS, flaps 30', 73), ('VS1 80 KCAS, flaps 10', 80), ('clean 88 KCAS', 88)):
        q = 0.5 * 0.0023769 * (V * 1.68781) ** 2
        print('  %-22s %.2f  %.2f' % (tag, W / (q * S), 3075 / (q * S)))


def semispan():
    """STAR-CCM+ is a half model and LAVA a whole one: their single-motor increments should still agree"""
    rows = load('cruise')
    base = {}
    for r in rows:
        if r['prop'] is None:
            base[r['hp'], r['kcas'], r['alpha'], r['beta'], r['ail'], r['flap']] = r
    d = []
    for r in rows:
        b = base.get((r['hp'], r['kcas'], r['alpha'], r['beta'], r['ail'], r['flap']))
        if r['prop'] and b and all(s in r and s in b for s in ('star', 'lava')) and r['kcas'] > 40:
            a, c = r['star']['CL'] - b['star']['CL'], r['lava']['CL'] - b['lava']['CL']
            if abs(c) > 0.02:
                d.append(a / c)
    check('one motor\'s dCL agrees between half and whole models', d and 0.8 < st.median(d) < 1.25,
          'median STAR/LAVA ratio %.2f, %d cases' % (st.median(d) if d else 0, len(d)))


if __name__ == '__main__':
    print('Geometry'); geometry()
    print('\nMass and limits'); mass()
    print('\nSigns and symmetry'); signs(); symmetry()
    print('\nThe sheets\' own arithmetic'); deltas(); double_rows(); semispan(); tm_tables()
    f = prop_identities() + reynolds()
    print('\nSlips in NASA\'s sheets (%d)' % len(f))
    for x in f:
        print('  ', x)
    print('\nDigitised flying qualities'); digitised()
    coverage()
    spread()
    print('\n%s' % ('ALL CHECKS PASS' if not fails else 'FAILED: ' + ', '.join(fails)))
    sys.exit(1 if fails else 0)
