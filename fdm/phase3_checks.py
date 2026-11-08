#!/usr/bin/env python3
"""Phase 3: the built-up method on the P2006T, against its flight manual, its flight tests and its wind tunnel. Limits in docs/PLAN.md.

    python3 phase3_checks.py [tunnel modes derivatives trims sideslip cruise climb stall]        all of them by default
"""
import math, sys
import numpy as np
from scipy.optimize import brentq
import harness as H
import p2006t as P
import p3sim as X

G = 9.80665
KT = 1.6878099
fails = []


def check(name, val, ref, tol, fmt='%.3f', rel=False):
    """within tol of ref, or of a range (lo, hi)"""
    lo, hi = (ref, ref) if not isinstance(ref, tuple) else (min(ref), max(ref))
    t = tol * max(abs(lo), abs(hi)) if rel else tol
    ok = lo - t <= val <= hi + t
    if not ok:
        fails.append(name)
    r = (fmt % ref) if not isinstance(ref, tuple) else ('%s to %s' % (fmt % ref[0], fmt % ref[1]))
    print('  %-46s %10s   target %-18s %s' % (name, fmt % val, r, 'ok' if ok else 'MISS (limit %s)' % (('%.0f %%' % (100 * tol)) if rel else tol)))


def sweep(f, alphas, stab=0.0):
    t = dict(flap=0, gear=0, stab=stab, lever=0, run=(1, 1), V=120.0, hp=0.0, alpha=0.0, gamma=0.0)
    return np.array([[a] + [X.at(f, t, alpha=a)[k] for k in ('CL', 'CD', 'Cm')] for a in alphas])


def tunnel():
    print('\nWind tunnel, Re 0.6 million, one part at a time [ICAS figs. 14, 16, 18; AIAA fig. 4]. Moments about 25 % of the chord')
    A = [-2, 0, 2, 4, 6, 8]
    f = X.load(kg=1140, cg=0.25, tunnel=True, tail=False, nacelles=False)
    wb = sweep(f, A, -5)
    f = X.load(kg=1140, cg=0.25, tunnel=True, tail=False, nacelles=True)
    wn = sweep(f, A, -5)
    f = X.load(kg=1140, cg=0.25, tunnel=True)
    s = {d: sweep(f, A, d) for d in (0.0, -1.0, -3.5, -6.0)}
    ac = lambda r: 25 - 100 * np.polyfit(r[:, 1], r[:, 3], 1)[0]
    sl = lambda r: np.polyfit(r[:, 0], r[:, 1], 1)[0]
    check('wing-body lift slope, /deg', sl(wb), 0.080, 0.08, '%.4f', True)
    check('wing-body lift at alpha 0 (fitted incidence)', np.interp(0, wb[:, 0], wb[:, 1]), 0.13, 0.03)
    check('wing-body aerodynamic centre, % chord', ac(wb), 14.0, 4.0, '%.1f')
    check('nacelles move it forward, % chord', ac(wb) - ac(wn), 3.0, 2.0, '%.1f')
    check('nacelles\' lift (their measured loss)', np.interp(4, wn[:, 0], wn[:, 1]) - np.interp(4, wb[:, 0], wb[:, 1]), -0.048, 0.01)
    check('wing-body Cm at CL 0', np.interp(0, wb[:, 1], wb[:, 3]), -0.08, 0.02)
    full = s[0.0]
    print('  fixed stabilator, whole aircraft: lift slope %.4f/deg; neutral point, CL 0.3: %.1f %%' % (sl(full), ac(full)))
    check('whole aircraft neutral point, % chord', ac(full), 41.0, 4.0, '%.1f')
    d = (s[-6.0][:, 3] - s[-1.0][:, 3]) / np.radians(-5.0)
    check('Cm delta, fig. 4, /rad', float(np.mean(d)), -1.95, 0.10, '%.2f', True)
    print('  Cm at delta 0 by alpha, fig. 4 reads -0.055 at 0, -0.115 at 4, -0.195 at 8:', np.round(full[[1, 3, 5], 3], 3))
    t = dict(flap=0, gear=0, stab=0.0, lever=0, run=(1, 1), V=120.0, hp=0.0, alpha=4.0, gamma=0.0)
    b0, b1 = X.at(f, t, beta=-4.0), X.at(f, t, beta=4.0)
    h = math.radians(8.0)
    check('Cn beta, tunnel, /rad', (b1['Cn'] - b0['Cn']) / h, 0.05, 0.30, '%.4f', True)
    check('Cl beta, tunnel, /rad', (b1['Cl'] - b0['Cl']) / h, -0.0573, 0.30, '%.4f', True)
    check('CY beta, tunnel, /rad', (b1['CY'] - b0['CY']) / h, -0.688, 0.20, '%.3f', True)
    r0, r1 = X.at(f, t, rud=-0.3), X.at(f, t, rud=0.3)
    check('Cn delta r, tunnel, /rad', (r1['Cn'] - r0['Cn']) / math.radians(2 * 0.3 * P.SURFACES['rudder']), -0.0631, 0.20, '%.4f', True)


def flight_build():  # the identification's condition
    return X.load(kg=P.TEST['kg'], cg=P.TEST['cg_mac'])


def modes():
    print('\nModes in flight at 110 KIAS, 1140 kg, CG 19 % [AIAA tables 6-8]. These are the times seen on the aircraft')
    f = flight_build()
    t = X.trim(f, P.TEST['kias'], hp=P.TEST['hp'])
    m = X.modes(f, t)
    ph, sp, dr = m['phugoid'], m['short_period'], m['dutch_roll']
    check('phugoid period, s', 2 * math.pi / ph.imag, 27.0, 0.15, '%.1f', True)
    check('phugoid damping', X.zeta(ph), 0.09, 0.04)
    check('short period, rad/s', abs(sp), 3.41, 0.15, '%.2f', True)
    check('short period damping', X.zeta(sp), 0.40, 0.10)
    check('Dutch roll period, s', 2 * math.pi / dr.imag, 3.25, 0.15, '%.2f', True)
    check('Dutch roll damping', X.zeta(dr), 0.26, 0.08)
    print('  the roots left: %s. The paper\'s own table 9 gives a short period of 4.95 rad/s, damping 0.54' % np.round(m['real'], 3))


def derivatives():
    print('\nDerivatives at the same condition [AIAA tables 9, 10]')
    f = flight_build()
    t = X.trim(f, P.TEST['kias'], hp=P.TEST['hp'])
    d = X.derivatives(f, t)
    check('CL alpha, /rad', d['CLa'], (4.152, 4.5), 0.10, '%.3f', True)
    check('Cm alpha, /rad', d['Cma'], (-0.80, -0.871), 0.15, '%.3f', True)
    check('Cm delta, /rad', d['Cmde'], -1.811, 0.10, '%.3f', True)
    check('Cm q, /rad', d['Cmq'], (-14.8, -19.05), 0.15, '%.2f', True)
    check('Cn r, /rad', d['Cnr'], -0.106, 0.25, '%.3f', True)
    print('  with winglets and power, against the tunnel\'s without (checked like for like above): CY beta %.3f, Cn beta %.4f, Cl beta %.4f, Cn delta r %.4f' % (d['CYb'], d['Cnb'], d['Clb'], d['Cndr']))
    print('  printed, flight / handbook: Cl p %.3f (-0.281 / -0.529), Cl r %.3f (0.057), Cn p %.3f (-0.130), Cl delta a %.3f (0.045 / 0.057), Cl delta r %.4f (0.0009 / 0.014), CY delta r %.3f (0.04 / 0.173)' % (
        d['Clp'], d['Clr'], d['Cnp'], d['Clda'], d['Cldr'], d['CYdr']))
    print('  neutral point from the two slopes: %.1f %%; downwash lag Cm alpha-dot %.2f' % (100 * (P.TEST['cg_mac'] - d['Cma'] / d['CLa']), d['Cmad']))


def trims():
    print('\nStabilator to trim level flight [AIAA fig. 11a], 1100 kg assumed, 2000 ft')
    F16 = [(.22, -4.0), (.29, -4.5), (.345, -4.9), (.48, -6.3), (.555, -7.0), (.75, -9.0), (.925, -10.9)]
    F30 = [(.335, -3.5), (.48, -4.0), (.735, -5.4), (.945, -6.9)]
    worst = {}
    slopes = {}
    for cg, pts in ((0.165, F16), (0.305, F30)):
        f = X.load(kg=1100.0, cg=cg)
        cl, st = [], []
        for k in (68, 72, 78, 85, 95, 110, 130, 150):
            t = X.trim(f, k, hp=2000.0)
            cl.append(2 * 1100 * G / (1.1548 * (t['V'] * 0.3048) ** 2 * 14.76))
            st.append(t['stab'])
        o = np.argsort(cl)
        cl, st = np.array(cl)[o], np.array(st)[o]
        e = [np.interp(c, cl, st) - s for c, s in pts]
        print('  CG %.1f %%: model at the paper\'s CLs %s, paper %s' % (100 * cg, np.round([np.interp(c, cl, st) for c, _ in pts], 1), [s for _, s in pts]))
        check('worst stabilator error, CG %.1f %%, deg' % (100 * cg), float(np.abs(e).max()), 0.0, 1.0, '%.2f')
        for c in (0.25, 0.5, 0.75):
            slopes.setdefault(c, []).append((cg, (np.interp(c + 0.05, cl, st) - np.interp(c - 0.05, cl, st)) / 0.1))
    npts = {}
    for c, ((a, sa), (b, sb)) in slopes.items():
        npts[c] = a + (0 - sa) * (b - a) / (sb - sa)
    print('  neutral point at CL 0.25, 0.5, 0.75: %.1f, %.1f, %.1f %% (flight: 37, 43.5, 52.5)' % tuple(100 * npts[c] for c in (0.25, 0.5, 0.75)))
    check('neutral point at CL 0.5, % chord', 100 * npts[0.5], (41.0, 43.0), 4.0, '%.1f')
    if not npts[0.25] < npts[0.5] < npts[0.75]:
        fails.append('neutral point moves aft with CL')
        print('  MISS: neutral point must move aft with CL')


def sideslip():
    print('\nSideslip at 100 kt, the wings near level as fig. 12 shows them, so these are flat skids and only the moments balance [AIAA fig. 13]')
    from scipy.optimize import least_squares
    f = X.load(kg=1100.0, cg=0.25)
    t = X.trim(f, 100, hp=2000.0)
    V = t['V']
    cuts = {}
    for sign, last in ((1, 25.0), (-1, 15.0)):
        x = [t['alpha'], t['stab'], 0.0, 0.0]
        for beta in np.arange(5.0, last + 0.1, 5.0) * sign:
            def res(y):
                al, st, ail, rud = y
                X.configure(f, 0, 0, st, (t['lever'], t['lever']), ail, rud)
                a, b = math.radians(al), math.radians(beta)
                acc = H.state(f, V * math.cos(a) * math.cos(b), V * math.sin(b), V * math.sin(a) * math.cos(b), theta=a, hp=2000.0)
                return [-acc[0] * math.sin(a) + acc[2] * math.cos(a), acc[3] * 5, acc[4] * 5, acc[5] * 5]
            s_ = least_squares(res, x, x_scale=[1, 1, .2, .2])
            x = list(s_.x)
            res(x)
            cuts[float(beta)] = (f['ctl/rud-deg'], 0.5 * (f['ctl/ail-right-deg'] - f['ctl/ail-left-deg']), float(np.abs(s_.fun).max()))
    print('  rudder, aileron by sideslip: ' + ', '.join('%+.0f: %+.1f, %+.1f' % (b, cuts[b][0], cuts[b][1]) for b in sorted(cuts) if cuts[b][2] < 1e-3))
    # the paper's points, -15 to +15 deg: rudder -15.5 to +3.5, aileron -12.8 to +4.5, each about an offset from the propellers
    check('rudder per degree, -15 to +15 deg', abs(cuts[15.0][0] - cuts[-15.0][0]) / 30, 0.65, 0.25, '%.2f', True)
    check('aileron per degree, -15 to +15 deg', abs(cuts[15.0][1] - cuts[-15.0][1]) / 30, 0.58, 0.25, '%.2f', True)
    print('  past +20 the fin\'s section stalls and the rudder stops helping, where the airplane held +25 on some 16 deg. The vane may read high here too; the offsets at zero sideslip aren\'t modelled')


def modii():
    """A second source for the modes: NASA's Mod II simulator, built on its own identification of an instrumented P2006T.
    Figures read by eye [20230004174 figs. 15-22]. Same airframe and mass, but electric nacelles and other propellers."""
    print('\nNASA\'s Mod II simulator, cruise, at its mass [20230004174]: 2835.5 lb, CG 24.4 %, inertias as printed')
    lbin2 = 0.45359237 * 0.0254 ** 2
    keep = dict(P.MASS)
    P.MASS.update(Ixx=6.29101e6 * lbin2, Iyy=8.51692e6 * lbin2, Izz=1.30684e7 * lbin2, cg_below_chord=22.7 / 12 / P.REF['c_ft'])
    f = X.load(kg=2835.5 * 0.45359237, cg=0.244)
    P.MASS.update(keep)
    sp_w = dict(zip((90, 100, 110, 120, 130), (3.7, 4.1, 4.4, 4.8, 5.2)))
    ph_z = dict(zip((90, 100, 110, 120, 130), (0.063, 0.076, 0.086, 0.104, 0.118)))
    ph_w = dict(zip((90, 100, 110, 120, 130), (0.225, 0.205, 0.185, 0.170, 0.155)))
    dr_z = dict(zip((90, 100, 110, 120, 130), (0.275, 0.29, 0.305, 0.31, 0.31)))
    dr_w = dict(zip((90, 100, 110, 120, 130), (1.09, 1.18, 1.27, 1.37, 1.47)))
    roll = dict(zip((90, 100, 110, 120, 130), (0.88, 0.83, 0.78, 0.73, 0.68)))
    for v in (90, 100, 110, 120, 130):
        t = X.trim(f, v, hp=6000.0)
        m = X.modes(f, t)
        ph, sp, dr = m['phugoid'], m['short_period'], m['dutch_roll']
        tr = min(m['real'])
        print('  %3d KCAS: short %.2f (%.1f), %.3f (0.59); phugoid %.3f (%.3f), %.3f (%.3f); Dutch %.2f (%.2f), %.3f (%.3f); roll %.2f s (%.2f); alpha %.1f' % (
            v, abs(sp), sp_w[v], X.zeta(sp), abs(ph), ph_w[v], X.zeta(ph), ph_z[v], abs(dr), dr_w[v], X.zeta(dr), dr_z[v], -1 / tr, roll[v], t['alpha']))
        if v in (90, 110, 130):
            check('%d KCAS short period, rad/s' % v, abs(sp), sp_w[v], 0.15, '%.2f', True)
            check('%d KCAS short period damping' % v, X.zeta(sp), 0.59, 0.08)
            check('%d KCAS phugoid, rad/s' % v, abs(ph), ph_w[v], 0.15, '%.3f', True)
            check('%d KCAS phugoid damping' % v, X.zeta(ph), ph_z[v], 0.03)
            check('%d KCAS Dutch roll, rad/s' % v, abs(dr), dr_w[v], 0.15, '%.2f', True)
            check('%d KCAS Dutch roll damping' % v, X.zeta(dr), dr_z[v], 0.06)
            check('%d KCAS roll time constant, s' % v, -1 / tr, roll[v], 0.15, '%.2f', True)
    # NASA's figs. 11 and 12 together: the same five cruise points give alpha and CL, 5.09/rad through them, CL 0.11 at alpha 0
    w_n = 2835.5 * 4.4482216
    pts = [(X.trim(f, v, hp=6000.0)['alpha'], w_n / (0.5 * 1.225 * (v * 0.514444) ** 2 * P.REF['S_ft2'] * 0.3048 ** 2)) for v in (80, 100, 120)]
    slope = math.degrees((pts[0][1] - pts[-1][1]) / (pts[0][0] - pts[-1][0]))
    print('  trimmed CL by alpha: ' + ', '.join('%.2f at %.1f (NASA %.1f)' % (c, a, (c - 0.11) / 0.0889) for a, c in pts))
    check('trimmed lift slope, /rad', slope, 5.09, 0.10, '%.2f', True)
    from scipy.optimize import least_squares
    t = X.trim(f, 100, hp=6000.0)
    V = t['V']
    cuts = {}
    for sign in (1, -1):
        x = [t['alpha'], t['stab'], 0.0, 0.0]
        for beta in np.arange(5.0, 15.1, 5.0) * sign:
            def res(y):
                al, st, ail, rud = y
                X.configure(f, 0, 0, st, (t['lever'], t['lever']), ail, rud)
                a, b = math.radians(al), math.radians(beta)
                acc = H.state(f, V * math.cos(a) * math.cos(b), V * math.sin(b), V * math.sin(a) * math.cos(b), theta=a, hp=6000.0)
                return [-acc[0] * math.sin(a) + acc[2] * math.cos(a), acc[3] * 5, acc[4] * 5, acc[5] * 5]
            x = list(least_squares(res, x, x_scale=[1, 1, .2, .2]).x)
            res(x)
            cuts[float(beta)] = (f['ctl/rud-deg'], 0.5 * (f['ctl/ail-right-deg'] - f['ctl/ail-left-deg']))
    print('  rudder, aileron by sideslip: ' + ', '.join('%+.0f: %+.1f, %+.1f' % (b, cuts[b][0], cuts[b][1]) for b in sorted(cuts)))
    check('aileron per degree of sideslip, -15 to +15', abs(cuts[15.0][1] - cuts[-15.0][1]) / 30, 0.80, 0.20, '%.2f', True)


# Pressure height, then rows of rpm, % power, KTAS at ISA [AFM 5-17 to 5-19]
CRUISE = {0: [(2250, 97, 145), (2250, 83, 136), (2250, 65, 124), (2250, 56, 116), (2250, 43, 103), (2250, 37, 95), (2100, 63, 122), (2100, 41, 100), (1900, 58, 118), (1900, 39, 97)],
          3000: [(2388, 87, 143), (2250, 81, 138), (2250, 54, 117), (2100, 65, 127), (1900, 60, 122)],
          6000: [(2388, 79, 141), (2250, 65, 130), (2250, 44, 108), (2100, 51, 116), (1900, 47, 112)],
          9000: [(2388, 71, 139), (2250, 62, 131), (2100, 48, 116), (1900, 54, 123)],
          12000: [(2388, 63, 136), (2250, 57, 129), (1900, 50, 121)]}


def cruise(show=True):
    f = X.load(kg=1150.0, cg=0.25)
    errs = []
    if show:
        print('\nCruise, 1150 kg, ISA [AFM 5-17 to 5-19]: speed at the manual\'s power')
    for hp, rows in CRUISE.items():
        line = []
        for rpm, pct, ktas in rows:
            for n in 'lr':
                f['prop/%s/rpm' % n] = rpm
            kw = pct / 100 * P.ENGINE['takeoff_kw']

            def need(k):
                c = H.tas_of_cas(f, 100.0, hp) / 100.0                 # TAS per CAS, near enough for a start
                t = X.trim(f, k / (c / KT), hp=float(hp))
                return t['kw'] - kw
            try:
                k = brentq(need, 82, 190, xtol=0.1)
            except ValueError:
                k = float('nan')
            errs.append(k - ktas)
            line.append('%d/%d%%: %.0f (%d)' % (rpm, pct, k, ktas))
        if show:
            print('  %5d ft  ' % hp + ', '.join(line))
    e = np.array(errs)
    for n in 'lr':
        f['prop/%s/rpm' % n] = P.PROP['rpm']
    if show:
        print('  mean %+.1f kt, rms %.1f kt, worst %+.1f kt' % (e.mean(), np.sqrt((e ** 2).mean()), e[np.argmax(np.abs(e))]))
        check('worst cruise speed error, kt', float(np.abs(e).max()), 0.0, 3.0, '%.1f')
    return e


# weight, height, KIAS, ft/min at ISA [AFM 5-13, 5-15]
CLIMB = [(1180, 0, 84, 1102), (1180, 4000, 81, 880), (1180, 8000, 77, 657), (1080, 0, 83, 1251), (930, 0, 82, 1527)]
CLIMB1 = [(1180, 0, 80, 206), (1180, 4000, 79, 85), (1080, 0, 80, 271), (930, 0, 79, 390)]


def climb():
    print('\nEnroute climb, maximum continuous power [AFM 5-13]')
    f = None
    for kg, hp, kias, roc in CLIMB:
        f = X.load(kg=float(kg), cg=0.25)
        kw = P.ENGINE['continuous_kw'] * np.interp(hp, P.POWER_LAPSE['alt_ft'], P.POWER_LAPSE['frac'])
        t = X.trim(f, kias - 1.0, hp=float(hp), kw=kw)
        check('%d kg, %d ft, %d KIAS, ft/min' % (kg, hp, kias), t['roc'], roc, 0.10, '%.0f', True)
    print('One engine, feathered, maximum continuous on the other, balanced by rudder and sideslip with 3 deg of bank towards it [AFM 5-15]')
    for kg, hp, kias, roc in CLIMB1:
        f = X.load(kg=float(kg), cg=0.25)
        kw = P.ENGINE['continuous_kw'] * np.interp(hp, P.POWER_LAPSE['alt_ft'], P.POWER_LAPSE['frac'])
        t = one_engine(f, kias - 1.0, float(hp), kw)
        check('%d kg, %d ft, %d KIAS, ft/min' % (kg, hp, kias), t, roc, 60.0, '%.0f')


def one_engine(f, kcas, hp, kw):
    """left engine feathered: wings level, the rudder holding the heading, a little sideslip and bank as a pilot flies it"""
    from scipy.optimize import least_squares
    V = H.tas_of_cas(f, kcas, hp)
    lv = X.lever_of(kw, hp)

    def res(x):
        al, st, ail, rud, beta, gam = x
        X.configure(f, 0, 0, st, (0.0, lv), ail, rud, (0, 1))
        a, b = math.radians(al), math.radians(beta)
        phi = math.radians(3.0)                          # up to 5 deg into the good engine, as the manual's procedure
        acc = H.state(f, V * math.cos(a) * math.cos(b), V * math.sin(b), V * math.sin(a) * math.cos(b), phi=phi, theta=a + math.radians(gam), hp=hp)
        c = math.cos(a)
        return [acc[0] * c + acc[2] * math.sin(a), -acc[0] * math.sin(a) + acc[2] * c, acc[1], acc[3] * 5, acc[4] * 5, acc[5] * 5]
    s = least_squares(res, [5.0, -6.0, 0.0, 0.0, 0.0, 1.0], x_scale=[1, 1, .2, .2, 1, 1])
    return V * math.sin(math.radians(s.x[5])) * 60


def stall():
    print('\nStall speeds, 1230 kg, forward CG 16.5 %, idle, gear down [AFM 5-6]')
    f = X.load(kg=1230.0, cg=0.165)
    W = 1230 * G
    for flap, name, ref in ((0.0, 'flaps up', 64.0), (20.0, 'flaps T/O', 56.0), (40.0, 'flaps full', 54.0)):
        best = (0, 0, 0)
        for a in np.arange(4.0, 22.0, 0.5):
            t = dict(flap=flap, gear=1, stab=-8.0, lever=0.0, run=(1, 1), V=110.0, hp=0.0, alpha=a, gamma=0.0)

            def cm(st):
                return X.at(f, t, alpha=a, stab=st)['Cm']
            try:
                st = brentq(cm, -15.0, 4.0, xtol=0.02)
            except ValueError:
                continue
            c = X.at(f, t, alpha=a, stab=st)
            if c['CL'] > best[0]:
                best = (c['CL'], a, st)
        vs = math.sqrt(2 * W / (1.225 * 14.76 * best[0])) / 0.514444
        print('  %s: CL max %.3f at alpha %.1f, stabilator %.1f' % (name, best[0], best[1], best[2]))
        check('stall speed, %s, KCAS' % name, vs, ref, 2.0, '%.1f')


def fly(f, t, stab_of, T=50.0):
    """the model flown through a stabilator history, hands off otherwise. The rates and the wind are JSBSim's own, so the downwash's lag is live"""
    f['bu/test/frozen'] = 0
    H.freeze(f, False)
    a = math.radians(t['alpha'])
    f['ic/h-sl-ft'] = t['hp']
    f['ic/terrain-elevation-ft'] = -3000.0
    f['ic/psi-true-rad'] = 0.0
    f['ic/theta-rad'] = a
    f['ic/phi-rad'] = 0.0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = t['V'] * math.cos(a), 0.0, t['V'] * math.sin(a)
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0.0
    X.configure(f, t['flap'], t['gear'], t['stab'], (t['lever'], t['lever']))
    f.run_ic()
    out = []
    n = int(T * 120)
    for i in range(n):
        ti = i / 120.0
        f['fcs/elevator-cmd-norm'] = (stab_of(ti) + 5.0) / 10.0
        f.run()
        out.append((ti, f['velocities/vt-fps'] * 0.3048, f['aero/alpha-deg'], math.degrees(f['attitude/theta-rad']), math.degrees(f['velocities/q-rad_sec']), f['accelerations/Nz'] * 9.81))
    f['bu/test/frozen'] = 1
    H.freeze(f)
    return np.array(out)


def ramps(points):
    return lambda x: float(np.interp(x, [p[0] for p in points], [p[1] for p in points]))


def max_slope(tm, a):
    """Kimberlin's method on a step's response, as the paper reads fig. 15: the steepest tangent between the start and the settled value, the overshoot, and the chart of fig. 15b"""
    i0 = int(np.argmin(a))
    j = i0 + int(np.argmax(np.gradient(a[i0:], tm[i0:])))
    s = np.gradient(a, tm)[j]
    peak = a[i0:i0 + 360].max()
    settled = a[i0 + 300:i0 + 500].mean()
    t1 = tm[j] + (a[i0] - a[j]) / s
    t2 = tm[j] + (peak - a[j]) / s
    over = (peak - settled) / max(settled - a[i0], 1e-6)
    x = [0.0, 0.08, 0.115, 0.16, 0.20, 0.24, 0.28, 0.32]          # fig. 15b: overshoot to w (t2 - t1)
    y = [1.7, 1.9, 2.05, 2.45, 2.75, 3.1, 3.6, 4.2]
    return float(np.interp(over, x, y)) / (t2 - t1), over, t2 - t1


def replay():
    print('\nThe stabilator history of fig. 24, flown. Measured by eye from the paper: V 55-64 m/s, az 3.5-17 m/s2, q -10 to +11 deg/s, vane alpha -1.2 to 4.7 deg')
    f = flight_build()
    t = X.trim(f, P.TEST['kias'], hp=P.TEST['hp'])
    s0 = t['stab']
    d = s0 - (-4.5)
    fig = ramps([(0, -4.5), (13.3, -4.5), (13.9, -6.5), (15.3, -6.5), (16.2, -3.0), (17.9, -3.0), (18.5, -6.3), (19.7, -6.3), (20.6, -4.5), (50, -4.5)])
    r = fly(f, t, lambda x: fig(x) + d)
    print('  model: V %.1f-%.1f m/s, az %.1f-%.1f m/s2, q %.1f to %.1f deg/s, alpha %.1f to %.1f deg (trim %.1f)' % (r[:, 1].min(), r[:, 1].max(), r[:, 5].min(), r[:, 5].max(), r[:, 4].min(), r[:, 4].max(), r[:, 2].min(), r[:, 2].max(), t['alpha']))
    up, da = r[:, 5].max() - 9.81, r[:, 2].max() - t['alpha']
    print('  lift for 2 deg of stabilator: az up %.1f m/s2 (flight 7), alpha up %.1f deg (the vane 3.7). Load per degree %.2f against the vane\'s 1.89: the vane reads %.2f of true alpha, if the model\'s lift slope is right' % (up, da, up / da, up / da / 1.89))
    m = X.modes(f, t)
    w = abs(m['short_period'])
    # the paper's method on the same kind of input: steps as fast as the pilot's, about 0.6 s
    for ramp in (0.05, 0.3, 0.6, 0.9):
        st = ramps([(0, s0), (4.0, s0), (4.0 + ramp, s0 - 2.0), (6.0, s0 - 2.0), (6.0 + ramp, s0), (20, s0)])
        r = fly(f, t, st, T=12.0)
        k = r[:, 0] > 6.0
        # a fall then a rise, as fig. 15: alpha after a stabilator step back
        e, over, dt = max_slope(r[k, 0], -r[k, 2])
        print('  max-slope method, step in %.2f s: %.2f rad/s (overshoot %.3f, t2 - t1 %.2f s); the model\'s root is %.2f' % (ramp, e, over, dt, w))


def vmc():
    print('\nMinimum control speed, left engine feathered, right at take-off power, flaps T/O, gear up, 5 deg of bank [AFM 62 KIAS]. Printed, no limit: Phase 5\'s check')
    from scipy.optimize import least_squares
    f = X.load(kg=1180.0, cg=0.25)
    hp = 0.0
    lv = X.lever_of(P.ENGINE['takeoff_kw'], hp)

    def rudder(kcas):
        V = H.tas_of_cas(f, kcas, hp)

        def res(x):
            al, st, ail, rud, beta = x
            X.configure(f, 20.0, 0, st, (0.0, lv), ail, rud, (0, 1))
            a, b = math.radians(al), math.radians(beta)
            acc = H.state(f, V * math.cos(a) * math.cos(b), V * math.sin(b), V * math.sin(a) * math.cos(b), phi=math.radians(5.0), theta=a + math.radians(2.0), hp=hp)
            return [-acc[0] * math.sin(a) + acc[2] * math.cos(a), acc[1], acc[3] * 5, acc[4] * 5, acc[5] * 5]
        s = least_squares(res, [8.0, -7.0, 0.0, 0.3, 0.0], x_scale=[1, 1, .2, .2, 1])
        return s.x[3]
    for k in (70, 65, 60, 55):
        print('  %d KCAS: pedal %.2f of full' % (k, abs(rudder(float(k)))))


def main():
    run = sys.argv[1:] or ['tunnel', 'modes', 'replay', 'derivatives', 'trims', 'sideslip', 'cruise', 'climb', 'stall', 'vmc', 'modii']
    for r in run:
        globals()[r]()
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else '%d misses: %s' % (len(fails), '; '.join(fails))))


if __name__ == '__main__':
    main()
