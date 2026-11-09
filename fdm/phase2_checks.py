#!/usr/bin/env python3
"""Phase 2 checks: the discs, motors, hubs and battery, through JSBSim, against NASA's tables, its tunnel and its mission.

    python3 phase2_checks.py [tables] [tunnel] [windmill] [response] [failures] [cell] [mission] [power]

Limits are stated with each check. Tunnel and cell data are measured; the rest are NASA's estimates.
"""
import json, math, os, statistics as st, sys
import numpy as np
import harness as H
import propulsion as PR
import x57data as X

HERE = os.path.dirname(os.path.abspath(__file__))
L = lambda n: json.load(open(os.path.join(HERE, 'data', n)))
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'MISS', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def tables(f):
    """NASA's Airspeed and Fixed tables, standard day, through the model: the model follows the tunnel, so its thrust at high
    J is meant to be under NASA's estimates"""
    Hh = L('hlp_tables.json')
    for name, mode in (('airspeed', 2), ('fixed', 1)):
        rows = []
        for q in Hh[name]:
            if q['hp'] > 8000 or q['keas'] < 20:
                continue
            H.configure(f, flap=10, gear=1, lever=0.2, stab=-3, hl_mode=mode)
            V = q['keas'] * H.FPS_KT / math.sqrt(H_sigma(q['hp']))
            H.air(f, V, 0.0, hp=q['hp'])
            rows.append((q['keas'], q['hp'], f['prop/hl/rpm-cmd'], q['rpm'], f['prop/hl/d7/torque-nm'], q['Q_nm'], f['prop/hl/d7/thrust-lbs'], q['T_lbf'], f['prop/hl/d7/J'], q['J']))
        A = np.array(rows)
        print('%s mode, %d of NASA\'s points, up to 8,000 ft' % (name, len(A)))
        if mode == 2:
            e = np.abs(A[:, 2] - A[:, 3])
            check('  rpm schedule reproduced', e.max() < 5, 'worst %.1f rpm' % e.max())
        for lo, hi in ((0.5, 0.8), (0.8, 1.1), (1.1, 1.35)):
            m = (A[:, 9] >= lo) & (A[:, 9] < hi) & (np.abs(A[:, 5]) > 1.5)
            if m.any():
                dq = A[m, 4] / A[m, 5] - 1
                dt = A[m, 6] / np.where(np.abs(A[m, 7]) > 1, A[m, 7], np.nan) - 1
                print('    J %.1f-%.2f, %3d points: torque %+.0f %% (worst %+.0f), thrust %+.0f %% (worst %+.0f)' % (
                    lo, hi, m.sum(), 100 * dq.mean(), 100 * dq[np.abs(dq).argmax()], 100 * np.nanmean(dt), 100 * dt[np.nanargmax(np.abs(dt))]))
        m = (A[:, 9] >= 0.5) & (A[:, 9] < 1.1) & (np.abs(A[:, 5]) > 1.5)
        dq = np.abs(A[m, 4] / A[m, 5] - 1)
        check('  %s: torque within 10 %% of NASA\'s to J 1.1' % name, dq.max() < 0.10, 'worst %.1f %%' % (100 * dq.max()))


def H_sigma(hp):
    return (1 - 6.87559e-6 * hp) ** 4.2559


def tunnel():
    T = L('digitised/hlp_tunnel.json')
    d = L('disc_tables.json')['high_lift']
    for c, tol in (('Ct', 0.012), ('Cq', 0.0015)):
        for k in ('LH', 'RH'):
            x = np.array(T[c][k])
            e = np.interp(x[:, 0], d['J'], d[c]) - x[:, 1]
            print('     %s %s: model less measured, mean %+.4f, rms %.4f, %d points' % (c, k, e.mean(), np.sqrt((e ** 2).mean()), len(e)))
        x = np.array(T[c]['LH'] + T[c]['RH'])
        e = np.interp(x[:, 0], d['J'], d[c]) - x[:, 1]
        check('%s inside the tunnel\'s scatter' % c, np.sqrt((e ** 2).mean()) < tol, 'rms %.4f' % np.sqrt((e ** 2).mean()))


def wake():
    """The high-lift slipstream one radius aft of the disc's centre against NASA's wake survey [20220006506 slide 9], an isolated propeller in the tunnel.
    The model's jet, as builtup's discs() makes it: the blades' thrust (NASA's table) and the swirl's low pressure over the annulus outside the nacelle, its
    outer edge by momentum theory (builtup.jet_numbers), spread one radius back by a ring-vortex tube, less the nacelle's own tube. Before 2026-11-09 (FIT jet
    'ring'): the balance's thrust over the whole disc at a fixed contraction. Limits set before the model was run: the mean over 0.45-0.85 R within 15 % at 58
    and 75 KEAS and 5 ft/s at 90; the edge, where the speed falls through half that, within 0.05 R"""
    import builtup as BU
    import ringwake as RW
    import x57built as B
    W = L('digitised/hlp_wake.json')
    d = L('disc_tables.json')['high_lift']
    D = B.GEO['high_lift']['diameter_in'] / 12
    k = B.FIT['contraction']

    def plateau_edge(r, v):
        m = (r >= 0.45 - 1e-9) & (r <= 0.85 + 1e-9)
        pl = float(v[m].mean())
        i = np.where((v[:-1] >= pl / 2) & (v[1:] < pl / 2))[0]
        if not len(i):
            return pl, float('nan')
        i = i[-1]
        return pl, float(r[i] + (pl / 2 - v[i]) / (v[i + 1] - v[i]) * (r[i + 1] - r[i]))
    rg = np.round(np.arange(0.30, 1.201, 0.01), 2)
    for s in ('58', '75', '90'):
        V = W[s]['keas'] * 1.68781                   # the tunnel near sea level: true speed as equivalent
        n = W[s]['rpm'] / 60
        J = V / (n * D)
        p = np.array(W[s]['points'])
        r = np.unique(p[:, 0])
        pm, em = plateau_edge(r, np.array([p[p[:, 0] == q, 1].mean() for q in r]))
        if B.FIT.get('jet') in ('beta', 'sat'):
            rho, hub = 0.002377, B.HL_NACELLE_IN / B.GEO['high_lift']['diameter_in']
            a, ro = BU.jet_numbers(float(np.interp(J, d['J'], d['nasa_Ct'])) * rho * n * n * D ** 4, float(np.interp(J, d['J'], d['Cq'])) * rho * n * n * D ** 5, V, D / 2, hub, rho)
            prof = [a * (RW.at(1.0, float(q), round(ro, 3)) - RW.at(round(1 / hub, 3), round(float(q) / hub, 2), 1.0)) for q in rg]
        else:
            Ct = float(np.interp(J, d['J'], d['Ct']))
            a = 0.5 * V * (math.sqrt(1 + 8 * Ct / (math.pi * J * J)) - 1)
            prof = [a * RW.at(1.0, float(q), k) for q in rg]
        pl, ed = plateau_edge(rg, np.array(prof))
        print('     %s KEAS, %d rpm, J %.3f: the model %.1f ft/s to %.3f R, the survey %.1f to %.3f' % (s, W[s]['rpm'], J, pl, ed, pm, em))
        if s == '90':
            check('slipstream at 90 KEAS within 5 ft/s of the survey', abs(pl - pm) <= 5.0, '%+.1f ft/s' % (pl - pm))
        else:
            check('slipstream speed at %s KEAS within 15 %% of the survey' % s, abs(pl / pm - 1) <= 0.15, '%+.0f %%' % (100 * (pl / pm - 1)))
            check('slipstream edge at %s KEAS within 0.05 R of the survey' % s, abs(ed - em) <= 0.05, '%+.3f R' % (ed - em))


def cruise_points():
    """NASA's Ct and Cp at its 37 points, from the tables the flight model reads"""
    import discs
    c = L('disc_tables.json')['cruise']
    from scipy.interpolate import RegularGridInterpolator as RG
    ct = RG((c['J'], c['pitch']), np.array(c['Ct']))
    cp = RG((c['J'], c['pitch']), np.array(c['Cp']))
    P = discs.nasa_cruise()
    e = np.array([[float(ct(k)) - v[0], float(cp(k)) - v[1]] for k, v in P.items()])
    check('cruise tables pass through NASA\'s points', np.abs(e).max() < 0.01, 'worst Ct %.4f, Cp %.4f, %d points' % (np.abs(e[:, 0]).max(), np.abs(e[:, 1]).max(), len(e)))


def still(f, V, hp, alpha=4.0):
    """the flight state fixed, the rotors free: integrators off, so only the discs and motors move"""
    H.air(f, V, alpha, hp=hp)
    f['ctl/hold-props'] = 0


def run(f, s, fn=None):
    for i in range(int(s * 120)):
        f.run()
        if fn:
            fn(i / 120.0)


def windmill():
    """A disc that loses power windmills. Measured in the tunnel at 75 KEAS at sea level: 2483 rpm left-handed, 2654 right-handed
    [20210016834 table 5, zero torque]. NASA's Table 7 is an older blade-element estimate, printed beside the model"""
    tm = {50: 1651, 58: 1937, 65: 2184, 75: 2357, 90: 3065}
    f = H.load('mod4')

    def fail(k, hp):
        f['ctl/hold-props'] = 1
        for b in ('al', 'ar', 'bl', 'br'):
            f['ctl/contactor-%s' % b] = 1
        H.configure(f, flap=30, gear=1, lever=0.2, stab=-3, hl_mode=2)
        V = k * H.FPS_KT / math.sqrt(H_sigma(hp))
        still(f, V, hp, 0.0)
        run(f, 2.5)                      # past an air start's hold
        f['ctl/contactor-br'] = 0        # motors 7, 9, 11
        run(f, 8)
        return f['prop/hl/d7/rpm']

    r = fail(75, 0.0)
    print('     tunnel, 75 KEAS, sea level: failed disc %.0f rpm, J %.2f, thrust %.1f lbf (measured 2483 and 2654 rpm)' % (r, f['prop/hl/d7/J'], f['prop/hl/d7/thrust-lbs']))
    check('windmilling speed inside the tunnel\'s measured range, or within 3 %', 2483 * 0.97 <= r <= 2654 * 1.03, '%.0f rpm' % r)
    for k, want in tm.items():
        r = fail(k, 2500.0)
        print('     %2d KCAS, 2,500 ft: failed disc %.0f rpm, %+.0f %% on Table 7\'s %d, J %.2f; a good disc %.0f rpm' % (k, r, 100 * (r / want - 1), want, f['prop/hl/d7/J'], f['prop/hl/d8/rpm']))


def response():
    f = H.load('mod3')
    t = H.trim(f, 120, flap=0, gear=0, hp=6000)
    H.configure(f, flap=0, gear=0, lever=t['lever'], stab=t['stab'])
    still(f, t['V'], 6000.0, t['alpha'])
    run(f, 3)
    p0 = f['prop/r/pitch-deg']
    print('Cruise motor, 120 KCAS, 6,000 ft: lever %.2f, rpm %.0f, pitch %.1f' % (t['lever'], f['prop/r/rpm'], p0))
    check('steady before the step', abs(f['prop/r/rpm'] - 2250) < 5, '%.1f rpm' % f['prop/r/rpm'])
    rec = dict(peak=0.0, rate=0.0, last=p0)

    def log(ti):
        rec['peak'] = max(rec['peak'], f['prop/r/rpm'])
        rec['rate'] = max(rec['rate'], abs(f['prop/r/pitch-deg'] - rec['last']) * 120)
        rec['last'] = f['prop/r/pitch-deg']

    for e in (0, 1):
        f['/controls/engines/engine[%d]/throttle' % e] = min(t['lever'] + 0.25, 1.0)
    run(f, 40, log)
    print('     lever +0.25: rpm peak %.0f, back to %.0f after 40 s; pitch %.1f to %.1f at up to %.2f deg/s; thrust %.0f lbf each' % (
        rec['peak'], f['prop/r/rpm'], p0, f['prop/r/pitch-deg'], rec['rate'], f['prop/r/thrust-lbs']))
    check('the hub keeps to 1 deg/s', rec['rate'] <= 1.01)
    check('overspeed held near 2750 rpm', rec['peak'] < 2900, 'peak %.0f' % rec['peak'])
    check('the governor brings rpm back', abs(f['prop/r/rpm'] - 2250) < 25, '%.0f rpm' % f['prop/r/rpm'])
    # a hub that fails fixed: the speed then follows the torque, as NASA says
    f['ctl/hub-fixed-r'] = 1
    for e in (0, 1):
        f['/controls/engines/engine[%d]/throttle' % e] = t['lever']
    run(f, 10)
    print('     hub fixed, lever back: right %.0f rpm, left %.0f rpm' % (f['prop/r/rpm'], f['prop/l/rpm']))
    check('a fixed hub lets the rpm go, the other governs', f['prop/r/rpm'] < 2200 and abs(f['prop/l/rpm'] - 2250) < 200)

    # idle at 75 KCAS: the hub is on its fine stop and the speed sags. The trim's speed has to be the rotor's
    f = H.load('mod4')
    t = H.trim(f, 75, flap=12, gear=1, hl_mode=1, hp=2500, lever=0.0, x0=[5, -3, 3.0])
    held, thrust = f['prop/r/rpm'], f['prop/r/thrust-lbs']
    H.configure(f, flap=12, gear=1, hl_mode=1, lever=0.0, stab=t['stab'])
    still(f, t['V'], 2500.0, t['alpha'])
    f['prop/r/rpm-state'], f['prop/l/rpm-state'], f['prop/r/pitch-state'], f['prop/l/pitch-state'] = 2250.0, 2250.0, 20.0, 20.0
    run(f, 40)
    print('Idle, 75 KCAS, high lift Fixed: trim %.0f rpm, %.1f lbf a side; flown %.0f rpm, %.1f lbf, pitch %.1f' % (
        held, thrust, f['prop/r/rpm'], f['prop/r/thrust-lbs'], f['prop/r/pitch-deg']))
    check('on the fine stop the trim\'s rpm is where the rotor settles', abs(f['prop/r/rpm'] - held) < 15, '%.0f, %.0f' % (held, f['prop/r/rpm']))

    f = H.load('mod4')
    f['ctl/hold-props'] = 0
    H.configure(f, flap=10, gear=1, lever=0.0, stab=-5, hl_mode=0)
    H.air(f, 1.0, 0.0, hp=2300.0)
    f['ctl/hold-props'] = 0
    run(f, 2.5)                          # past an air start's hold
    f['ctl/hl-mode'] = 1
    tt = {}

    def up(ti):
        if f['prop/hl/d7/rpm'] > 4560 and 'up' not in tt:
            tt['up'] = ti
    run(f, 6, up)
    print('High-lift motors, standing: armed Fixed: 95 %% in %.2f s, %.0f rpm, %.1f Nm, %.1f kW, %.1f lbf a disc (NASA run-up 10 kW, 120 kW all twelve)' % (
        tt.get('up', -1), f['prop/hl/d7/rpm'], f['prop/hl/d7/torque-nm'], f['prop/hl/d7/shaft-kw'], f['prop/hl/d7/thrust-lbs']))
    check('Fixed run-up power near NASA\'s 10 kW a disc', abs(f['prop/hl/d7/shaft-kw'] - 10) < 1.5, '%.1f kW' % f['prop/hl/d7/shaft-kw'])
    f['ctl/hl-mode'] = 0
    td = {}

    def down(ti):
        if f['prop/hl/d7/rpm'] <= 0 and 'dn' not in td:
            td['dn'] = ti
    run(f, 12, down)
    check('disarmed, the discs stop', 'dn' in td, 'in %.1f s' % td.get('dn', -1))


def failures():
    """a bus lost, then a cruise motor stopped and feathered, on the approach in Airspeed mode"""
    f = H.load('mod4')
    t = H.trim(f, 75, flap=10, gear=1, hl_mode=2, hp=2500)
    H.configure(f, flap=10, gear=1, hl_mode=2, lever=t['lever'], stab=t['stab'])
    still(f, t['V'], 2500.0, t['alpha'])
    run(f, 3)
    both = f['bat/b/load-kw']
    f['ctl/bus-a'] = 0
    run(f, 6)
    print('Bus A lost: pack A %.1f kW, pack B %.1f (%.1f before); discs on A %.0f rpm, on B %.0f; cruise %d controller, %.0f Nm' % (
        f['bat/a/load-kw'], f['bat/b/load-kw'], both, f['prop/hl/d1/rpm'], f['prop/hl/d2/rpm'], f['prop/r/controllers'], f['prop/r/torque-nm']))
    check('a lost bus takes half the discs, evenly, and one controller a motor', abs(f['prop/hl/on-l'] - 0.5) < 0.01 and abs(f['prop/hl/on-r'] - 0.5) < 0.01 and f['prop/r/controllers'] == 1)
    check('a lost bus draws nothing', f['bat/a/load-kw'] == 0.0, '%.1f kW' % f['bat/a/load-kw'])
    f['ctl/bus-a'] = 1
    f['ctl/motor-on-r'] = 0
    run(f, 20)
    wind = f['prop/r/rpm']
    f['ctl/feather-r'] = 1
    run(f, 90)
    print('Right motor off: windmilling at %.0f rpm; feathered: %.0f rpm, pitch %.0f; left %.0f rpm' % (wind, f['prop/r/rpm'], f['prop/r/pitch-deg'], f['prop/l/rpm']))
    check('a stopped motor windmills, and feathers to a stop in 71 s at 1 deg/s', wind > 500 and f['prop/r/rpm'] < 5 and f['prop/r/pitch-deg'] > 84.9)


def cell():
    """Chin fig. 9, the cell's voltage at constant discharge. The rates' curves, read by eye, at 1 and 2 Ah"""
    fig = {0.5: (3.80, 3.47), 1: (3.70, 3.38), 2: (3.50, 3.18), 3: (3.33, 3.00)}
    f = H.load('mod3')
    N = X.BATTERY['series'] * X.BATTERY['parallel']
    worst = 0.0
    for c, want in fig.items():
        f['ctl/hold-props'] = 1
        for b in ('a', 'b'):
            f['bat/%s/soc' % b] = 1.0
            f['bat/%s/uth' % b] = 0.0
            f['bat/%s/temp-c' % b] = 25.0
        H.configure(f, flap=0, gear=0, lever=0.0, stab=-3)
        H.air(f, 200.0, 4.0, hp=2000.0)
        f['ctl/hold-props'] = 0
        f['ctl/bus-test'] = 1
        A = 3.0 * c
        got = []
        ah = 0.0
        last = 0.0
        tstep = 1 / 10.0
        while ah < 2.0 and last < 3 * 3600:
            f['ctl/bus-test-kw'] = A * f['bat/a/cell-v'] * N / 1000.0 if f['bat/a/cell-v'] > 2 else 0.0
            for i in range(12):
                f.run()
            ah = (1 - f['bat/a/soc']) * 3.0
            last += tstep
            if not got and ah >= 1.0:
                got.append(f['bat/a/cell-v'])
        got.append(f['bat/a/cell-v'])
        e = [got[0] - want[0], got[1] - want[1]]
        worst = max(worst, max(abs(x) for x in e))
        print('     %.1f C: at 1 Ah %.2f V (fig. %.2f), at 2 Ah %.2f V (%.2f); cell %.1f C' % (c, got[0], want[0], got[1], want[1], f['bat/a/temp-c']))
    f['ctl/bus-test'] = 0
    check('cell within 0.1 V of Chin fig. 9', worst < 0.1, 'worst %.2f V' % worst)


MISSION = [        # ANLYS-CEPT-018, Mod IV slow flight: seconds, kW of the system
    (300, 2.0), (30, 137.2), (10, 8.2), (10, 136.6), (30, 8.2), (20, 229.2), (90, 184.2), (385, 137.2), (60, 103.4), (50, 66.2), (180, 140.3), (50, 190.1),
    (60, 103.4), (180, 35.8), (120, 69.6), (60, 69.6), (60, 18.9)]


def mission():
    f = H.load('mod3')
    H.configure(f, flap=0, gear=0, lever=0.0, stab=-3)
    H.air(f, 200.0, 4.0, hp=2000.0)
    f['ctl/hold-props'] = 0
    f['ctl/bus-test'] = 1
    for b in ('a', 'b'):
        f['bat/%s/temp-c' % b] = 10.0
    e = 0.0
    vmin = 999.0
    for sec, kw in MISSION:
        f['ctl/bus-test-kw'] = kw / 2
        for i in range(int(sec * 10)):
            for j in range(12):
                f.run()
            vmin = min(vmin, f['bat/a/volts'])
        e += sec * kw / 3600
    soc = f['bat/a/soc']
    tmax = f['bat/a/temp-c']
    print('Mission: %.1f kWh drawn (NASA 41.9, of 51.2 usable), %d s. NASA\'s plots for it, against the model: lowest 370 V, %.0f; depth of discharge 82.6 %%, %.1f; hottest 57.9 C, %.1f' % (
        e, sum(s for s, _ in MISSION), vmin, 100 * (1 - soc), tmax))
    check('mission energy is NASA\'s', abs(e - 41.9) < 0.3, '%.2f kWh' % e)
    check('lowest voltage within 15 V of NASA\'s 370', abs(vmin - 370) < 15, '%.0f V' % vmin)
    check('depth of discharge within 5 points of NASA\'s 82.6 %', abs(100 * (1 - soc) - 82.6) < 5, '%.1f %%' % (100 * (1 - soc)))
    check('hottest pack within 5 C of NASA\'s 57.9', abs(tmax - 57.9) < 5, '%.1f C' % tmax)
    f['ctl/bus-test'] = 0


def power():
    """NASA's planning powers, ANLYS-CEPT-018: shaft kW, cruise and high lift, against the model trimmed there"""
    # NASA doesn't say how the aircraft is set up for each, so the gear is tried both ways
    cases = (('Mod III setup, 120 KCAS, 6,000 ft, clean', 'mod3', 120, 6000, 0, 0, 0, 0.0, 90.0, 0.0),
             ('Mod III setup, the same, gear down', 'mod3', 120, 6000, 0, 1, 0, 0.0, 90.0, 0.0),
             ('Mod III pattern, 105 KCAS, 3,800 ft, clean', 'mod3', 105, 3800, 0, 0, 0, 0.0, 60.0, 0.0),
             ('Mod III pattern, the same, gear down', 'mod3', 105, 3800, 0, 1, 0, 0.0, 60.0, 0.0),
             ('Mod III climb, 100 KCAS, 2,800 ft, 500 ft/min, clean', 'mod3', 100, 2800, 0, 0, 0, 2.8, 120.0, 0.0),
             ('Mod III climb, the same, gear down', 'mod3', 100, 2800, 0, 1, 0, 2.8, 120.0, 0.0),
             ('Mod IV hold, 63 KCAS, 6,000 ft, flaps 30, gear down', 'mod4', 63, 6000, 30, 1, 2, 0.0, 10.0, 115.2))
    for name, mod, k, hp, flap, gear, hl, gam, nc, nh in cases:
        f = H.load(mod)
        f['inertia/pointmass-weight-lbs[0]'] = 0.0
        t = H.trim(f, k, flap=flap, gear=gear, hl_mode=hl, hp=hp, gamma_deg=gam)
        hk = 12 * f['prop/hl/power-each-kw'] if mod == 'mod4' else 0.0
        bus = f['bat/a/load-kw'] + f['bat/b/load-kw']
        print('     %-54s cruise %.0f kW (NASA %.0f), high lift %.0f (%.0f), system %.0f; alpha %.1f' % (name, t['power_kw'], nc, hk, nh, bus, t['alpha']))
    # the chain itself: from shaft power to the system's
    f = H.load('mod4')
    t = H.trim(f, 90, flap=10, gear=1, hl_mode=1, hp=2500)
    c, h = t['power_kw'], 12 * f['prop/hl/power-each-kw']
    want = (c / (0.92 * 0.97) + h / (0.93 * 0.98) + 2.0) / 0.995
    got = f['bat/a/load-kw'] + f['bat/b/load-kw']
    check('the efficiency chain is NASA\'s', abs(got / want - 1) < 0.002, 'system %.1f kW from %.1f + %.1f shaft' % (got, c, h))
    a = 96 / (0.92 * 0.97) + 108 / (0.93 * 0.98)
    print('     (NASA\'s ground roll: 96 + 108 kW shaft gives %.1f kW by this chain; its table says 229.2)' % ((a + 2) / 0.995))


if __name__ == '__main__':
    what = sys.argv[1:] or ['tables', 'tunnel', 'windmill', 'response', 'failures', 'cell', 'mission', 'power']
    if 'tables' in what:
        tables(H.load('mod4'))
    if 'tunnel' in what:
        print('Tunnel, 2022 retest'); tunnel(); wake(); cruise_points()
    if 'windmill' in what:
        print('Windmilling after a contactor failure'); windmill()
    if 'response' in what:
        response()
    if 'failures' in what:
        failures()
    if 'cell' in what:
        print('Cell'); cell()
    if 'mission' in what:
        mission()
    if 'power' in what:
        print('Planning powers'); power()
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else 'MISSES: ' + '; '.join(fails)))
