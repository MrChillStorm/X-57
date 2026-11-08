#!/usr/bin/env python3
"""Phase 5: ground effect and NASA's failures, on the built-up model. Limits in docs/PLAN.md. All of NASA's numbers here are its simulator's or its CFD's: the X-57 never flew with this wing.

    python3 phase5_checks.py [ground climb vmc upset contactor failures gear]
"""
import json, math, os, sys
os.environ['X57_BUILT'] = '1'
import numpy as np
from scipy.optimize import brentq, least_squares
import harness as H
import phase1_checks as C

fails = []
Z_LE = 87.5


def check(name, val, ref, tol, fmt='%.0f', rel=False):
    t = tol * abs(ref) if rel else tol
    ok = abs(val - ref) <= t
    if not ok:
        fails.append(name)
    print('  %-50s %9s   target %-8s %s' % (name, fmt % val, fmt % ref, 'ok' if ok else 'MISS (limit %s)' % (('%.0f %%' % (100 * tol)) if rel else tol)))


def clear(f):
    for s in 'lr':
        f['ctl/motor-on-%s' % s] = 1
        f['ctl/cmc-%sa' % s] = 1
        f['ctl/cmc-%sb' % s] = 1
        for k in ('feather', 'hub-fixed', 'hub-fine', 'feather-now', 'no-prop-forces', 'cmd-stuck'):
            f['ctl/%s-%s' % (k, s)] = 0
        if f.mod == 'mod4':
            f['ctl/contactor-a%s' % s] = 1
            f['ctl/contactor-b%s' % s] = 1
    bus(f, 'a', 1)
    bus(f, 'b', 1)


def bus(f, b, on):
    """the battery's channel runs every twelfth frame, so a frozen state needs telling at once"""
    f['ctl/bus-' + b] = on
    f['bat/%s/avail' % b] = on


def ground(f3, f4):
    print('\nGround effect against the CFD, up to 10 deg')

    def at(f, height, flap, a, V, mode=0, rpm=0):
        H.configure(f, flap=flap, gear=0, lever=0.0, stab=0.0, hl_mode=mode)
        if mode:
            f['ctl/hl-rpm-test'] = rpm
        f['ctl/one-aileron'] = 0
        al = math.radians(a)
        f['ic/h-sl-ft'] = 2500.0
        f['ic/terrain-elevation-ft'] = 2500.0 - (height - (Z_LE - f['inertia/cg-z-in']) / 12)
        f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, al, 0
        f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = V * math.cos(al), 0, V * math.sin(al)
        f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
        H.freeze(f)
        f.run_ic()
        f.run()
        f.run()
        return {k: f['aero/coef/' + k] for k in ('CL', 'Cm')}
    for flap in (10.0, 30.0):
        e = {'CL': [], 'Cm': []}
        over = {'CL': 0, 'Cm': 0}
        n = 0
        for r in C.L('poweroff_ground'):
            if r['flap'] != flap or r['alpha'] > 10 or r['height_ft'] > 16:
                continue
            m, m0 = at(f3, r['height_ft'], flap, r['alpha'], 170.0), at(f3, 900.0, flap, r['alpha'], 170.0)
            n += 1
            for c in e:
                d = [r[s]['delta'][c] for s in ('star', 'lava') if s in r and 'delta' in r[s]]
                d = [x for x in d if abs(x) < 0.5 or c == 'Cm']       # STAR-CCM+'s 7.87 ft flaps 30 rows read 0.65: a slip
                x = m[c] - m0[c] - float(np.mean(d))
                e[c].append(x)
                over[c] += abs(x) > max(0.05, max(d) - min(d))
        print('  flaps %2g, power off: RMS CL %.3f, Cm %.3f; outside the limit %d and %d of %d' % (flap, np.sqrt(np.mean(np.square(e['CL']))), np.sqrt(np.mean(np.square(e['Cm']))), over['CL'], over['Cm'], n))
        check('flaps %g, power off, worst dCL' % flap, float(np.abs(e['CL']).max()), 0.0, 0.05, '%.3f')
        check('flaps %g, power off, worst dCm' % flap, float(np.abs(e['Cm']).max()), 0.0, 0.05, '%.3f')
    sets = {s['rpm']: s for s in C.L('highlift_settings')}
    free = {(r['kcas'], r['flap'], r['alpha']): r for r in C.L('highlift') if not any(r[k] for k in ('beta', 'ail', 'stab', 'rud', 'tab'))}
    e = {'CL': [], 'Cm': []}
    worst = []
    for r in C.L('highlift_ground'):
        if any(r[k] for k in ('beta', 'ail', 'stab', 'rud', 'tab')) or r['alpha'] > 10 or r['height_ft'] > 16 or r['rpm'] not in sets:
            continue
        s = sets[r['rpm']]
        b = free.get((s['kcas'], r['flap'], r['alpha']))
        if not b or 'lava' not in b or 'on' not in b['lava'] or 'lava' not in r or 'on' not in r['lava']:
            continue
        f4['bu/test/hl-thrust'], f4['bu/test/hl-torque'] = s['T_lbf'], s['Q_lbft']
        m, m0 = at(f4, r['height_ft'], r['flap'], r['alpha'], s['v_fts'], 3, r['rpm']), at(f4, 900.0, r['flap'], r['alpha'], s['v_fts'], 3, r['rpm'])
        for c in e:
            e[c].append(m[c] - m0[c] - (r['lava']['on'][c] - b['lava']['on'][c]))
        worst.append((abs(e['Cm'][-1]), r['flap'], r['height_ft'], r['alpha'], r['rpm'], m['Cm'] - m0['Cm'], r['lava']['on']['Cm'] - b['lava']['on']['Cm'], m['CL'] - m0['CL'], r['lava']['on']['CL'] - b['lava']['on']['CL']))
    f4['bu/test/hl-thrust'] = -999
    print('  power on, LAVA, %d points: RMS CL %.3f, Cm %.3f' % (len(e['CL']), np.sqrt(np.mean(np.square(e['CL']))), np.sqrt(np.mean(np.square(e['Cm'])))))
    for w in sorted(worst, key=lambda w: -w[0])[:6]:
        print('    flaps %g, %.2f ft, %g deg, %g rpm: dCm %.3f against %.3f, dCL %.3f against %.3f' % w[1:])
    check('power on, worst dCL', float(np.abs(e['CL']).max()), 0.0, 0.15, '%.3f')
    check('power on, worst dCm', float(np.abs(e['Cm']).max()), 0.0, 0.15, '%.3f')


def asym(f, kcas, flap, gear, hp, rpm, lever, hl_mode, phi=None):
    """a steady climb with something failed, flight path solved. With no bank given, at zero sideslip: the bank that gives it, as a pilot flies the ball-out least-drag climb. With one, the sideslip it takes"""
    V = H.tas_of_cas(f, kcas, hp)

    def res(x):
        a, st, ail, rud, y, g = x
        be, ph = (y, phi) if phi is not None else (0.0, y)
        H.configure(f, flap, gear, hl_mode, rpm, lever, stab=st, ail=ail, rud=rud)
        ar, br = math.radians(a), math.radians(be)
        acc = H.state(f, V * math.cos(ar) * math.cos(br), V * math.sin(br), V * math.sin(ar) * math.cos(br), phi=math.radians(ph), theta=ar + math.radians(g), hp=hp)
        return [acc[0] * math.cos(ar) + acc[2] * math.sin(ar), -acc[0] * math.sin(ar) + acc[2] * math.cos(ar), acc[1], acc[3] * 5, acc[4] * 5, acc[5] * 5]
    best = None
    for x0 in ([5.0, -3.0, 0.2, -0.7, 3.0, -3.0], [5.0, -3.0, 0.05, -0.15, 1.0, 3.0], [8.0, -5.0, 0.3, -0.9, 6.0, 0.0]):
        s = least_squares(res, x0, bounds=([-5, -15, -1, -1, -20, -15], [20, 5, 1, 1, 20, 20]), x_scale=[1, 1, 0.2, 0.2, 1, 1], xtol=1e-7, ftol=1e-9, gtol=1e-9)
        if best is None or s.cost < best.cost:
            best = s
        if s.cost < 1e-6:
            break
    s = best
    res(s.x)
    a, st, ail, rud, y, g = s.x
    return dict(roc=V * math.sin(math.radians(g)) * 60, alpha=a, stab=st, ail=ail, rud=rud, y=y, res=float(np.abs(s.fun).max()))


def climb(f3, f4):
    print('\nRates of climb, gear down, takeoff flaps, 2,500 ft. High lift on NASA\'s propeller table, as its simulator')
    HP = 2500.0
    f4['ctl/hl-nasa-table'] = 1

    def sym(f, k, rpm, lever, mode):
        t = H.trim(f, k, flap=10, gear=1, hl_mode=mode, hp=HP, rpm=rpm, lever=lever, x0=[6, -3, 3.0])
        return t['V'] * math.sin(math.radians(t['gamma'])) * 60, t
    clear(f3)
    r, t = sym(f3, 97, 2700.0, 1.0, 0)
    print('  full power: %.1f kW, stabilator %.1f, alpha %.1f' % (t['power_kw'], t['stab'], t['alpha']))
    check('Mod III, 97 KCAS, full power', r, 430, 60)
    f3['ctl/motor-on-l'] = 0
    a = asym(f3, 97, 10, 1, HP, 2700.0, 1.0, 0)
    print('  one motor out, zero sideslip: left disc %.0f lbf, right %.0f, rudder %.2f, aileron %.2f, bank %.1f, residual %.0e' % (f3['prop/l/thrust-lbs'], f3['prop/r/thrust-lbs'], a['rud'], a['ail'], a['y'], a['res']))
    print('  windmilling, at zero sideslip: %.0f ft/min' % a['roc'])
    f3['ctl/feather-now-l'] = 1
    a = asym(f3, 97, 10, 1, HP, 2700.0, 1.0, 0)
    print('  feathered: left disc %.0f lbf, rudder %.2f, aileron %.2f, bank %.1f' % (f3['prop/l/thrust-lbs'], a['rud'], a['ail'], a['y']))
    check('Mod III, one motor out, feathered', a['roc'], -450, 60)
    clear(f3)
    bus(f3, 'a', 0)
    r, t = sym(f3, 97, 2700.0, 1.0, 0)
    check('Mod III, a traction bus out', r, -300, 60)
    clear(f3)
    r, t = sym(f3, 97, 2400.0, 1.0, 0)
    check('Mod III, 2,400 rpm', r, 270, 60)
    for k, nom, cm, con, bus_roc in ((97, 720, 360, 480, -100), (84, 1000, 530, 740, 100)):
        clear(f4)
        r, t = sym(f4, k, 1800.0, 1.0, 1)
        print('  DiTTo, %d KIAS: %.1f kW a motor, each disc %.1f lbf' % (k, t['power_kw'] / 2, f4['prop/hl/d7/thrust-lbs']))
        check('DiTTo %d KIAS' % k, r, nom, 80)
        f4['ctl/motor-on-l'] = 0
        f4['ctl/feather-now-l'] = 1
        a = asym(f4, k, 10, 1, HP, 1800.0, 1.0, 1)
        print('  one motor out, feathered: left disc %.0f lbf, right %.0f, rudder %.2f, bank %.1f' % (f4['prop/l/thrust-lbs'], f4['prop/r/thrust-lbs'], a['rud'], a['y']))
        check('DiTTo %d KIAS, one motor out' % k, a['roc'], cm, 80)
        clear(f4)
        f4['ctl/contactor-bl'] = 0
        a = asym(f4, k, 10, 1, HP, 1800.0, 1.0, 1)
        check('DiTTo %d KIAS, contactor out' % k, a['roc'], con, 80)
        clear(f4)
        bus(f4, 'a', 0)
        r, t = sym(f4, k, 1800.0, 1.0, 1)
        check('DiTTo %d KIAS, a traction bus out' % k, r, bus_roc, 80)
    clear(f4)
    f4['ctl/hl-nasa-table'] = 0


def vmc(f):
    print('\nMinimum control speed, Mod III, takeoff flaps, gear down, full power on the right, left disc windmilling, zero sideslip')
    clear(f)
    f['ctl/motor-on-l'] = 0
    pts = []
    for k in (120.0, 105.0, 97.0, 92.0, 88.0, 85.0, 82.0, 78.0):
        a = asym(f, k, 10, 1, 2500.0, 2700.0, 1.0, 0)
        pts.append((k, abs(a['rud']), a['y'], a['res']))
        print('  %d KCAS: pedal %.2f of full, bank %.1f, residual %.0e' % (k, abs(a['rud']), a['y'], a['res']))
    clear(f)
    p = np.array([q for q in pts if q[1] < 0.995 and q[3] < 1e-3])       # past the stop nothing is solved
    v = float(p[-1, 0] * math.sqrt(p[-1, 1]))           # pedal goes as one over the speed squared from the last point short of the stop
    check('minimum control speed, KCAS', v, 85.0, 5.0, '%.1f')


def upset(f, flap=0.0, gear=0, judge=True):
    # NASA's text says takeoff flaps, gear down, but its own traces say clean: alpha 8.4 and 5.4 (Mod II) are the cruise line of [SIM II] figs. 11-12,
    # and its airspeed falls as a loss of about 115 lbf, not the 180 a motor takeoff flaps and gear need. The table model, flown clean, lands on them
    print('\nLeft motor lost, no pilot, 6,000 ft, 95 KCAS, flaps %g, gear %s [20240006682 figs. 6-9]' % (flap, 'down' if gear else 'up'))
    clear(f)
    t = H.trim(f, 95, flap=flap, gear=gear, hp=6000.0, rpm=2500.0)
    print('  trim: alpha %.1f (NASA 8.4), lever %.2f, %.0f kW, %.0f lbf a motor' % (t['alpha'], t['lever'], t['power_kw'], t['thrust'] / 2))
    lat = asym(f, 95, flap, gear, 6000.0, 2500.0, t['lever'], 0, 0.0)      # the centre of gravity is off to the left: hold the wings level first
    a = math.radians(t['alpha'])
    H.freeze(f, False)
    f['ic/h-sl-ft'] = 6000.0
    f['ic/terrain-elevation-ft'] = 0.0
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, a, 0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = t['V'] * math.cos(a), 0, t['V'] * math.sin(a)
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
    H.configure(f, flap, gear, 0, 2500.0, t['lever'], stab=t['stab'], ail=lat['ail'], rud=lat['rud'])
    f.run_ic()
    f['ctl/hold-props'] = 0
    for _ in range(int(2.5 * H.HZ)):
        f.run()
    h0 = f['attitude/psi-deg']
    r = []
    f['ctl/motor-on-l'] = 0
    for i in range(int(3 * H.HZ)):
        f.run()
        r.append((i / H.HZ, f['attitude/phi-deg'], f['attitude/theta-deg'], f['attitude/psi-deg'] - h0, math.degrees(f['velocities/p-aero-rad_sec']), f['aero/beta-deg'], -f['velocities/v-down-fps'] * -1, f['aero/alpha-deg'], f['prop/l/thrust-lbs'], f['prop/l/rpm']))
    r = np.array(r)
    print('  left disc after 3 s: %.0f lbf at %.0f rpm; alpha %.1f to %.1f' % (r[-1, 8], r[-1, 9], r[:, 7].min(), r[:, 7].max()))
    psi = r[-1, 3] if r[-1, 3] < 180 else r[-1, 3] - 360
    if not judge:
        print('  at 3 s: bank %.0f (-87), pitch %.1f from %.1f (-11.5 from 8), heading %.1f (-23), roll rate %.0f (-47), sideslip %.1f (9.1), sink %.1f (24.5)' % (r[-1, 1], r[-1, 2], math.degrees(a), psi, r[:, 4].min(), np.abs(r[:, 5]).max(), r[-1, 6]))
        f['ctl/hold-props'] = 1
        H.freeze(f)
        clear(f)
        return
    check('bank at 3 s', r[-1, 1], -87.0, 15.0)
    check('pitch at 3 s', r[-1, 2], -11.5, 5.0, '%.1f')
    check('heading at 3 s', psi, -23.0, 5.0, '%.1f')
    check('peak roll rate, deg/s', r[:, 4].min(), -47.0, 10.0)
    check('peak sideslip', float(np.abs(r[:, 5]).max()), 9.1, 2.0, '%.1f')
    check('sink at 3 s, ft/s', r[-1, 6], 24.5, 6.0, '%.1f')
    f['ctl/hold-props'] = 1
    H.freeze(f)
    clear(f)


def contactor(f):
    print('\nContactor failure on the right wing, 58 KCAS, flaps 30, the CFD\'s thrust [TM figs. 47-51, read by eye]')
    s = next(x for x in C.L('highlift_settings') if x['kcas'] == 58.0)
    f['bu/test/hl-thrust'], f['bu/test/hl-torque'], f['ctl/hl-rpm-test'] = s['T_lbf'], s['Q_lbft'], s['rpm']
    res = {}
    for name, c in (('A', 'ar'), ('B', 'br')):
        for a in (2.0, 8.0):
            clear(f)
            H.configure(f, flap=30, gear=0, lever=0.0, stab=0.0, hl_mode=3)
            f['ctl/one-aileron'] = 0
            H.air(f, s['v_fts'], a, hp=2500.0)
            H.air(f, s['v_fts'], a, hp=2500.0)
            m0 = {k: f['aero/coef/' + k] for k in ('CL', 'Cl', 'Cn', 'CY', 'Cm')}
            f['ctl/contactor-' + c] = 0
            H.air(f, s['v_fts'], a, hp=2500.0)
            H.air(f, s['v_fts'], a, hp=2500.0)
            m = {k: f['aero/coef/' + k] for k in ('CL', 'Cl', 'Cn', 'CY', 'Cm')}
            res[name, a] = {k: m[k] - m0[k] for k in m}
            print('  failure %s, alpha %g: dCL %+.3f, Cl %+.3f, Cn %+.4f, CY %+.3f, dCm %+.3f; %.2f of the discs working' % (name, a, *[res[name, a][k] for k in ('CL', 'Cl', 'Cn', 'CY', 'Cm')], f['prop/hl/on']))
    f['bu/test/hl-thrust'] = -999
    clear(f)
    check('failure A, dCL at 2 deg', res['A', 2.0]['CL'], -0.30, 0.10, '%.3f')
    check('failure A, dCL at 8 deg', res['A', 8.0]['CL'], -0.35, 0.10, '%.3f')
    check('failure A, Cl at 2 deg', res['A', 2.0]['Cl'], 0.052, 0.25, '%.3f', True)
    check('failure A, Cl at 8 deg', res['A', 8.0]['Cl'], 0.065, 0.25, '%.3f', True)
    check('failure B, Cl at 8 deg', res['B', 8.0]['Cl'], 0.060, 0.25, '%.3f', True)
    check('failure A, Cn at 8 deg, inside the solvers', res['A', 8.0]['Cn'], 0.0, 0.006, '%.4f')


def fly(f, flap, gear, kcas, hp, rpm, lever, mode=0, secs=2.5):
    """a trim, then free flight with the rotors live"""
    clear(f)
    t = H.trim(f, kcas, flap=flap, gear=gear, hl_mode=mode, hp=hp, rpm=rpm, lever=lever, x0=[6, -3, 0.0])
    a, g = math.radians(t['alpha']), math.radians(t.get('gamma', 0.0))
    H.freeze(f, False)
    f['ic/h-sl-ft'] = hp
    f['ic/terrain-elevation-ft'] = 0.0
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, a + g, 0
    f['ic/u-fps'], f['ic/v-fps'], f['ic/w-fps'] = t['V'] * math.cos(a), 0, t['V'] * math.sin(a)
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
    H.configure(f, flap, gear, mode, rpm, lever, stab=t['stab'])
    f.run_ic()
    f['ctl/hold-props'] = 0
    for _ in range(int(secs * H.HZ)):
        f.run()


def reading(f):
    return {s: dict(T=f['prop/%s/thrust-lbs' % s], nm=f['prop/%s/cmd-nm' % s], pitch=f['prop/%s/pitch-deg' % s], rpm=f['prop/%s/rpm' % s], side=f['prop/%s/inplane-lbs' % s],
                    q=f['prop/%s/q-air-ftlb' % s]) for s in 'lr'}


def failures(f3, f4):
    """every row of NASA's table 1, from full power at 97 KCAS, flaps 10; each judged on what its row says, by the property the row is about"""
    import propulsion as PR
    print('\nNASA\'s failure list [20240006496 table 1], Mod III at 97 KCAS, full power, 4 s after each')

    def lever(f, x):
        f['/controls/engines/engine[0]/throttle'] = x
        f['/controls/engines/engine[1]/throttle'] = x

    def row(name, setup, ok, say, then=None, secs=4.0):
        fly(f3, 10, 1, 97, 2500.0, 2700.0, 1.0)
        b = reading(f3)
        setup(f3)
        if then:
            then(f3)
        for _ in range(int(secs * H.HZ)):
            f3.run()
        a = reading(f3)
        good = ok(b['l'], a['l'], b['r'], a['r'])
        if not good:
            fails.append(name)
        print('  %-44s %s  %s' % (name, say(b['l'], a['l'], b['r'], a['r']), 'ok' if good else 'MISS'))
        f3['ctl/hold-props'] = 1
        H.freeze(f3)
        clear(f3)

    def set_(**k):
        def go(f):
            for n, x in k.items():
                f['ctl/' + n.replace('_', '-')] = x
        return go
    tq = lambda bl, al, br, ar: 'torque %.0f to %.0f Nm, thrust %.0f to %.0f lbf, %.0f rpm, pitch %.1f to %.1f' % (bl['nm'], al['nm'], bl['T'], al['T'], al['rpm'], bl['pitch'], al['pitch'])
    down = lambda f: lever(f, 0.5)
    row('Controller fails, half power', set_(cmc_la=0), lambda bl, al, br, ar: abs(al['nm'] / bl['nm'] - 0.5) < 0.02 and al['T'] < 0.8 * bl['T'], tq)
    row('Motor fails, propeller forces remain', set_(motor_on_l=0), lambda bl, al, br, ar: al['nm'] == 0 and al['T'] < 0 and al['rpm'] < bl['rpm'] and al['pitch'] < bl['pitch'] - 3, tq)
    row('Motor fails, no propeller forces', lambda f: (set_(motor_on_l=0)(f), set_(no_prop_forces_l=1)(f)), lambda bl, al, br, ar: al['nm'] == 0 and al['T'] == 0 and al['side'] == 0 and al['q'] == 0, tq)
    row('Fixed pitch, lever to half', set_(hub_fixed_l=1), lambda bl, al, br, ar: abs(al['pitch'] - bl['pitch']) < 0.01 and abs(ar['pitch'] - br['pitch']) > 0.3, tq, down)
    row('Half power, hub frozen', lambda f: (set_(cmc_la=0)(f), set_(hub_fixed_l=1)(f)), lambda bl, al, br, ar: abs(al['nm'] / bl['nm'] - 0.5) < 0.02 and abs(al['pitch'] - bl['pitch']) < 0.01, tq)
    row('Zero power, hub frozen', lambda f: (set_(motor_on_l=0)(f), set_(hub_fixed_l=1)(f)), lambda bl, al, br, ar: al['nm'] == 0 and abs(al['pitch'] - bl['pitch']) < 0.01, tq)
    row('Instant feather', set_(feather_now_l=1), lambda bl, al, br, ar: al['pitch'] > 80 and al['T'] < 0.2 * bl['T'], tq, secs=0.1)
    row('Command stuck, one motor, lever to half', set_(cmd_stuck_l=1), lambda bl, al, br, ar: abs(al['nm'] - bl['nm']) < 0.01 and ar['nm'] < 0.7 * br['nm'], lambda bl, al, br, ar: 'left %.0f to %.0f Nm, right %.0f to %.0f' % (bl['nm'], al['nm'], br['nm'], ar['nm']), down)
    row('Command stuck, both, lever to half', lambda f: (set_(cmd_stuck_l=1)(f), set_(cmd_stuck_r=1)(f)), lambda bl, al, br, ar: abs(al['nm'] - bl['nm']) < 0.01 and abs(ar['nm'] - br['nm']) < 0.01, lambda bl, al, br, ar: 'left %.0f to %.0f Nm, right %.0f to %.0f' % (bl['nm'], al['nm'], br['nm'], ar['nm']), down)
    row('Runaway to feather', set_(feather_l=1), lambda bl, al, br, ar: al['pitch'] > bl['pitch'] + 3 and al['T'] < bl['T'], tq)
    row('Runaway to fine', set_(hub_fine_l=1), lambda bl, al, br, ar: al['pitch'] < bl['pitch'] - 3 and al['rpm'] > bl['rpm'], tq)
    print('  Mod IV, 75 KCAS, flaps 30, high lift on')

    def hl(name, off, n):
        fly(f4, 30, 1, 75, 2500.0, 2250.0, 0.6, 1)
        for k in off:
            f4['ctl/contactor-' + k] = 0
        for _ in range(int(2 * H.HZ)):
            f4.run()
        T = {i: f4['prop/hl/d%d/thrust-lbs' % i] for i in range(1, 13)}
        rpm = {i: f4['prop/hl/d%d/rpm' % i] for i in range(1, 13)}
        dead = [i for i in T if rpm[i] < 1000 or T[i] <= 0.5]
        good = len(dead) == n and all(PR.SIDE_OF[i] == 'l' for i in dead)
        if not good:
            fails.append(name)
        print('  %-44s %d discs down, on the left: %s, thrust left %.0f, right %.0f lbf  %s' % (name, len(dead), dead, sum(T[i] for i in T if i <= 6), sum(T[i] for i in T if i > 6), 'ok' if good else 'MISS'))
        f4['ctl/hold-props'] = 1
        H.freeze(f4)
        clear(f4)
    hl('3 high-lift motors, same wing', ['bl'], 3)
    hl('6 high-lift motors, same wing', ['al', 'bl'], 6)


def park(f, stab_cmd=None, z_in=None):
    """on its wheels at rest. JSBSim's height is the centre of gravity's"""
    f['ic/terrain-elevation-ft'] = -3000.0
    f['ic/h-sl-ft'] = 3000.0
    H.freeze(f)
    f.run_ic()
    f.run()
    H.freeze(f, False)
    z = (f['inertia/cg-z-in'] - z_in) / 12
    f['ic/terrain-elevation-ft'] = 0.0
    f['ic/h-sl-ft'] = z - 0.1
    f['ic/psi-true-rad'], f['ic/theta-rad'], f['ic/phi-rad'] = 0, 0, 0
    f['ic/u-fps'] = f['ic/v-fps'] = f['ic/w-fps'] = 0
    f['ic/p-rad_sec'] = f['ic/q-rad_sec'] = f['ic/r-rad_sec'] = 0
    f.run_ic()


def centred(psi):
    return psi - 360 if psi > 180 else psi


def field():
    """the P2006T's takeoff and landing rolls, 1180 kg, sea level, ISA. Paved, by the manual's own corrections. Lift-off 65 KIAS by its table; landing flaps, idle, a touchdown at 58 KIAS and 2 s before the brakes come on, both assumed"""
    import p3sim as P3
    print('\nThe P2006T on a paved runway, 1180 kg, sea level')
    f = P3.load(kg=1180.0, cg=0.25)
    f['bu/test/frozen'] = 0
    f['ctl/hold-now'] = 0
    P3.configure(f, 20.0, 1.0, -8.0, (0.0, 0.0))
    f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 1
    park(f, z_in=26.6)
    for _ in range(int(5 * H.HZ)):
        f.run()
    print('  parked: pitch %.1f, centre of gravity %.2f ft up' % (f['attitude/theta-deg'], f['position/h-agl-ft']))
    f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 0
    P3.configure(f, 20.0, 1.0, -8.0, (1.0, 1.0))
    x = 0.0
    t = 0.0
    while t < 60:
        f.run()
        t += 1 / H.HZ
        x += f['velocities/u-fps'] / H.HZ
        f['fcs/steer-cmd-norm'] = -0.02 * centred(f['attitude/psi-deg'])
        if abs(t - round(t / 4) * 4) < 0.5 / H.HZ:
            print('    %4.1f s, %3.0f KIAS, %4.0f m: thrust %.0f lbf, tyres %.0f lbf, wheels carry %.0f lbf, pitch %.1f' % (t, f['velocities/vc-kts'], x * 0.3048, f['prop/l/thrust-lbs'] + f['prop/r/thrust-lbs'], f['bu/tyre/drag'], -f['forces/fbz-gear-lbs'], f['attitude/theta-deg']))
        if f['velocities/vc-kts'] > 52:
            f['fcs/elevator-cmd-norm'] = (-15.0 + 5) / 10
        if f['gear/unit[1]/WOW'] == 0 and f['gear/unit[2]/WOW'] == 0:
            break
    print('  lift-off at %.0f KIAS after %.1f s' % (f['velocities/vc-kts'], t))
    check('takeoff run, m', x * 0.3048, 273.0, 0.2, '%.0f', True)
    P3.configure(f, 40.0, 1.0, -5.0, (0.0, 0.0))
    park(f, z_in=26.6)
    f['ic/u-fps'] = 58 * 1.6878
    f.run_ic()
    f['gear/gear-cmd-norm'] = 1
    x = 0.0
    t = 0.0
    while f['velocities/u-fps'] > 0.5 and t < 60:
        f.run()
        t += 1 / H.HZ
        x += f['velocities/u-fps'] / H.HZ
        if t > 2:
            f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 1
    print('  stopped in %.1f s' % t)
    check('landing roll, m', x * 0.3048, 209.0, 0.2, '%.0f', True)


def gear(f):
    import x57data as X
    field()
    print('\nThe X-57 on its wheels, Mod III')
    clear(f)
    H.configure(f, 0, 1, 0, 2250.0, 0.0, stab=0.0)
    f['ctl/hold-props'] = 0
    f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 1
    park(f, z_in=X.GEAR['ground_z_in'] - 3.0)
    for _ in range(int(8 * H.HZ)):
        f.run()
    ld = [f['gear/unit[%d]/compression-ft' % i] for i in range(3)]
    print('  parked: strut compression nose %.2f, mains %.2f, %.2f ft; wing height %.2f ft' % (ld[0], ld[1], ld[2], f['bu/h-b'] * 31.623))
    check('parked pitch', f['attitude/theta-deg'], 0.0, 1.0, '%.2f')
    check('parked bank', f['attitude/phi-deg'], 0.0, 0.1, '%.2f')
    hub = json.load(open(os.path.join(H.HERE, 'data', 'geometry.json')))['cruise']
    tip_z = hub['hub_in'][2] - hub['diameter_in'] / 2
    z = X.GEAR['ground_z_in'] - 3.0
    y = hub['hub_in'][1] - X.GEAR['track_in'] / 2
    print('  propeller tip %.1f in above the wheels, %.1f in outboard of them' % (tip_z - z, y))
    print('  bank to the tip: %.1f deg at rest, %.1f with the struts at 2.3 g, %.1f at 2.3 g with flat tyres (7.5 in)' % tuple(math.degrees(math.atan((tip_z - z - c) / y)) for c in (3.0, 6.9, 14.4)))
    check('bank to the tip, as NASA figures it', math.degrees(math.atan((tip_z - z - 14.4) / y)), 9.6, 1.5, '%.1f')
    print('  takeoff from rest, flaps 10, full power, 88 KCAS rotation as NASA\'s pilots, 2,500 ft runway height left at sea level')
    H.configure(f, 10, 1, 0, 2700.0, 1.0, stab=-2.0)
    for _ in range(int(3 * H.HZ)):
        f.run()
    print('    held on the brakes: pitch %.1f, %.0f rpm, %.0f lbf a motor' % (f['attitude/theta-deg'], f['prop/l/rpm'], f['prop/l/thrust-lbs']))
    f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 0
    x = t = 0.0
    while t < 60:
        f.run()
        t += 1 / H.HZ
        x += f['velocities/u-fps'] / H.HZ
        f['fcs/steer-cmd-norm'] = -0.02 * centred(f['attitude/psi-deg'])
        if abs(t - round(t / 4) * 4) < 0.5 / H.HZ:
            print('    %4.1f s, %3.0f KCAS, %4.0f m: %.0f + %.0f lbf, %.0f rpm, %.0f Nm, pitch deg %.1f, tyres %.0f, gear x %.0f, aero x %.0f, wheels %.0f, theta %.1f' % (t, f['velocities/vc-kts'], x * 0.3048, f['prop/l/thrust-lbs'], f['prop/r/thrust-lbs'], f['prop/l/rpm'], f['prop/l/cmd-nm'], f['prop/l/pitch-deg'], f['bu/tyre/drag'], f['forces/fbx-gear-lbs'], f['forces/fbx-aero-lbs'], -f['forces/fbz-gear-lbs'], f['attitude/theta-deg']))
        if f['velocities/vc-kts'] > 88:
            f['fcs/elevator-cmd-norm'] = (-9.0 + 5) / 10
        if f['gear/unit[1]/WOW'] == 0 and f['gear/unit[2]/WOW'] == 0:
            break
    print('    lift-off at %.0f KCAS, %.0f m, %.1f s, pitch %.1f, heading %.1f' % (f['velocities/vc-kts'], x * 0.3048, t, f['attitude/theta-deg'], f['attitude/psi-deg']))
    f['ctl/hold-props'] = 1
    H.freeze(f)


def main():
    what = sys.argv[1:] or ['ground', 'climb', 'vmc', 'upset', 'contactor', 'failures', 'gear']
    f3, f4 = H.load('mod3'), H.load('mod4')
    for w in what:
        if w in ('ground', 'climb', 'failures'):
            globals()[w](f3, f4)
        elif w == 'contactor':
            contactor(f4)
        else:
            globals()[w](f3)
            if w == 'upset':
                upset(f3, 10.0, 1, False)
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else '%d misses: %s' % (len(fails), '; '.join(fails))))


if __name__ == '__main__':
    main()
