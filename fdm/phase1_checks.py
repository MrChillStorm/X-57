#!/usr/bin/env python3
"""Phase 1 checks: the table model, through JSBSim itself, against NASA's CFD and NASA's predictions.

    python3 phase1_checks.py [cfd] [modes] [stall] [cruise] [hl]

Everything here is prediction against prediction: Mods III and IV never flew. Limits are set by how far NASA's own
solvers disagree, or stated with the check. The CFD comparison is against the mean of the solvers.
"""
import json, math, os, statistics as st, sys
import numpy as np
import harness as H

HERE = os.path.dirname(os.path.abspath(__file__))
L = lambda n: json.load(open(os.path.join(HERE, 'data', 'cfd', n + '.json')))
FQ = json.load(open(os.path.join(HERE, 'data', 'digitised', 'flying_qualities.json')))
SIX = ('CD', 'CL', 'CY', 'Cl', 'Cm', 'Cn')
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'MISS', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def coef(f):
    """the airframe's coefficients: what the CFD's power-off runs hold"""
    out = {}
    for c in SIX:
        out[c] = f['aero/coef/' + c] - f['aero/%s/cruise' % c]
    return out


def setrow(f, r, V, hl_mode=0, flap=None):
    H.configure(f, flap=r['flap'] if flap is None else flap, gear=0, lever=0.0, stab=r['stab'], hl_mode=hl_mode)
    f['ctl/one-rudder'] = 1
    f['ctl/one-rudder-deg'] = r['rud']
    f['ctl/one-aileron'] = 1
    f['ctl/one-aileron-deg'] = r['ail']
    H.air(f, V, r['alpha'], r['beta'], hp=2500.0)


def rms(x):
    return math.sqrt(st.mean(e * e for e in x))


def table_lines(groups, order):
    for g in order:
        if g in groups:
            print('  %-26s' % g + ' '.join('%s %.4f (%.4f)' % (c, rms([e for e, _ in x]), st.mean(s for _, s in x)) for c, x in groups[g].items())
                  + '  n %d' % max(len(x) for x in groups[g].values()))


def poweroff(f):
    """The neutral curves against the solvers' mean; then each input's increment, against the same solver's neutral case at
    that alpha: a solver's own offset, or the choice of curve past stall, isn't the table's error. In a run with no
    sideslip, rudder or aileron, CY, Cl and Cn should be nil: where they aren't, the CFD's wing has stalled unevenly, which
    a symmetric table can't hold, so those are left out and counted."""
    rows = [r for r in L('poweroff')['rows'] if r['flap'] in (0, 10, 30) and not r['tab']]
    stall = {0.0: 15, 10.0: 12, 30.0: 12}
    base = {(r['flap'], r['alpha']): r for r in rows if not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))}
    neutral, groups, uneven = {}, {}, 0
    for r in rows:
        late = r['alpha'] > stall[r['flap']]
        setrow(f, r, 200.0)
        m = coef(f)
        n = sum(1 for k in ('beta', 'stab', 'ail', 'rud') if r[k])
        if n == 0:
            if not late:
                for c in ('CD', 'CL', 'Cm'):
                    v = [r[s][c] for s in H_SOLVERS if s in r and c in r[s]]
                    neutral.setdefault(c, []).append((m[c] - float(np.mean(v)), max(v) - min(v)))
            continue
        z = dict(r, beta=0.0, stab=0.0, ail=0.0, rud=0.0)
        setrow(f, z, 200.0)
        m0 = coef(f)
        g = ('one input' if n == 1 else 'combined') + (' past stall' if late else '')
        b = base.get((r['flap'], r['alpha']))
        for c in SIX:
            odd = c in ('CY', 'Cl', 'Cn')
            if odd and not (r['beta'] or r['ail'] or r['rud']):
                uneven += any(s in r and c in r[s] and abs(r[s][c]) > 0.02 for s in H_SOLVERS)
                continue
            d = []
            for s in H_SOLVERS:
                if s in r and c in r[s]:
                    if b and s in b and c in b[s]:
                        d.append(r[s][c] - b[s][c])
                    elif odd:
                        d.append(r[s][c])
            if d:
                groups.setdefault(g, {}).setdefault(c, []).append((m[c] - m0[c] - float(np.mean(d)), max(d) - min(d)))
    print('Power off: model less the solvers, RMS (typical solver spread)')
    print('  %-26s' % 'neutral curves' + ' '.join('%s %.4f (%.4f)' % (c, rms([e for e, _ in x]), st.mean(s for _, s in x)) for c, x in neutral.items()))
    table_lines(groups, ('one input', 'combined', 'one input past stall', 'combined past stall'))
    print('  rows where a symmetric case shows an uneven stall: %d, left out' % uneven)
    check('neutral, below stall: reproduces the solvers\' mean', all(rms([e for e, _ in neutral[c]]) < 0.003 for c in neutral))
    o = groups['one input']
    check('one input: its increment reproduced', all(rms([e for e, _ in o[c]]) <= max(st.mean(s for _, s in o[c]), 0.004) for c in o),
          ' '.join('%s %.4f' % (c, rms([e for e, _ in o[c]])) for c in o))
    x = groups.get('combined')
    check('combined inputs: cross terms stay small', x and all(rms([e for e, _ in x[c]]) < 0.03 for c in x), ' '.join('%s %.3f' % (c, rms([e for e, _ in x[c]])) for c in x))


H_SOLVERS = ('star', 'lava', 'usm3d', 'kestrel')


def highlift(f):
    """High lift on, at the CFD's own rpm (its schedule is older than NASA's final one). The propellers' effect, on minus
    off within a solver; then each input's increment with the propellers running"""
    sets = {s['kcas']: s for s in L('highlift_settings')}
    rows = [r for r in L('highlift') if not r['tab']]
    clean = lambda r: not any(r[k] for k in ('beta', 'stab', 'ail', 'rud'))
    base = {(r['kcas'], r['flap'], r['alpha']): r for r in rows if clean(r)}
    groups = {}
    for r in rows:
        k = r['kcas']
        f['ctl/hl-rpm-test'] = sets[k]['rpm']
        setrow(f, r, sets[k]['v_fts'], hl_mode=3)
        m = coef(f)
        n = sum(1 for q in ('beta', 'stab', 'ail', 'rud') if r[q])
        band = '58-90 KCAS' if 58 <= k <= 90 else 'other speeds'
        if n == 0:
            setrow(f, r, sets[k]['v_fts'], hl_mode=0)
            m0 = coef(f)
            g = 'effect of the props, ' + band
            for c in ('CD', 'CL', 'Cm'):
                d = [r[s]['on'][c] - r[s]['off'][c] for s in H_SOLVERS if s in r and 'on' in r[s] and 'off' in r[s] and c in r[s]['on'] and c in r[s]['off']]
                if d:
                    groups.setdefault(g, {}).setdefault(c, []).append((m[c] - m0[c] - float(np.mean(d)), max(d) - min(d)))
            continue
        z = dict(r, beta=0.0, stab=0.0, ail=0.0, rud=0.0)
        setrow(f, z, sets[k]['v_fts'], hl_mode=3)
        m0 = coef(f)
        g = ('one input, ' if n == 1 else 'combined, ') + band
        b = base.get((k, r['flap'], r['alpha']))
        for c in SIX:
            odd = c in ('CY', 'Cl', 'Cn')
            if odd and not (r['beta'] or r['ail'] or r['rud']):
                continue
            d = []
            for s in H_SOLVERS:
                x = r.get(s, {}).get('on', {})
                y = (b or {}).get(s, {}).get('on', {}) if b else {}
                if c in x and c in y:
                    d.append(x[c] - y[c])
                elif c in x and odd:
                    d.append(x[c])
            if d:
                groups.setdefault(g, {}).setdefault(c, []).append((m[c] - m0[c] - float(np.mean(d)), max(d) - min(d)))
    print('High lift on: model less the solvers, RMS (typical solver spread)')
    table_lines(groups, sorted(groups))
    e = groups['effect of the props, 58-90 KCAS']
    check('the props\' effect, 58-90 KCAS: inside the solvers\' spread', all(rms([x for x, _ in e[c]]) <= max(st.mean(s for _, s in e[c]), 0.02) for c in e),
          ' '.join('%s %.3f' % (c, rms([x for x, _ in e[c]])) for c in e))
    o = groups['one input, 58-90 KCAS']
    check('one input with the props on, 58-90 KCAS: within 0.05 in lift and pitch', rms([x for x, _ in o['CL']]) < 0.05 and rms([x for x, _ in o['Cm']]) < 0.05,
          'CL %.3f Cm %.3f' % (rms([x for x, _ in o['CL']]), rms([x for x, _ in o['Cm']])))
    v = sets[58.0]['v_fts']
    f['ctl/hl-rpm-test'] = sets[58.0]['rpm']
    cl = []
    for a in np.arange(8, 19, 1.0):
        f['ctl/one-aileron'] = 0
        H.configure(f, flap=30, hl_mode=3, lever=0.0, stab=0.0)
        H.air(f, v, float(a), hp=2500.0)
        cl.append((coef(f)['CL'], float(a)))
    best = max(cl)
    check('Mod IV, 58 KCAS, flaps 30: peak CL within 0.05 of NASA\'s 4.40-4.58', 4.35 <= best[0] <= 4.63, 'CL %.2f at %g deg' % best)
    f['ctl/one-aileron'] = 0


def modes():
    out = {}
    for name, mod, hl, speeds in (('mod3', 'mod3', 0, (80, 88, 96, 104, 112)), ('mod4_fixed', 'mod4', 1, (75, 80, 88, 96, 104, 112)),
                                  ('mod4_airspeed', 'mod4', 2, (75, 80, 88, 96))):
        f = H.load(mod)
        V, E = FQ['vs_kcas'], FQ['eigenvalues']
        print('%s: model (NASA)' % name)
        err = dict(ph=[], sp=[], dr=[], tau=[], wsp=[], wdr=[], wph=[])
        for k in speeds:
            t = H.trim(f, k, flap=12, gear=1, hl_mode=hl, hp=2500)
            if t['lever'] < 0.063:             # the high-lift thrust alone is more than the drag: idle, climbing
                t = H.trim(f, k, flap=12, gear=1, hl_mode=hl, hp=2500, lever=0.0, x0=[5, -3, 3.0])
            m = H.modes(f, t)
            s = str(k)
            n = dict(ph=V['zeta_phugoid'][name][s], sp=V['zeta_short_period'][name][s], dr=V['zeta_dutch_roll'][name][s], tau=V['tau_roll_s'][name][s])
            w = {a: E[b][name].get(s, {}).get('omega') for a, b in (('wph', 'phugoid'), ('wsp', 'short_period'), ('wdr', 'dutch_roll'))}
            got = dict(ph=H.zeta(m['phugoid']), sp=H.zeta(m['short_period']), dr=H.zeta(m['dutch_roll']), tau=m['roll_tau'],
                       wph=abs(m['phugoid']), wsp=abs(m['short_period']), wdr=abs(m['dutch_roll']))
            edge = k == 80 and name == 'mod3'
            print('  %3d  zeta phugoid %.3f (%.3f)  short %.3f (%.3f)  dutch %.3f (%.3f)  roll %.2f s (%.2f)  w %.2f (%s) %.2f (%s) %.2f (%s)%s' % (
                k, got['ph'], n['ph'], got['sp'], n['sp'], got['dr'], n['dr'], got['tau'], n['tau'],
                got['wph'], '%.2f' % w['wph'] if w['wph'] else '-', got['wsp'], '%.2f' % w['wsp'] if w['wsp'] else '-',
                got['wdr'], '%.2f' % w['wdr'] if w['wdr'] else '-', '  [NASA\'s stall speed]' if edge else ''))
            if not edge:
                for q in ('ph', 'sp', 'dr'):
                    err[q].append(abs(got[q] - n[q]))
                err['tau'].append(abs(got['tau'] / n['tau'] - 1))
                for q in ('wph', 'wsp', 'wdr'):
                    if w[q]:
                        err[q].append(abs(got[q] / w[q] - 1))
        out[name] = {k: max(v) if v else 0 for k, v in err.items()}
        print('   worst: damping ratio phugoid %.3f, short period %.3f, Dutch roll %.3f; roll %.0f %%; frequency phugoid %.0f %%, short %.0f %%, Dutch %.0f %%' % (
            out[name]['ph'], out[name]['sp'], out[name]['dr'], 100 * out[name]['tau'], 100 * out[name]['wph'], 100 * out[name]['wsp'], 100 * out[name]['wdr']))
    e = out['mod3']
    check('Mod III modes match NASA\'s (damping within 0.02, frequency and roll within 8 %, stall speed aside)',
          max(e['ph'], e['sp'], e['dr']) <= 0.02 and max(e['tau'], e['wph'], e['wsp'], e['wdr']) <= 0.08)
    e = out['mod4_airspeed']
    check('Mod IV airspeed mode: damping within 0.05, frequency and roll within 20 %',
          max(e['ph'], e['sp'], e['dr']) <= 0.05 and max(e['tau'], e['wph'], e['wsp'], e['wdr']) <= 0.2)
    e = out['mod4_fixed']
    check('Mod IV fixed mode: damping within 0.05, frequency and roll within 25 %',
          max(e['ph'], e['sp'], e['dr']) <= 0.05 and max(e['tau'], e['wph'], e['wsp'], e['wdr']) <= 0.25)


def stall():
    """highest trimmed lift, lever at idle, from a sweep of alpha: the speed it gives at NASA's 3,000 lb"""
    from scipy.optimize import brentq
    f = H.load('mod3')
    f['inertia/pointmass-weight-lbs[0]'] = 0.0
    print('Trimmed stall, Mod III, 3,000 lb: model, NASA')
    res = []
    for flap, nasa, cfd in ((0, 88, (2.13, 2.34)), (10, 80, (2.28, 2.53)), (30, 73, (2.76, 3.03))):
        V = 150.0
        best = (0, 0, 0)
        for a in np.arange(4, 20, 0.5):
            H.configure(f, flap=flap, gear=0, lever=0.0, stab=-5)

            def q(s):
                H.configure(f, flap=flap, gear=0, lever=0.0, stab=s)
                return H.air(f, V, float(a), hp=2500.0)[4]
            try:
                s = brentq(q, -15, 5, xtol=0.01)
            except ValueError:
                continue
            q(s)
            cl = f['aero/coef/CL'] - f['aero/CL/cruise']
            if cl > best[0]:
                best = (cl, float(a), s)
        rho = 0.0023769
        vs = math.sqrt(2 * 3000.0 / (rho * 66.667 * best[0])) / H.FPS_KT
        print('  flaps %2d: trimmed CLmax %.2f at alpha %.1f, stabilator %.1f: %.1f KCAS (NASA %d); untrimmed CFD peaks %.2f-%.2f' % (flap, best[0], best[1], best[2], vs, nasa, *cfd))
        res.append(abs(vs - nasa))
    check('stall speeds within 5 KCAS of NASA\'s', max(res) <= 5.0, 'worst %.1f KCAS' % max(res))


def cruise():
    f = H.load('mod3')
    f['inertia/pointmass-weight-lbs[0]'] = 0.0
    t = H.trim(f, 133.0, flap=0, gear=0, hp=8000)
    ld = t['CL'] / t['CD']
    print('Cruise, 8,000 ft, 3,000 lb: %.1f KTAS, CL %.3f, CD %.4f, L/D %.1f (NASA 13.5), alpha %.2f, stabilator %.2f, %.1f kW, thrust %.0f lbf' % (
        t['V'] / H.FPS_KT, t['CL'], t['CD'], ld, t['alpha'], t['stab'], t['power_kw'], t['thrust']))
    check('L/D at 150 KTAS within 1 of NASA\'s 13.5', abs(ld - 13.5) <= 1.0)
    best = max((H.trim(f, k, flap=0, gear=0, hp=8000) for k in range(92, 134, 6)), key=lambda x: x['CL'] / x['CD'])
    print('  peak L/D %.1f (NASA 15.9) at %g KCAS, CL %.2f' % (best['CL'] / best['CD'], best['kcas'], best['CL']))
    check('peak L/D within 1.5 of NASA\'s 15.9', abs(best['CL'] / best['CD'] - 15.9) <= 1.5)


if __name__ == '__main__':
    what = sys.argv[1:] or ['cfd', 'hl', 'stall', 'cruise', 'modes']
    if 'cfd' in what:
        poweroff(H.load('mod3'))
    if 'hl' in what:
        highlift(H.load('mod4'))
    if 'stall' in what:
        stall()
    if 'cruise' in what:
        cruise()
    if 'modes' in what:
        modes()
    print('\n%s' % ('ALL WITHIN LIMITS' if not fails else 'MISSES: ' + '; '.join(fails)))
