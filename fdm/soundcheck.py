#!/usr/bin/env python3
"""Do the X-57's sounds keep to SimGear's limit? Checks on the generated XML itself.

    python3 soundcheck.py

SimGear has 128 sources for everything, and a looped sound holds one while its condition is true, however quiet. So a
sound that can't be heard must have a false condition. This evaluates every loop's real <condition> and <volume> the way
SimGear reads them, over a sweep of states and along flights through the flight model:

  - how many run at once, against LIMIT (kept well under 128, which other sounds share);
  - that no loop holds a source at nil volume, and none is cut off while it could be heard;
  - that nothing runs on a parked aircraft, or from a propeller that isn't turning;
  - that each group's loops add to one equal-power whole, so the bookkeeping hasn't lost a loop;
  - that the airframe's sounds play at their stated levels, and the gear's loops at PyWopWop's, only with the gear out.
"""
import os, re, sys
import numpy as np
import xml.etree.ElementTree as ET

os.environ.setdefault('X57_BUILT', '1')      # flown on what ships
HERE = os.path.dirname(os.path.abspath(__file__))
S = os.path.join(HERE, '..', 'X-57')
LIMIT = 64
SIG = '/environment/density-slugft3'
TAS = '/fdm/jsbsim/velocities/vtrue-kts'
DOORS = ('sim/model/door-left', 'sim/model/door-right')        # the main and rear doors, 0 shut to 1 open
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'FAIL', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def val(e, P):
    """a condition's operand or an expression's node, on arrays of states"""
    t = e.tag
    if t == 'value':
        return float(e.text)
    if t == 'property':
        return P[e.text.strip()]
    k = list(e)
    if t == 'sum':
        return sum(val(c, P) for c in k)
    if t == 'product':
        r = 1.0
        for c in k:
            r = r * val(c, P)
        return r
    if t == 'div':
        return val(k[0], P) / val(k[1], P)
    if t == 'abs':
        return np.abs(val(k[0], P))
    if t == 'sqrt':
        return np.sqrt(np.maximum(val(k[0], P), 0.0))
    if t == 'pow':
        return np.power(np.maximum(val(k[0], P), 0.0), val(k[1], P))
    if t == 'clip':
        lo, hi = float(e.find('clipMin').text), float(e.find('clipMax').text)
        body = [c for c in k if c.tag not in ('clipMin', 'clipMax')][0]
        return np.clip(val(body, P), lo, hi)
    raise ValueError('expression: ' + t)


def test(e, P):
    t = e.tag
    if t == 'and':
        r = True
        for c in e:
            r = r & test(c, P)
        return r
    if t == 'or':
        r = False
        for c in e:
            r = r | test(c, P)
        return r
    if t == 'not':
        return ~test(list(e)[0], P)
    if t == 'property':
        return P[e.text.strip()] > 0.5
    a, b = list(e)
    if t == 'greater-than':
        return val(a, P) > val(b, P)
    if t == 'less-than':
        return val(a, P) < val(b, P)
    raise ValueError('condition: ' + t)


def loops(mod):
    root = ET.parse(os.path.join(S, 'Sounds', 'x57-%s-sound.xml' % mod)).getroot()
    out = []
    for e in root.find('fx'):
        if e.tag not in ('prop', 'motor', 'hub'):     # the airframe's sounds and the wind aren't the bank's
            continue
        n = e.find('name').text.split('-')
        out.append(dict(tag=e.tag, name=e.find('name').text, group=n[-2], view=n[-1], path=e.find('path').text, mode=e.find('mode').text,
                        door=DOORS[0] in ET.tostring(e.find('condition'), encoding='unicode'),
                        cond=list(e.find('condition'))[0], vol=list(e.find('volume').find('expression'))[0]))
    return out


def evaluate(L, P):
    n = len(next(iter(P.values())))
    P = dict(P)
    for d in DOORS:                     # shut, unless the state says otherwise
        P.setdefault(d, np.zeros(n))
    C = np.zeros((len(L), n), bool)
    V = np.zeros((len(L), n))
    for i, l in enumerate(L):
        C[i] = np.broadcast_to(test(l['cond'], P), (n,))
        V[i] = np.broadcast_to(val(l['vol'], P), (n,))
    return C, V


def names(L):
    return sorted({m for l in L for e in (l['cond'], l['vol']) for m in re.findall(r'/fdm/jsbsim/x57/[^<]+', ET.tostring(e, encoding='unicode'))})


def sweep(L, n=40000, seed=3):
    """every property on its own: grid points, the middles between them, and anything else, each group apart. Worse than any
    flight, where the two sides and the twelve discs agree"""
    r = np.random.default_rng(seed)
    spd = [0, 20, 40, 50, 60, 70, 80, 90, 100, 110, 120, 125, 150, 170]
    P = {'sim/current-view/internal': r.integers(0, 2, n).astype(float), SIG: 0.00237689 * r.uniform(0.6, 1.0, n)}
    P[TAS] = np.where(r.random(n) < 0.5, r.choice(spd, n), r.uniform(0, 200, n))
    for p in names(L):
        if p.endswith('/rpm'):
            g = [0, 150, 1000, 1500, 1800, 2250, 2500, 2700, 3000, 3500, 4000, 4400, 4800, 5100, 5400]
            P[p] = np.where(r.random(n) < 0.5, r.choice(g, n), r.uniform(0, 5500, n))
        elif p.endswith('/ct'):
            P[p] = np.where(r.random(n) < 0.5, r.choice([-0.05, -0.03, -0.015, 0, 0.025, 0.05, 0.085, 0.12, 0.2], n), r.uniform(-0.06, 0.2, n))
        else:
            P[p] = r.uniform(-0.3, 1.2, n)
    for d in DOORS:                     # mostly shut; else shut, open or anywhere between
        P[d] = np.where(r.random(n) < 0.7, 0.0, np.where(r.random(n) < 0.5, r.choice([0.0, 0.005, 0.02, 1.0], n), r.uniform(0, 1, n)))
    return P


def audit(mod):
    L = loops(mod)
    print('%s: %d loops' % (mod, len(L)))
    check('every loop is looped, its file there', all(l['mode'] == 'looped' and os.path.exists(os.path.join(S, l['path'])) for l in L))
    P = sweep(L)
    C, V = evaluate(L, P)
    held = C.sum(axis=0)
    inside = P['sim/current-view/internal'] > 0.5
    print('     sweep of %d states, every group free: most at once %d in the cockpit, %d outside' % (len(held), held[inside].max(), held[~inside].max()))
    check('sources under %d, with room for what else is playing' % LIMIT, held.max() <= LIMIT)
    silent = C & (V <= 1e-6)
    check('no loop holds a source at nil volume', not silent.any(), '%d cases' % silent.sum())
    # each view's loops, and in the cockpit the outside's too while a door is open
    door = (P[DOORS[0]] > 0.01) | (P[DOORS[1]] > 0.01)
    through = np.array([l['door'] for l in L])[:, None]
    view = np.where(np.array([l['view'] == 'cockpit' for l in L])[:, None], inside[None, :], ~inside[None, :] | (through & door[None, :]))
    # a shut loop must already have faded: nothing above -40 dB, beside a neighbour at full level, is cut off. (Exactly on a
    # grid point the rounded fades leave about -68 dB, under a shut condition, which is nothing)
    cut = (~C) & view & (V > 0.01)
    check('nothing audible is cut off by its condition', not cut.any(), '%d cases' % cut.sum())
    wrong = C & ~view
    check('only the current view\'s loops run, and the outside\'s through an open door', not wrong.any())
    # all stopped: nothing, whatever the speed or view
    Z = {k: (np.zeros_like(v) if k.endswith('/rpm') else v) for k, v in P.items()}
    C0, _ = evaluate(L, Z)
    check('no propeller turning: no loop runs', not C0.any(), '%d' % C0.sum())
    # equal power: at sea level, within a group and a view, its propeller loops' squares add to one value
    Vs = evaluate(L, dict(P, **{SIG: np.full_like(P[SIG], 0.00237689)}))[1]
    worst = 0.0
    for grp in sorted({l['group'] for l in L}):
        for vw, on in (('cockpit', inside), ('side', ~inside)):
            idx = [i for i, l in enumerate(L) if l['group'] == grp and l['view'] == vw and l['tag'] == 'prop']
            rpmp = next(p for p in names([L[idx[0]]]) if p.endswith('/rpm'))
            ok = on & (P[rpmp] >= 1500)
            s = (Vs[idx][:, ok] ** 2).sum(axis=0)
            k = np.median(s)
            worst = max(worst, float(np.abs(s / k - 1).max()))
    check('each group\'s loops add to one whole', worst < 0.01, 'worst %.4f of the power' % worst)
    return L


def doors(mod, L):
    """parked with the propellers turning, in the cockpit: the outside's loops come in by how far a door is open, the rear door at
    half, and none while both are shut; outside, the doors change nothing"""
    import x57sound as X
    n = 1
    P = {TAS: np.zeros(n), SIG: np.full(n, 0.00237689), 'sim/current-view/internal': np.ones(n)}
    for p in names(L):
        P[p] = np.full(n, 1500.0 if p.endswith('/rpm') else 0.0)
    side = np.array([l['view'] != 'cockpit' for l in L])
    through = np.array([l['door'] for l in L])[side]
    check('doors: only outside loops come through a door', not any(l['door'] for l in L if l['view'] == 'cockpit'))
    out = evaluate(L, dict(P, **{'sim/current-view/internal': np.zeros(n)}))
    ref = np.where(out[0], out[1], 0.0)[side, 0]
    check('doors: the outside\'s loops are heard outside', (ref > 0).any())
    worst, shut = 0.0, 0
    for dl, dr, k in ((0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.5, 0.0, 0.5), (0.0, 1.0, X.REAR_WEIGHT), (0.0, 0.5, 0.5 * X.REAR_WEIGHT),
                      (1.0, 1.0, 1.0), (0.6, 0.6, min(1.0, 0.6 * np.hypot(1, X.REAR_WEIGHT)))):
        C, V = evaluate(L, dict(P, **{DOORS[0]: np.full(n, dl), DOORS[1]: np.full(n, dr)}))
        got = np.where(C, V, 0.0)[side, 0]
        if k == 0.0:
            shut += int(C[side].sum())
        else:
            worst = max(worst, float(np.abs(got - k * ref * through).max()))
        Co, Vo = evaluate(L, dict(P, **{'sim/current-view/internal': np.zeros(n), DOORS[0]: np.full(n, dl), DOORS[1]: np.full(n, dr)}))
        worst = max(worst, float(np.abs(np.where(Co, Vo, 0.0)[side, 0] - ref).max()))
    check('doors: shut, the cockpit hears none of the outside\'s loops', shut == 0, '%d' % shut)
    check('doors: open, the outside\'s loops come in by how far, the rear door at %g' % X.REAR_WEIGHT, worst < 1e-6, 'worst %.2g' % worst)


def flights(mod, L):
    """the same along fly.py's flight: parked, takeoff, climb, cruise, turn, approach, in the flight model itself"""
    import harness as H
    import fly
    need = names(L)
    rows = []
    orig = H.Sim.run
    state = dict(n=0)

    def run(self):
        r = orig(self)
        state['n'] += 1
        if state['n'] % 30 == 0 and self['simulation/integrator/rate/rotational'] != 0:        # not inside a trim
            rows.append([self['velocities/vtrue-kts'], self['atmosphere/rho-slugs_ft3'], self['position/h-agl-ft'], self['velocities/vc-kts']] + [self[p[len('/fdm/jsbsim/'):]] for p in need])
        return r

    H.Sim.run = run
    try:
        fly.fly(mod)
    finally:
        H.Sim.run = orig
    A = np.array(rows)
    print('     %d samples along the flight' % len(A))
    res = {}
    for label, internal in (('cockpit', 1.0), ('outside', 0.0)):
        P = {TAS: A[:, 0], SIG: A[:, 1], 'sim/current-view/internal': np.full(len(A), internal)}
        P.update({p: A[:, 4 + i] for i, p in enumerate(need)})
        C, V = evaluate(L, P)
        held = C.sum(axis=0)
        res[label] = held
        print('     %s: at most %d at once, %.0f on average' % (label, held.max(), held.mean()))
        faint = C & (V < 1e-3)
        print('     %s: %d of %d sources held under -60 dB, %d under -40 dB' % (label, faint.sum(), C.sum(), (C & (V < 1e-2)).sum()))
        check('%s: under 1 %% of the sources held are under -60 dB' % label, faint.sum() < 0.01 * C.sum())
    moving = np.argmax(A[:, 0] > 10.0)           # (not the gear settling at the start)
    check('the propellers are heard once under way', res['cockpit'][moving + 20:].min() > 0)


def parked(mod, L):
    """on the ground with everything off, then started and shut down again: how long the loops take to let go"""
    import harness as H
    need = names(L)
    f = H.load(mod)
    f['ctl/hold-props'] = 0
    H.freeze(f, False)
    f['flex/enable'] = 1
    H.configure(f, flap=0, gear=1, lever=0.0, stab=-5, hl_mode=0)
    f['ic/terrain-elevation-ft'] = 2300.0
    f['ic/h-agl-ft'] = 5.0
    for k in ('ic/vt-fps', 'ic/theta-deg', 'ic/phi-deg', 'ic/psi-true-deg', 'ic/u-fps', 'ic/v-fps', 'ic/w-fps'):
        f[k] = 0.0
    f['fcs/left-brake-cmd-norm'] = 1
    f['fcs/right-brake-cmd-norm'] = 1
    f.run_ic()

    def count():
        P = {TAS: np.array([f['velocities/vtrue-kts']]), SIG: np.array([f['atmosphere/rho-slugs_ft3']]), 'sim/current-view/internal': np.array([1.0])}
        P.update({p: np.array([f[p[len('/fdm/jsbsim/'):]]]) for p in need})
        C, V = evaluate(L, P)
        return int(C.sum()), float(V[C[:, 0], 0].min()) if C.any() else 1.0

    def run(s):
        for _ in range(int(s * H.HZ)):
            f.run()

    run(5)
    n0 = count()[0]
    check('parked, everything off: no loop runs', n0 == 0, '%d' % n0)
    f['/controls/engines/engine[0]/throttle'] = 0.2
    f['/controls/engines/engine[1]/throttle'] = 0.2
    if mod == 'mod4':
        f['ctl/hl-mode'] = 1
    run(6)
    n1 = count()[0]
    check('started: loops run', n1 > 0, '%d in the cockpit' % n1)
    f['/controls/engines/engine[0]/throttle'] = 0.0
    f['/controls/engines/engine[1]/throttle'] = 0.0
    if mod == 'mod4':
        f['ctl/hl-mode'] = 0
    t, quiet = 0.0, None
    while t < 30:
        run(0.25)
        t += 0.25
        if count()[0] == 0:
            quiet = t
            break
    check('shut down: every loop lets go of its source', quiet is not None, 'after %s s' % quiet)


def airframe(mod):
    """the sounds that aren't the bank's, read back off the XML: each at the level x57sound.py says it plays at, and the gear's loops at PyWopWop's"""
    import json
    import x57sound as X
    root = ET.parse(os.path.join(S, 'Sounds', 'x57-%s-sound.xml' % mod)).getroot().find('fx')
    bank = os.path.expanduser('~/git/PyWopWop/out/x57')
    full = X.full_scale_db(json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']['cockpit'])
    by = {e.find('name').text: e for e in root}
    worst = 0.0
    for tag, name, fn, plays, l_in, l_out, law, click, at, note in X.LEVELS:
        e = next(q for q in by[name].findall('volume') if q.find('expression') is not None)
        lf = X.file_level(os.path.join(S, 'Sounds', fn), click)
        for inside, want in ((1.0, l_in), (0.0, l_out)):
            if want is None:
                continue
            ref = X.TYRE_REF_KT if name == 'rumble' else X.TOUCH_REF_KT
            v = val(list(e.find('expression'))[0], {'sim/current-view/internal': inside, X.GS: ref, X.SINK: X.TOUCH_REF_FPS})
            worst = max(worst, abs(full + lf + 20 * np.log10(v) - want))
    check('the airframe\'s sounds play at their stated levels', worst < 0.05, 'worst %.3f dB' % worst)
    G = [e for e in root if e.tag == 'gearair']
    man = json.load(open(os.path.join(bank + '_gear', 'manifest.json')))['files']
    check('each view has its gear loops, their files there', len(G) == len(man)
          and all(os.path.exists(os.path.join(S, e.find('path').text)) for e in G))
    tas = np.linspace(0.0, 250.0, 2501)
    ones = np.ones_like(tas)
    state = lambda pos, inside: {TAS: tas, SIG: 0.00237689 * ones, 'gear/gear[0]/position-norm': pos * ones, 'sim/current-view/internal': inside * ones}
    cond = lambda e, P: np.broadcast_to(test(list(e.find('condition'))[0], P), tas.shape)
    vol = lambda e, P: np.broadcast_to(val(list(e.find('volume').find('expression'))[0], P), tas.shape)
    up = sum(cond(e, state(0.0, i)).sum() for e in G for i in (0.0, 1.0))
    check('gear up: no gear loop runs', up == 0, '%d cases' % up)
    # at each loop's own speed the view's loops add to that loop's level, and away from them nothing is held at nil or cut off
    worst, nil, cut, most = 0.0, 0, 0, 0
    for inside, view, gain in ((1.0, 'cockpit', 0.0), (0.0, 'side', 0.0)):
        P = state(1.0, inside)
        L = [e for e in G if e.find('name').text.endswith(view)]
        C = np.array([cond(e, P) for e in L])
        V = np.array([vol(e, P) for e in L])
        nil += int((C & (V <= 1e-6)).sum())
        cut += int(((~C) & (V > 0.01)).sum())
        most = max(most, int(C.sum(axis=0).max()))
        for m in (q for q in man if q['view'] == view):
            i = int(np.argmin(np.abs(tas - m['airspeed_kt'])))
            lf = X.file_level(os.path.join(S, 'Sounds', 'pywopwop', m['file']))
            worst = max(worst, abs(full + lf + 10 * np.log10((V[:, i] ** 2 * C[:, i]).sum()) - m['dba']))
    check('gear loops: PyWopWop\'s levels at their speeds', worst < 0.1, 'worst %.3f dB' % worst)
    check('gear loops: none held at nil, none cut off, two at most', nil == 0 and cut == 0 and most <= 2, 'nil %d, cut %d, most %d' % (nil, cut, most))


if __name__ == '__main__':
    for mod in ('mod3', 'mod4'):
        L = audit(mod)
        doors(mod, L)
        flights(mod, L)
        parked(mod, L)
        airframe(mod)
    print('\n%s' % ('ALL CHECKS PASS' if not fails else 'FAILED: ' + '; '.join(fails)))
    sys.exit(1 if fails else 0)
