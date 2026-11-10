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
  - that each loop plays at its own pitch, its rpm over the one it was rendered at, wherever it can be heard (SimGear adds 1.0
    to a <pitch> that states no <offset>, and plays nothing over 2.0);
  - that the airframe's sounds play at their stated levels, the gear's loops at PyWopWop's only with the gear out, the tyres' only rolling, and the touchdowns once a wheel.
"""
import os, re, sys
import numpy as np
import scipy.io.wavfile as wav
import xml.etree.ElementTree as ET

os.environ.setdefault('X57_BUILT', '1')      # flown on what ships
HERE = os.path.dirname(os.path.abspath(__file__))
S = os.path.join(HERE, '..', 'X-57')
LIMIT = 64
HEARD = 1e-3            # -60 dB of a loop's full volume, as x57sound.py's gates
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
    if a.tag not in ('property', 'value') or b.tag not in ('property', 'value'):
        raise ValueError('SimGear refuses a comparison of %s and %s' % (a.tag, b.tag))
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
                        door=DOORS[0] in ET.tostring(e.find('condition'), encoding='unicode'), pitch=e.findall('pitch'),
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


def sg_pitch(l, P):
    """the pitch SimGear plays a loop at, and its own. SimGear (xmlsound.cxx) multiplies the <pitch> blocks' factor x property, each
    within its <min> and <max>, and adds their <offset>s, which are 1.0 where none is given (a <volume>'s are 0); then it holds the
    sum within 0.01 and 2.0 (sample.hxx). A loop's own pitch is its rpm over the rpm it was rendered at, the factor x property"""
    n = len(next(iter(P.values())))
    sg, own, offset = np.ones(n), np.ones(n), 0.0
    for p in l['pitch']:
        q = np.broadcast_to(val(p.find('property'), P) * float(p.findtext('factor')), (n,))
        own = own * q
        sg = sg * np.clip(q, float(p.findtext('min') or 0.0), float(p.findtext('max') or np.inf))
        offset += float(p.findtext('offset') or 1.0)
    return np.clip(offset + sg, 0.01, 2.0), own


# the most each propeller's rpm can reach in the flight model: the cruise propellers' integration limit, the high-lift discs' free spin
RPM_MAX = (('x57/cruise', 3400.0), ('x57/hl', 6500.0))


def off_pitch(L, P, C, V):
    """the loops that play off their own pitch, or need more than SimGear can play, anywhere they can be heard; for each, its worst"""
    bad, over = {}, {}
    for i, l in enumerate(L):
        rp = [q for q in names([l]) if q.endswith('/rpm')][0]
        top = next(m for k, m in RPM_MAX if k in rp)
        heard = C[i] & (V[i] > HEARD) & (P[rp] <= top)
        if not heard.any():
            continue
        sg, own = sg_pitch(l, P)
        err = np.abs(sg / np.clip(own, 0.01, 2.0) - 1.0)[heard]
        if err.max() > 1e-3:
            bad[l['name']] = float(err.max())
        if own[heard].max() > 2.0:
            over[l['name']] = float(own[heard].max())
    return bad, over


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


def loads(mod):
    """what SimGear loads (props/condition.cxx): a condition of and, or, not, a property and comparisons, each comparing a property or a value.
    Anything else and FlightGear drops the whole sound, saying so only in its log"""
    CMP = ('greater-than', 'less-than', 'equals', 'greater-than-equals', 'less-than-equals', 'not-equals')

    def ok(e):
        if e.tag in ('and', 'or', 'not'):
            return all(ok(c) for c in e)
        if e.tag == 'property':
            return True
        return e.tag in CMP and len(e) == 2 and all(c.tag in ('property', 'value') for c in e)
    root = ET.parse(os.path.join(S, 'Sounds', 'x57-%s-sound.xml' % mod)).getroot()
    bad = [e.find('name').text for e in root.find('fx') if e.find('condition') is not None and not all(ok(c) for c in e.find('condition'))]
    check('every sound\'s condition is one FlightGear loads', not bad, '%d refused, e.g. %s' % (len(bad), bad[:2]) if bad else '%d sounds' % len(root.find('fx')))


def audit(mod):
    loads(mod)
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
    bad, over = off_pitch(L, P, C, V)
    worst = max(bad, key=bad.get) if bad else None
    check('every loop plays at its own pitch wherever it can be heard', not bad,
          '%d off, worst %s by %.0f %%' % (len(bad), worst, 100 * bad[worst]) if bad else '')
    check('no loop needs a pitch over SimGear\'s 2.0 to keep it', not over, '%d: %s' % (len(over), sorted(over)[:3]) if over else '')
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
        bad, _ = off_pitch(L, P, C, V)
        check('%s: every loop heard along the flight at its own pitch' % label, not bad, '%d off: %s' % (len(bad), sorted(bad)[:3]) if bad else '')
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
    f['/controls/gear/brake-left'] = 1
    f['/controls/gear/brake-right'] = 1
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
            v = val(list(e.find('expression'))[0], {'sim/current-view/internal': inside})
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
    # the tyres rolling: each wheel's loops on its own roll speed, only while it carries weight
    T = [e for e in root if e.tag == 'tyre']
    tman = json.load(open(os.path.join(bank + '_tyres', 'manifest.json')))
    check('each view has its tyre loops, their files there', len(T) == len(tman['files']) and len(T) > 0
          and all(os.path.exists(os.path.join(S, e.find('path').text)) for e in T))
    kt = np.r_[0.0, np.linspace(0.05, 120.0, 2400) + 0.0013]      # off the windows' exact edges, where a 6-digit edge may hold a loop for an instant
    one = np.ones_like(kt)
    sag = {t['gear']: t['static_compression_ft'] for t in tman['tyres']}

    def roll(wheels, inside, wow=1.0):
        P = {'sim/current-view/internal': inside * one}
        for g in range(3):
            P['/gear/gear[%d]/rollspeed-ms' % g] = kt / 1.943846 if g in wheels else 0 * kt
            P['/gear/gear[%d]/wow' % g] = wow * one
            P['/gear/gear[%d]/compression-norm' % g] = sag[g] * one
        return P
    tc = lambda e, P: np.broadcast_to(test(list(e.find('condition'))[0], P), kt.shape)
    tv = lambda e, P: np.broadcast_to(val(list(e.find('volume').find('expression'))[0], P), kt.shape)
    still = sum(int(tc(e, roll((0, 1, 2), i))[0]) for e in T for i in (0.0, 1.0))
    air = sum(int(tc(e, roll((0, 1, 2), i, wow=0.0)).sum()) for e in T for i in (0.0, 1.0))
    check('tyres: none runs standing still or in the air', still == 0 and air == 0, 'standing %d, in the air %d' % (still, air))
    worst, nil, cut, most = 0.0, 0, 0, 0
    for inside, view in ((1.0, 'cockpit'), (0.0, 'side')):
        P = roll((0, 1, 2), inside)
        most = max(most, int(np.sum([tc(e, P) for e in T], axis=0).max()))
        for t in tman['tyres']:
            L = [e for e in T if ('-%s-' % t['name']) in e.find('name').text and e.find('name').text.endswith(view)]
            C = np.array([tc(e, P) for e in L])
            V = np.array([tv(e, P) for e in L])
            nil += int((C & (V <= 1e-6)).sum())
            cut += int(((~C) & (V > 0.01)).sum())
            for m in (q for q in tman['files'] if q['view'] == view and q['tyre'] == t['name']):
                i = int(np.argmin(np.abs(kt - m['groundspeed_kt'])))
                lf = X.file_level(os.path.join(S, 'Sounds', 'pywopwop', m['file']))
                worst = max(worst, abs(full + lf + 10 * np.log10((V[:, i] ** 2 * C[:, i]).sum()) - m['dba']))
    check('tyre loops: PyWopWop\'s levels at their speeds', worst < 0.1, 'worst %.3f dB' % worst)
    check('tyre loops: none held at nil, none cut off, six at most', nil == 0 and cut == 0 and most <= tman['max_sources'], 'nil %d, cut %d, most %d' % (nil, cut, most))
    # the touchdowns, once each as a wheel's weight comes on: a cross-fade of each wheel's one-shots over the slip and sink rate the flight model
    # latched then, up to four squeals and two thumps. Each wheel's files are one waveform under different envelopes (PyWopWop's tyre.py), so they
    # blend by amplitude into one event: the summed waveform is what's heard, and its exposure is tested against the law
    Dn = [e for e in root if e.tag == 'touchdown']
    dman = json.load(open(os.path.join(bank + '_touchdown', 'touchdown.json')))
    byfile = {q['file']: q for q in dman['files']}
    check('each wheel has its touchdown sounds, their files there', len(Dn) == len(dman['files']) and len(Dn) > 0
          and all(os.path.exists(os.path.join(S, e.find('path').text)) for e in Dn) and all(e.find('mode').text == 'once' for e in Dn))
    flat = np.ones(1)

    def landed(n, inside, slip, sink, wow=1.0):
        slip, sink = np.atleast_1d(np.asarray(slip, float)), np.atleast_1d(np.asarray(sink, float))
        P = {'sim/current-view/internal': inside * np.ones_like(slip)}
        for g in range(3):
            P['/gear/gear[%d]/wow' % g] = wow * np.ones_like(slip)
            P['/fdm/jsbsim/gear/unit[%d]/touchdown-slip-kt' % g] = slip if g == n else 0 * slip
            P['/fdm/jsbsim/gear/unit[%d]/touchdown-sink-fps' % g] = sink if g == n else 0 * sink
        return P
    dc = lambda e, P: np.broadcast_to(test(list(e.find('condition'))[0], P), next(iter(P.values())).shape)
    dv = lambda e, P: np.broadcast_to(val(list(e.find('volume').find('expression'))[0], P), next(iter(P.values())).shape)
    G, K = np.meshgrid(np.linspace(0.0, 130.0, 131), np.r_[0.0, np.geomspace(0.05, 8.0, 50)])
    start = sum(int(dc(e, landed(g, i, 0.0, 0.0)).any()) for e in Dn for g in range(3) for i in (0.0, 1.0))
    air = sum(int(dc(e, landed(g, i, G.ravel(), K.ravel(), wow=0.0)).any()) for e in Dn for g in range(3) for i in (0.0, 1.0))
    check('touchdowns: none on a start with the weight on, none in the air', start == 0 and air == 0, 'start %d, air %d' % (start, air))
    waves, peak = {}, 0

    def wave(e):
        fn = e.find('path').text
        if fn not in waves:
            fs_, y = wav.read(os.path.join(S, fn))
            waves[fn] = y.astype(np.float64) / 32768.0
        return waves[fn]

    def mix(on):              # the files that play, each at its volume and to its own end, as FlightGear plays them: a gentle touchdown's are longer
        n_ = max(len(wave(q)) for q, _ in on)
        return sum(np.pad(w * wave(q), (0, n_ - len(wave(q)))) for q, w in on)

    def heard_db(x):          # A-weighted exposure of a waveform on the scale, as x57sound.file_level reads a file
        Y = np.fft.rfft(x)
        ya = np.fft.irfft(Y * 10 ** (X.a_weight_db(np.maximum(np.fft.rfftfreq(len(x), 1 / 48000.0), 1e-3)) / 20), len(x))
        return full + 10 * np.log10(np.sum(ya ** 2) / 48000.0 + 1e-30)
    over, many, worst_node, worst_mid, mono, indep, mid_at = 0, 0, 0.0, 0.0, 0, 1.0, ''
    for g in range(3):
        for inside, view in ((1.0, 'cockpit'), (0.0, 'side')):
            for kind in ('squeal', 'thump'):
                L = [e for e in Dn if '_g%d_%s_' % (g, kind) in e.find('name').text and e.find('name').text.endswith('_' + view)]
                if not L:
                    continue
                P = landed(g, inside, G.ravel(), K.ravel())
                C = np.array([dc(e, P) for e in L])
                V = np.array([dv(e, P) for e in L]) * C
                over += int((V > 1.0 + 1e-9).sum())
                many = max(many, int(C.sum(axis=0).max()))
                # at each node its own file alone, at the manifest's level: the law for a squeal, the limited level for a thump
                for e in L:
                    m = byfile[os.path.basename(e.find('path').text)]
                    Q = landed(g, inside, m.get('slip_kt', 60.0), m['sink_fps'])
                    on = [q for q in L if dc(q, Q)[0]]
                    if on != [e]:
                        over += 1000
                        continue
                    want = m['played_dba'] if kind == 'thump' else m['law_dba']
                    worst_node = max(worst_node, abs(heard_db(wave(e) * dv(e, Q)[0]) - want))
                # between nodes: the blend, summed as heard, against the law between them (squeals: K + 20 log slip, from the two nodes'
                # law; thumps: between the two nodes' played levels, rising with the sink rate)
                ms = [byfile[os.path.basename(e.find('path').text)] for e in L]
                axis = sorted({q.get('slip_kt') for q in ms if q.get('slip_kt')}) if kind == 'squeal' else [None]
                sinks = sorted({q['sink_fps'] for q in ms})
                if kind == 'squeal':
                    for (u0, u1) in zip(axis, axis[1:]):
                        for kk in sinks:
                            for t in (0.25, 0.5, 0.75):
                                u = u0 * (u1 / u0) ** t
                                Q = landed(g, inside, u, kk * 1.13)
                                on = [(q, dv(q, Q)[0]) for q in L if dc(q, Q)[0]]
                                x = mix(on)
                                law0 = next(q['law_dba'] for q in ms if q['slip_kt'] == u0 and q['sink_fps'] == kk)
                                d_ = abs(heard_db(x) - (law0 + 20 * np.log10(u / u0)))
                                if d_ > worst_mid:
                                    worst_mid, mid_at = d_, 'gear %d %s, %.1f kt, %.2f fps' % (g, view, u, kk * 1.13)
                else:
                    lv = []
                    for kk in np.geomspace(sinks[0], sinks[-1], 25):
                        Q = landed(g, inside, 60.0, kk)
                        on = [(q, dv(q, Q)[0]) for q in L if dc(q, Q)[0]]
                        lv.append(heard_db(mix(on)))
                    mono += int((np.diff(lv) < -0.05).sum())
    # all three wheels down at once, between nodes on both axes: the sources the touchdowns take then
    for inside in (1.0, 0.0):
        n = sum(int(dc(e, landed(gg, inside, 47.0, 1.3))[0]) for gg in range(3) for e in Dn if '_g%d_' % gg in e.find('name').text)
        peak = max(peak, n)
    # the two mains are independent tyres: their files at one node don't add coherently
    for e in Dn:
        nm = e.find('name').text
        if '_g1_squeal_' in nm and nm.endswith('_cockpit'):
            twin = next((q for q in Dn if q.find('name').text == nm.replace('_g1_', '_g2_')), None)
            if twin is not None:
                a_, b_ = wave(e), wave(twin)
                k_ = min(len(a_), len(b_))
                indep = min(indep, 1.0 - abs(np.corrcoef(a_[:k_], b_[:k_])[0, 1]))
    check('touchdowns: none over full volume, four squeals and two thumps at most a wheel', over == 0 and many <= 4, 'over %d, most %d a kind' % (over, many))
    check('touchdowns: each node alone plays at PyWopWop\'s level', worst_node < 0.1, 'worst %.3f dB' % worst_node)
    # within 1 dB, the step the ear just notices: the residual is where neighbouring envelopes don't overlap, worst on long gentle slides
    check('touchdowns: a squeal blended between nodes keeps the energy law', worst_mid < 1.0, 'worst %.2f dB, %s' % (worst_mid, mid_at))
    check('touchdowns: a firmer touchdown always thumps harder', mono == 0, '%d steps down' % mono)
    check('touchdowns: the two mains are two tyres, not one file twice', indep > 0.7, 'correlation %.2f' % (1 - indep))
    check('touchdowns: all three wheels at once leave room in SimGear\'s 128 sources', peak + LIMIT + 6 <= 120,
          '%d one-shots, with the loops\' %d at most and the tyres\' 6: %d' % (peak, LIMIT, peak + LIMIT + 6))

if __name__ == '__main__':
    for mod in ('mod3', 'mod4'):
        L = audit(mod)
        doors(mod, L)
        flights(mod, L)
        parked(mod, L)
        airframe(mod)
    print('\n%s' % ('ALL CHECKS PASS' if not fails else 'FAILED: ' + '; '.join(fails)))
    sys.exit(1 if fails else 0)
