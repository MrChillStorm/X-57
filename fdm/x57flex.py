"""The X-57's wing bending: each panel as a beam held at the H-frame, in its first bending mode. The ASK 21's method (its flexwing.py), with a heavy tip.

The beam:
  - mass: the clean wing 313.5 lb with its H-frame, 134 lb at each tip [20240011864 table 1: plates 19.5 and mass simulators 114.5 lb make the 134 lb nacelle].
    The H-frame's own 30 lb is a guess. The rest goes with the square of the chord. Mod IV adds 12 motors of about 16 lb [20240007439];
  - stiffness: EI falls with the square of the distance from the tip, down to a small floor there, E0 ((1 - s) + r)^2 / (1 + r)^2 with s the panel's span fraction. E0 and r are
    fitted to the vibration test's symmetric first bending with and without the tip masses, 2.26 and 6.55 Hz. The ASK 21's spar and shell sum can't reach both: 2.33 and 6.39 at best;
  - the root: in symmetric bending the wing's own carry-through takes the moment. In antisymmetric bending it goes through the H-frame, so the root gives in roll:
    a spring there, fitted to 2.14 Hz. The FEM has the same split, 2.11 and 2.26 Hz, with the H-frame's four points fixed;
  - damping 0.57 % of critical, the Mod II wing's measured first bending [20240007439 table 1]. The Mod III paper gives none.

What's left as tests: the clean wing's antisymmetric first bending, four second-bending frequencies and NASA's whole-aircraft 2.31 and 2.29 Hz.

In flight the mode is driven by the strips' forces and the inertia of the panel in the moving airframe. Each strip moves with it, the local dihedral changes, the airframe feels
the panels' acceleration. Thrust lies along body x, which bending doesn't turn, so it only shifts in height. First bending only: no torsion, second bending or fore-aft, so
this says nothing about flutter.

    python3 x57flex.py          prints the beam against the test
"""
import json, math, os, sys
import numpy as np
from scipy.linalg import eigh
from scipy.optimize import brentq, least_squares
import x57data as X

HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
FIT_FILE = os.path.join(HERE, 'data', 'x57_flex_fit.json')
IN, LB, G0, FT = 0.0254, 0.45359237, 9.80665, 0.3048
SLUG = 14.5939
Y0 = 24.0 * IN                      # the H-frame's outer points, read off the paper's fig. 5: about the fuselage's half width
TIP = GEO['cruise']['hub_in'][1] * IN
WING_LB, HFRAME_LB, TIP_LB, HL_LB = 313.5, 30.0, 134.0, 16.0
ZETA = 0.0057
TEST = dict(tip=dict(s1=2.26, a1=2.14, s2=15.76, a2=14.82), clean=dict(s1=6.55, a1=5.76, s2=26.36, a2=23.90), free=dict(mod3=2.31, mod4=2.29))
IC_FRAMES = 12                      # frames of static deflection after an initial condition
NODES = 60


def chord(y):
    w = GEO['wing']['panels'][0]
    return (w['c0'] + (w['c1'] - w['c0']) * (y / IN) / w['y1']) * IN


def wing_mass(y):
    """kg per metre at the stations y, the H-frame left inboard of them"""
    full = np.linspace(0.0, TIP, 400)
    k = (WING_LB - HFRAME_LB) * LB / 2 / np.trapezoid(chord(full) ** 2, full)
    return k * chord(y) ** 2


def beam(A, B, tip_kg=0.0, discs=(), N=NODES):
    """stiffness and mass matrices, deflection and slope at each node from the root out, and the nodes. discs: (y, kg) on the panel"""
    ye = np.linspace(Y0, TIP, N + 1)
    Le = np.diff(ye)
    ym = 0.5 * (ye[1:] + ye[:-1])
    EI, me = A * ((1 - (ym - Y0) / (TIP - Y0)) + B) ** 2 / (1 + B) ** 2, wing_mass(ym)
    K = np.zeros((2 * N + 2,) * 2)
    Mm = np.zeros_like(K)
    for e in range(N):
        L = Le[e]
        k = EI[e] / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L], [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        m = me[e] * L / 420 * np.array([[156, 22 * L, 54, -13 * L], [22 * L, 4 * L * L, 13 * L, -3 * L * L], [54, 13 * L, 156, -22 * L], [-13 * L, -3 * L * L, -22 * L, 4 * L * L]])
        K[2 * e:2 * e + 4, 2 * e:2 * e + 4] += k
        Mm[2 * e:2 * e + 4, 2 * e:2 * e + 4] += m
    Mm[-2, -2] += tip_kg
    for y, kg in discs:                      # on the nearest node: they're small and many nodes are near
        Mm[2 * int(np.argmin(np.abs(ye - y))), 2 * int(np.argmin(np.abs(ye - y)))] += kg
    return K, Mm, ye


def hz(w2):
    return np.sqrt(np.maximum(w2, 0)) / (2 * math.pi)


def sym(A, B, tip_kg=0.0, discs=()):
    """held at the root, symmetric: the two lowest, Hz"""
    K, Mm, _ = beam(A, B, tip_kg, discs)
    return hz(eigh(K[2:, 2:], Mm[2:, 2:], eigvals_only=True, subset_by_index=[0, 1]))


def anti(A, B, Kr, tip_kg=0.0, discs=()):
    """the root's deflection held, its slope on the spring Kr, N m/rad"""
    K, Mm, _ = beam(A, B, tip_kg, discs)
    K[1, 1] += Kr
    keep = list(range(1, len(K)))
    return hz(eigh(K[np.ix_(keep, keep)], Mm[np.ix_(keep, keep)], eigvals_only=True, subset_by_index=[0, 1]))


def rest(discs, tip_kg):
    """the airplane less its panels outboard of the root: mass and roll inertia"""
    y = np.linspace(Y0, TIP, 300)
    m = wing_mass(y)
    panels = 2 * (np.trapezoid(m, y) + tip_kg + sum(d[1] for d in discs))
    Ip = 2 * (np.trapezoid(m * y * y, y) + tip_kg * TIP ** 2 + sum(d[1] * d[0] ** 2 for d in discs))
    Ixx = X.MASS['Ixx_lbin2'] * LB * IN ** 2
    return X.MASS['weight_lb'] * LB - panels, Ixx - Ip


def free_sym(A, B, tip_kg, discs):
    """the airplane free, symmetric: half the rest on the root as a mass, the root's slope held"""
    K, Mm, _ = beam(A, B, tip_kg, discs)
    Mm[0, 0] += rest(discs, tip_kg)[0] / 2
    keep = [0] + list(range(2, len(K)))
    return hz(eigh(K[np.ix_(keep, keep)], Mm[np.ix_(keep, keep)], eigvals_only=True, subset_by_index=[0, 2]))[1]


def free_anti(A, B, Kr, tip_kg, discs):
    """free, antisymmetric: the fuselage rolls through theta, the root's deflection y0 theta, its slope on the spring from theta"""
    K, Mm, _ = beam(A, B, tip_kg, discs)
    n = len(K)
    # coordinates: theta, root slope, the rest of the beam
    T = np.zeros((n, n))
    T[0, 0] = Y0
    T[1:, 1:] = np.eye(n - 1)
    Kt, Mt = T.T @ K @ T, T.T @ Mm @ T
    Kt[0, 0] += Kr
    Kt[1, 1] += Kr
    Kt[0, 1] -= Kr
    Kt[1, 0] -= Kr
    Mt[0, 0] += rest(discs, tip_kg)[1] / 2
    return hz(eigh(Kt, Mt, eigvals_only=True, subset_by_index=[0, 2]))[1]


def hl_discs():
    return [(h[1] * IN, HL_LB * LB) for h in GEO['high_lift']['hubs_in']]


def fit():
    tip = TIP_LB * LB

    def res(x):
        A, B = np.exp(x)
        return [sym(A, B, tip)[0] / TEST['tip']['s1'] - 1, sym(A, B)[0] / TEST['clean']['s1'] - 1]
    s = least_squares(res, [math.log(7e5), math.log(0.12)], x_scale=1.0, xtol=1e-10, ftol=1e-10)
    A, B = (float(x) for x in np.exp(s.x))
    Kr = brentq(lambda k: anti(A, B, math.exp(k), tip)[0] - TEST['tip']['a1'], math.log(1e4), math.log(1e10))
    return dict(A=A, B=B, Kr=float(math.exp(Kr)), res=float(np.abs(s.fun).max()))


def coefficients():
    if os.path.exists(FIT_FILE):
        return json.load(open(FIT_FILE))
    d = fit()
    with open(FIT_FILE, 'w') as f:
        json.dump(d, f)
    return d


def modal(hl=False):
    """what the flight model needs, in feet, slugs and lbf: the mode's shape with the tip at 1, generalized mass, frequencies, the mass's first moments"""
    c = coefficients()
    A, B, Kr = c['A'], c['B'], c['Kr']
    tip = TIP_LB * LB
    discs = hl_discs() if hl else []
    K, Mm, ye = beam(A, B, tip, discs)
    w2, V = eigh(K[2:, 2:], Mm[2:, 2:], subset_by_index=[0, 0])
    u = np.r_[0.0, 0.0, V[:, 0]]
    u = u / u[-2]
    phi, slope = u[0::2], u[1::2]
    mg = float(u @ Mm @ u)
    ws = math.sqrt(w2[0])
    wa = 2 * math.pi * anti(A, B, Kr, tip, discs)[0]
    y = np.linspace(Y0, TIP, 400)
    m = wing_mass(y)
    ph = np.interp(y, ye, phi)
    xw = (GEO['wing']['origin_in'][0] * IN) + 0.40 * chord(y)
    x_tip = 150.0 * IN                              # the nacelle's middle
    pts = [(TIP, tip, x_tip)] + [(d[0], d[1], h[0] * IN + 6 * IN) for d, h in zip(discs, GEO['high_lift']['hubs_in'])]
    Pt = float(np.trapezoid(m * ph, y)) + sum(k * np.interp(q, ye, phi) for q, k, _ in pts)
    Py = float(np.trapezoid(m * ph * y, y)) + sum(k * np.interp(q, ye, phi) * q for q, k, _ in pts)
    Px = float(np.trapezoid(m * ph * xw, y)) + sum(k * np.interp(q, ye, phi) * x for q, k, x in pts)
    Ms = X.MASS['weight_lb'] / 32.174
    Ix = X.MASS['Ixx_lbin2'] / 32.174 / 144
    mgs, Pts, Pys = mg / SLUG, Pt / SLUG, Py / SLUG / FT
    return dict(m_sym=mgs - 2 * Pts ** 2 / Ms, m_anti=mgs - 2 * Pys ** 2 / Ix, Ix=Ix, ye_ft=(ye / FT).tolist(), phi=phi.tolist(), slope_ft=(slope * FT).tolist(), mg_slug=mg / SLUG, ws=ws, wa=wa, k=mg / SLUG * ws ** 2, kc=0.5 * mg / SLUG * (wa ** 2 - ws ** 2),
                c=2 * ZETA * ws * mg / SLUG, Pt_slug=Pt / SLUG, Py_slugft=Py / SLUG / FT, x_c_ft=Px / Pt / FT, z_c_ft=87.5 / 12, y_c_ft=Py / Pt / FT)


def phi_at(D, y_ft):
    return float(np.interp(abs(y_ft), D['ye_ft'], D['phi'], left=0.0))


def slope_at(D, y_ft):
    return float(np.interp(abs(y_ft), D['ye_ft'], D['slope_ft'], left=0.0))


def system(hl):
    """the JSBSim system: the two panels' states, what drives them, the contacts at the tips"""
    from xmlfun import FCS, S, M, D as Df, Q, MAX, MIN, IFTHEN, GT, LT, p, v
    Dm = modal(hl)
    f = FCS('wing bending')
    f.declare('flex/enable', 1)
    f.declare('flex/ic-frames', 0)
    f.declare('flex/fz-gear-last', 0)
    f.declare('flex/l-gear-last', 0)
    for s in 'LR':
        for k in ('q%s-ft', 'qd%s-fps', 'qdd%s-fps2'):
            f.declare('flex/' + k % s, 0)
    f.channel('wing bending')
    dt = p('simulation/dt')
    f.fn('flex/ic-count', IFTHEN(LT(dt, v(1e-9)), v(0), MIN(v(1000), S(p('flex/ic-frames'), v(1)))), out='flex/ic-frames')
    start = LT(p('flex/ic-frames'), v(IC_FRAMES))
    arm = Df(M(v(1 / 12.0), p('inertia/cg-x-in')), v(Dm['x_c_ft']))
    # The panels don't read JSBSim's own accelerations: tried, that gave the wrong frequencies at 120 Hz, from the frame's lag, and negative damping at every rate up to 960 Hz, cause not found.
    # So the coupling is solved here: the airframe's acceleration from everything but the reaction, which the panels then see through effective masses, one for the two moving together, one for opposite
    fl, fr = p('bu/flex/FL-lbs'), p('bu/flex/FR-lbs')
    W = X.MASS['weight_lb']
    # the gear reaches them as the mean of two frames. A standing airplane's tyres hold it by friction that changes side each frame until the wheel stops, and the panels, answering a frame late,
    # kicked it on before it could: a 60 Hz shake. The mean takes out that frequency and nothing else, for half a frame's lag
    gz = M(v(0.5), S(p('forces/fbz-gear-lbs'), p('flex/fz-gear-last')))
    gl = M(v(0.5), S(p('moments/l-gear-lbsft'), p('flex/l-gear-last')))
    nz = f.fn('flex/nz-ext', Q(Df(S(fl, fr), S(p('forces/fbz-aero-lbs'), gz)), v(W)), desc='load factor without the panels\' reaction')
    nz0 = MAX(v(-1), MIN(v(3), nz))
    pd = f.fn('flex/pdot-ext', Q(S(Df(p('moments/l-aero-lbsft'), M(v(Dm['y_c_ft']), Df(fr, fl))), gl), v(Dm['Ix'])), desc='roll acceleration likewise')
    pdq = S(pd, M(p('velocities/q-rad_sec'), p('velocities/r-rad_sec')))
    prq = Df(M(p('velocities/p-rad_sec'), p('velocities/r-rad_sec')), p('accelerations/qdot-rad_sec2'))
    drive = {}
    for s, o, Py in (('L', 'R', -Dm['Py_slugft']), ('R', 'L', Dm['Py_slugft'])):
        q, qd, qo = p('flex/q%s-ft' % s), p('flex/qd%s-fps' % s), p('flex/q%s-ft' % o)
        rot = S(M(v(Py), pdq), M(v(Dm['Pt_slug']), prq, arm))
        air = p('bu/flex/Q%s-lbs' % s)
        spring = S(M(v(Dm['k']), q), M(v(Dm['kc']), Df(q, qo)))
        drive[s] = f.fn('flex/F%s-lbs' % s, Df(S(air, M(v(-Dm['Pt_slug'] * 32.174), nz), rot), spring), desc='air, inertia and structure on the mode')
        f.fn('flex/static%s-ft' % s, MAX(v(-6), MIN(v(6), Q(S(air, M(v(-Dm['Pt_slug'] * 32.174), nz0), rot), v(Dm['k'])))))
    # damping goes with the strain, so each kind of motion keeps the same share of critical
    cs = 2 * ZETA * math.sqrt(Dm['k'] * Dm['m_sym'])
    ca = 2 * ZETA * math.sqrt((Dm['k'] + 2 * Dm['kc']) * Dm['m_anti'])
    vl, vr = p('flex/qdL-fps'), p('flex/qdR-fps')
    As = f.fn('flex/acc-sym', Q(M(v(0.5), Df(S(drive['R'], drive['L']), M(v(cs), S(vr, vl)))), v(Dm['m_sym'])))
    Aa = f.fn('flex/acc-anti', Q(M(v(0.5), Df(Df(drive['R'], drive['L']), M(v(ca), Df(vr, vl)))), v(Dm['m_anti'])))
    for s, acc in (('L', Df(As, Aa)), ('R', S(As, Aa))):
        q, qd = p('flex/q%s-ft' % s), p('flex/qd%s-fps' % s)
        a = f.fn('flex/acc%s' % s, M(p('flex/enable'), IFTHEN(start, v(0), acc)))
        new = f.fn('flex/qd%s-new' % s, M(p('flex/enable'), IFTHEN(start, v(0), MAX(v(-60), MIN(v(60), S(qd, M(dt, a)))))), desc='held in reason, so that nothing that drives the wing too hard can take the flight model with it')
        f.fn('flex/qdd%s-calc' % s, a, out='flex/qdd%s-fps2' % s)
        f.fn('flex/qd%s-calc' % s, new, out='flex/qd%s-fps' % s)
        f.fn('flex/q%s-calc' % s, M(p('flex/enable'), MAX(v(-6), MIN(v(6), IFTHEN(start, p('flex/static%s-ft' % s), S(q, M(dt, new)))))), out='flex/q%s-ft' % s)
    # the contacts out at the tips ride with the wing (structural inches, z up)
    for unit, z, s in ((3, 86.0, 'L'), (4, 86.0, 'R'), (8, 58.0, 'L'), (9, 58.0, 'R')):
        f.fn('flex/contact%d-z' % unit, S(v(z), M(v(12.0), p('flex/q%s-ft' % s))), out='gear/unit[%d]/z-position' % unit)
    f.fn('flex/fz-gear-keep', p('forces/fbz-gear-lbs'), out='flex/fz-gear-last')
    f.fn('flex/l-gear-keep', p('moments/l-gear-lbsft'), out='flex/l-gear-last')
    return f


def main():
    print('fitting' if not os.path.exists(FIT_FILE) else 'stored fit')
    c = coefficients()
    A, B, Kr = c['A'], c['B'], c['Kr']
    tip = TIP_LB * LB
    print('EI at the root %.3g N m2, at the tip %.3g, root spring %.3g N m/rad, residual %.1e' % (A, A * (B / (1 + B)) ** 2, Kr, c['res']))
    s, a = sym(A, B, tip), anti(A, B, Kr, tip)
    print('tip masses: symmetric %.2f, %.2f Hz (test 2.26, 15.76); antisymmetric %.2f, %.2f (2.14, 14.82)' % (s[0], s[1], a[0], a[1]))
    s, a = sym(A, B), anti(A, B, Kr)
    print('clean:      symmetric %.2f, %.2f Hz (test 6.55, 26.36); antisymmetric %.2f, %.2f (5.76, 23.90)' % (s[0], s[1], a[0], a[1]))
    print('free in the air, Mod III: symmetric %.2f, antisymmetric %.2f Hz; Mod IV: %.2f, %.2f (NASA 2.31, 2.29, kind not stated)' % (free_sym(A, B, tip, []), free_anti(A, B, Kr, tip, []), free_sym(A, B, tip, hl_discs()), free_anti(A, B, Kr, tip, hl_discs())))
    for hl in (False, True):
        D = modal(hl)
        print('Mod %s mode: held %.2f, %.2f Hz, generalized mass %.2f slug, first moment %.2f slug, centre at y %.1f ft; sag on the ground %.2f ft' % ('IV' if hl else 'III', D['ws'] / 2 / math.pi, D['wa'] / 2 / math.pi, D['mg_slug'], D['Pt_slug'], D['y_c_ft'], D['Pt_slug'] * 32.174 / D['k']))


if __name__ == '__main__':
    main()
