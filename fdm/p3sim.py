"""The P2006T in JSBSim: load a build, set it up, trim it, and read its coefficients and modes. As harness.py is for the X-57.

    f = load(kg=1140, cg=0.19)
    t = trim(f, 110, hp=1000, power=None)
    d = derivatives(f, t)
    m = modes(f, t)
"""
import math, os
import jsbsim
import numpy as np
from scipy.optimize import least_squares
import genp2006t as G
import harness as H
import p2006t as P

ROOT = os.path.join(H.HERE, 'root-p2006t' + os.environ.get('X57_ROOT_TAG', ''))
S, B, C = P.REF['S_ft2'], P.REF['b_ft'], P.REF['c_ft']
LAST = {}


def load(kg=1180.0, cg=0.25, tunnel=False, tail=True, nacelles=True, dt=1 / H.HZ, edit=None):
    LAST['cfg'] = G.write(os.path.join(ROOT, 'aircraft', 'p2006t'), tunnel, tail, nacelles, kg, cg, edit)
    f = jsbsim.FGFDMExec(ROOT)
    f.set_debug_level(0)
    f.set_dt(dt)
    f.load_model('p2006t')
    f['ctl/instant'] = 1
    f['bu/test/frozen'] = 1
    return f


def lever_of(kw, hp):
    """the lever for a shaft power at a height"""
    e = P.ENGINE
    av = e['takeoff_kw'] * np.interp(hp, P.POWER_LAPSE['alt_ft'], P.POWER_LAPSE['frac'])
    return (kw - e['idle_kw']) / (av - e['idle_kw'])


def configure(f, flap=0.0, gear=0.0, stab=-5.0, lever=(0.0, 0.0), ail=0.0, rud=0.0, run=(1, 1)):
    f['ctl/flap-cmd-deg'] = flap
    f['gear/gear-cmd-norm'] = gear
    f['fcs/elevator-cmd-norm'] = (stab + 5.0) / 10.0
    f['fcs/aileron-cmd-norm'] = ail
    f['fcs/rudder-cmd-norm'] = rud
    for i, n in enumerate('lr'):
        f['/controls/engines/engine[%d]/throttle' % i] = lever[i]
        if f.get_property_manager().hasNode('prop/%s/run' % n):
            f['prop/%s/run' % n] = run[i]


def coef(f):
    """coefficients about the centre of gravity: stability axes for lift and drag, body axes for the rest"""
    q = f['aero/qbar-psf']
    a = f['aero/alpha-rad']
    X, Y, Z = f['forces/fbx-aero-lbs'], f['forces/fby-aero-lbs'], f['forces/fbz-aero-lbs']
    return dict(CL=(X * math.sin(a) - Z * math.cos(a)) / (q * S), CD=-(X * math.cos(a) + Z * math.sin(a)) / (q * S), CY=Y / (q * S),
                Cl=f['moments/l-aero-lbsft'] / (q * S * B), Cm=f['moments/m-aero-lbsft'] / (q * S * C), Cn=f['moments/n-aero-lbsft'] / (q * S * B))


def trim(f, kcas, flap=0.0, gear=0.0, hp=2500.0, gamma=0.0, kw=None, run=(1, 1), x0=None):
    """Level flight or a climb. With a power kw on both engines the flight path is solved; otherwise the lever. A list gives each engine's"""
    V = H.tas_of_cas(f, kcas, hp)
    g = math.radians(gamma)
    free = kw is None

    def res(x):
        a, st = x[0], x[1]
        lv = (x[2], x[2]) if free else tuple(lever_of(kw, hp) for _ in 'lr')
        gam = g if free else math.radians(x[2])
        configure(f, flap, gear, st, lv, run=run)
        ar = math.radians(a)
        acc = H.state(f, V * math.cos(ar), 0.0, V * math.sin(ar), theta=ar + gam, hp=hp)
        return [acc[0] * math.cos(ar) + acc[2] * math.sin(ar), -acc[0] * math.sin(ar) + acc[2] * math.cos(ar), acc[4] * 10]

    s = least_squares(res, x0 or [3.0, -6.0, 0.4 if free else 0.0], x_scale=[2, 2, 0.2 if free else 2], xtol=1e-7, ftol=1e-9, gtol=1e-9)
    res(s.x)
    t = dict(kcas=kcas, V=V, alpha=s.x[0], stab=s.x[1], lever=s.x[2] if free else lever_of(kw, hp), gamma=gamma if free else s.x[2], flap=flap, gear=gear, hp=hp, run=run,
             res=float(np.abs(s.fun).max()), ktas=V / 1.6878099, kw=f['prop/l/power-kw'], thrust=f['prop/l/thrust-lbs'] + f['prop/r/thrust-lbs'], eta=f['prop/r/eta'], **coef(f))
    t['roc'] = V * math.sin(math.radians(t['gamma'])) * 60
    return t


def at(f, t, alpha=None, beta=0.0, p=0.0, q=0.0, r=0.0, stab=None, ail=0.0, rud=0.0, adot=0.0):
    """the coefficients at a trim, with something changed, nothing integrated"""
    configure(f, t['flap'], t['gear'], t['stab'] if stab is None else stab, (t['lever'], t['lever']), ail, rud, t['run'])
    if f.get_property_manager().hasNode('bu/test/alphadot'):
        f['bu/test/alphadot'] = adot
    a = math.radians(t['alpha'] if alpha is None else alpha)
    b = math.radians(beta)
    V = t['V']
    H.state(f, V * math.cos(a) * math.cos(b), V * math.sin(b), V * math.sin(a) * math.cos(b), p, q, r, 0.0, math.radians(t['alpha'] + t['gamma']), hp=t['hp'])
    return coef(f)


def derivatives(f, t):
    """per radian, about the centre of gravity, by central differences. Rates non-dimensional as the paper's: q c / 2V, p b / 2V"""
    V = t['V']
    d = {}
    da = 1.0

    def cd(key, a, b, h):
        return (b[key] - a[key]) / h
    A0, A1 = at(f, t, alpha=t['alpha'] - da), at(f, t, alpha=t['alpha'] + da)
    h = 2 * math.radians(da)
    d.update(CLa=cd('CL', A0, A1, h), Cma=cd('Cm', A0, A1, h), CDa=cd('CD', A0, A1, h))
    B0, B1 = at(f, t, beta=-2.0), at(f, t, beta=2.0)
    h = 2 * math.radians(2.0)
    d.update(CYb=cd('CY', B0, B1, h), Clb=cd('Cl', B0, B1, h), Cnb=cd('Cn', B0, B1, h))
    e = 0.05
    Q0, Q1 = at(f, t, q=-e), at(f, t, q=e)
    h = 2 * e * C / (2 * V)
    d.update(Cmq=cd('Cm', Q0, Q1, h), CLq=cd('CL', Q0, Q1, h))
    P0, P1 = at(f, t, p=-e), at(f, t, p=e)
    h = 2 * e * B / (2 * V)
    d.update(Clp=cd('Cl', P0, P1, h), Cnp=cd('Cn', P0, P1, h), CYp=cd('CY', P0, P1, h))
    R0, R1 = at(f, t, r=-e), at(f, t, r=e)
    d.update(Clr=cd('Cl', R0, R1, h), Cnr=cd('Cn', R0, R1, h), CYr=cd('CY', R0, R1, h))
    S0, S1 = at(f, t, stab=t['stab'] - 1), at(f, t, stab=t['stab'] + 1)
    h = 2 * math.radians(1.0)
    d.update(Cmde=cd('Cm', S0, S1, h), CLde=cd('CL', S0, S1, h))
    # surfaces: as the paper's signs. Aileron: per radian of the mean of the two, right stick. Rudder: trailing edge left
    up, dn = -P.SURFACES['aileron'][0], P.SURFACES['aileron'][1]
    a0, a1 = at(f, t, ail=-0.25), at(f, t, ail=0.25)
    h = math.radians(0.25 * (up + dn))                     # both ways, half the sum of the two surfaces each
    d.update(Clda=cd('Cl', a0, a1, h), Cnda=cd('Cn', a0, a1, h), CYda=cd('CY', a0, a1, h))
    r0, r1 = at(f, t, rud=-0.3), at(f, t, rud=0.3)
    h = 2 * math.radians(0.3 * P.SURFACES['rudder'])
    d.update(Cldr=cd('Cl', r0, r1, h), Cndr=cd('Cn', r0, r1, h), CYdr=cd('CY', r0, r1, h))
    D0, D1 = at(f, t, adot=-0.1), at(f, t, adot=0.1)
    d.update(Cmad=cd('Cm', D0, D1, 2 * 0.1 * C / (2 * V)))
    at(f, t)
    return d


def modes(f, t):
    """eigenvalues about a trim. States u, v, w, p, q, r, phi, theta. The downwash's lag makes alpha-dot part of the forces, so its column goes on the left side"""
    configure(f, t['flap'], t['gear'], t['stab'], (t['lever'], t['lever']), run=t['run'])
    f['bu/test/alphadot'] = 0
    a = math.radians(t['alpha'])
    th = a + math.radians(t['gamma'])
    x0 = np.array([t['V'] * math.cos(a), 0.0, t['V'] * math.sin(a), 0, 0, 0, 0, th])

    def dx(x):
        acc = H.state(f, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], hp=t['hp'])
        ph, the = x[6], x[7]
        return np.r_[acc, x[3] + (x[4] * math.sin(ph) + x[5] * math.cos(ph)) * math.tan(the), x[4] * math.cos(ph) - x[5] * math.sin(ph)]

    h = np.array([1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    A = np.zeros((8, 8))
    for i in range(8):
        e = np.zeros(8)
        e[i] = h[i]
        A[:, i] = (dx(x0 + e) - dx(x0 - e)) / (2 * h[i])
    # the part of the accelerations that goes with alpha-dot = w-dot / V
    f['bu/test/alphadot'] = 0.1
    up = dx(x0)
    f['bu/test/alphadot'] = -0.1
    dn = dx(x0)
    f['bu/test/alphadot'] = 0
    col = (up - dn) / 0.2 / t['V']                          # per unit w-dot
    E = np.eye(8)
    E[:, 2] -= col
    ev = np.linalg.eigvals(np.linalg.solve(E, A))
    osc = sorted([e for e in ev if e.imag > 1e-4], key=abs)
    real = sorted([e.real for e in ev if abs(e.imag) <= 1e-4])
    out = dict(eig=ev, A=A, real=real)
    if len(osc) >= 3:
        out.update(phugoid=osc[0], dutch_roll=osc[1], short_period=osc[2])
    return out


def zeta(e):
    return -e.real / abs(e)
