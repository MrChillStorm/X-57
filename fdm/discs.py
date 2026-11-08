#!/usr/bin/env python3
"""The propellers' discs: blade-element momentum theory on NASA's blades, anchored to NASA's numbers, into
data/disc_tables.json.

    python3 discs.py

Each disc has tables of thrust, power and in-plane force coefficient. Where they come from, best first:
  - High lift, Ct and Cq on J: the 2022 tunnel test of the real propellers, over the J it covers. Measured torque runs a
    little above NASA's blade-element model, and thrust below it at high J.
  - High lift elsewhere: NASA's tables [ANLYS-CEPT-023], which go from near static to windmilling, joined to the tunnel's curve.
  - Cruise, Ct and Cp on J and pitch: NASA's blade-element points given to the CFD, along the operating band.
  - Everything else: blade-element theory here, on the OpenVSP blades with NeuralFoil's sections: the cruise propeller off the
    band, out to feather, and the in-plane force of both at incidence, which the CFD's actuator discs don't give. Its error
    against NASA's points is carried onto the grid, so the table passes through them and keeps the theory's shape beyond.

The in-plane force is the quasi-steady force of a disc whose blades see the crossflow: the advancing blades' extra torque
force lies along the crossflow, so a propeller ahead of the centre of gravity is destabilising. Hub moments are left out:
the discs turn in opposite pairs, so they cancel while all are running. No blade elasticity: NASA's tunnel test found the
high-lift blades detwist under load, which is in the measured curves but not in the theory.
"""
import json, math, os, sys, warnings
import numpy as np
from scipy.interpolate import RBFInterpolator
from scipy.optimize import brentq
import x57data as X_

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
HL_EFFECTIVE_PITCH = 20.0    # deg at 0.75 R: the blades fold 10 deg forward spinning, and the rest fits NASA's power [PyWopWop's fit]
ALPHAS = np.r_[np.arange(-30, -14, 3.0), np.arange(-14, 18, 1.0), np.arange(18, 31, 3.0)]


def polar(path, re):
    """cl and cd over +-180 deg: NeuralFoil near zero lift, a flat plate far from it"""
    import neuralfoil as nf
    xy = np.loadtxt(path, skiprows=1)
    xy[:, 0] -= xy[:, 0].min()
    xy = xy / xy[:, 0].max()
    r = nf.get_aero_from_coordinates(coordinates=xy, alpha=ALPHAS, Re=re, model_size='large')
    cl, cd = np.array(r['CL']), np.array(r['CD'])
    a = np.r_[np.arange(-180, -30, 10.0), ALPHAS, np.arange(40, 181, 10.0)]
    fl = 1.1 * np.sin(2 * np.radians(a))
    fd = 0.03 + 1.8 * np.sin(np.radians(a)) ** 2
    w = np.clip((np.abs(a) - 16) / 12, 0, 1)
    CL = (1 - w) * np.interp(a, ALPHAS, cl) + w * fl
    CD = np.maximum((1 - w) * np.interp(a, ALPHAS, cd), 0.006) + w * fd
    return a, CL, CD


class Blade:
    """stations, and the section polars each uses"""

    def __init__(s, table, radius_ft, blades, sections, nst=18):
        r0, r1 = table['r_R'][0] + 0.02, 0.985
        s.x = r0 + (r1 - r0) * (0.5 - 0.5 * np.cos(np.linspace(0, math.pi, nst)))
        s.R, s.B = radius_ft, blades
        s.c = np.interp(s.x, table['r_R'], table['chord_R']) * radius_ft
        s.beta = np.radians(np.interp(s.x, table['r_R'], table['beta_deg']))
        s.beta34 = float(np.interp(0.75, s.x, s.beta))
        s.dr = np.gradient(s.x) * radius_ft
        s.sec = sections(s.x)


def solve(bl, J, pitch_deg, n=30.0, rho=0.0023769, nu=1.57e-4):
    """Thrust and power coefficients, and the in-plane force coefficient per radian of incidence, by station"""
    D = 2 * bl.R
    V = max(J, 0.05) * n * D
    om = 2 * math.pi * n
    dth = math.radians(pitch_deg) - bl.beta34
    T = Q = N = 0.0
    inc = math.radians(4.0)
    psi = np.arange(8) * math.pi / 4
    for i, x in enumerate(bl.x):
        r = x * bl.R
        c = bl.c[i]
        th = bl.beta[i] + dth
        a_t, cl_t, cd_t = bl.sec[i]
        sig = bl.B * c / (2 * math.pi * r)

        def coef(phi):
            al = math.degrees(th - phi)
            return float(np.interp(al, a_t, cl_t)), float(np.interp(al, a_t, cd_t))

        def tip(phi):
            f = bl.B / 2 * (1 - x) / max(x * abs(math.sin(phi)), 1e-3)
            return max(2 / math.pi * math.acos(min(math.exp(-f), 1.0)), 1e-4)

        def fac(phi):
            cl, cd = coef(phi)
            cn = cl * math.cos(phi) - cd * math.sin(phi)
            ct = cl * math.sin(phi) + cd * math.cos(phi)
            F = tip(phi)
            sp = math.sin(phi) if abs(math.sin(phi)) > 1e-3 else 1e-3
            k = sig * cn / (4 * F * sp ** 2)
            kp = sig * ct / (4 * F * sp * math.cos(phi))
            return k, kp, cn, ct

        def induced(phi):
            """a propeller's axial and swirl factors: the flow through the disc is V (1 + a), the blade's speed om r (1 - a')"""
            k, kp, cn, ct = fac(phi)
            k = min(k, 0.9)                 # momentum theory runs out towards the static case
            return k / (1 - k), kp / (1 + kp) if abs(1 + kp) > 1e-3 else 0.0, cn, ct

        def res(phi):
            a, ap, _, _ = induced(phi)
            return math.sin(phi) * om * r * (1 - ap) - math.cos(phi) * V * (1 + a)

        # a lightly loaded section can have several roots, some false. The real one is the nearest to the undisturbed inflow, and a
        # sign change at a jump in the residual isn't a root at all
        phi0 = math.atan2(V, om * r)
        grid = np.linspace(-0.6, math.pi / 2 - 1e-3, 80)
        grid = grid[np.abs(grid) > 1e-3]
        vals = [res(p) for p in grid]
        roots = []
        for p0, p1, v0, v1 in zip(grid, grid[1:], vals, vals[1:]):
            if v0 * v1 < 0 and p0 * p1 > 0:
                g = brentq(res, p0, p1, xtol=1e-5)
                if abs(res(g)) < 0.02 * math.hypot(V, om * r):
                    roots.append(g)
        phi = min(roots, key=lambda g: abs(g - phi0)) if roots else phi0
        a, ap, cn, ct = induced(phi)
        w2 = (V * (1 + a)) ** 2 + (om * r * (1 - ap)) ** 2
        q = 0.5 * rho * w2 * c * bl.dr[i] * bl.B
        T += q * cn
        Q += q * ct * r
        # at incidence: each azimuth's tangential speed changes by the crossflow; the same induced flow
        U = V * math.sin(inc)
        vt = om * r * (1 - ap) + U * np.sin(psi)
        va = V * math.cos(inc) * (1 + a)
        ph = np.arctan2(va, vt)
        al = np.degrees(th - ph)
        cl = np.interp(al, a_t, cl_t)
        cd = np.interp(al, a_t, cd_t)
        ctan = cl * np.sin(ph) + cd * np.cos(ph)
        ft = 0.5 * rho * (va ** 2 + vt ** 2) * c * bl.dr[i] * ctan        # one blade's torque force at each azimuth
        N += bl.B * float(np.mean(ft * np.sin(psi)))
    k = rho * n * n * D ** 4
    return T / k, 2 * math.pi * Q / (k * D), N / k / inc


def hl_blade():
    a = polar(os.path.join(HERE, 'data', 'sections', 'hl_blade.dat'), 2.5e5)
    h = GEO['high_lift']
    return Blade(h['blade'], h['diameter_in'] / 24.0, h['blades'], lambda x: [a] * len(x))


def cruise_blade():
    S = os.path.join(HERE, 'data', 'sections')
    c = GEO['cruise']
    # the slices run root to tip, thick to thin; the thick ones matter little
    picks = [(0.45, 5), (0.65, 7), (0.85, 9), (1.0, 10)]
    pol = {k: polar(os.path.join(S, 'cruise_blade_%02d.dat' % k), 1.5e6) for _, k in picks}
    return Blade(c['blade'], c['diameter_in'] / 24.0, c['blades'], lambda x: [pol[next(k for lim, k in picks if u <= lim)] for u in x])


def nasa_cruise():
    rows = json.load(open(os.path.join(HERE, 'data', 'cfd', 'cruise.json')))
    p = {}
    for r in rows:
        q = r['prop']
        if q and q['Ct'] is not None and r['alpha'] == 2 and not r['beta'] and not r['ail'] and not r['flap']:
            p[round(q['J'], 2), q['pitch']] = (q['Ct'], q['Cp'])
    return p


def cruise(bl):
    # fine enough that a trim's slopes are the propeller's, not the table's: the phugoid hangs on how thrust moves with speed
    Js = [0.0, 0.1, 0.25] + [round(0.4 + 0.05 * i, 2) for i in range(25)] + [1.7, 1.8, 2.0, 2.4, 3.0, 4.0]
    Bs = [8.0, 11.0] + [14.0 + 2.0 * i for i in range(17)] + [50.0, 55.0, 65.0, 75.0, 85.0]
    ct = np.zeros((len(Js), len(Bs)))
    cp = ct.copy()
    cn = ct.copy()
    for i, J in enumerate(Js):
        for j, b in enumerate(Bs):
            t, p, n = solve(bl, J, b, n=37.5)
            ct[i, j], cp[i, j], cn[i, j] = t, p, n
    P = nasa_cruise()
    X = np.array(list(P))
    bem = np.array([solve(bl, k[0], k[1], n=37.5)[:2] for k in P])
    nas = np.array(list(P.values()))
    e = nas - bem
    print('cruise, theory against NASA\'s %d points: Ct rms %.4f (bias %+.4f), Cp rms %.4f (bias %+.4f)' % (
        len(P), np.sqrt((e[:, 0] ** 2).mean()), e[:, 0].mean(), np.sqrt((e[:, 1] ** 2).mean()), e[:, 1].mean()))
    # carry the error onto the grid: thin plate inside the band, fading to nil away from NASA's points
    sc = np.array([1.0, 30.0])
    G = np.array([(j, b) for j in Js for b in Bs])
    f = RBFInterpolator(X / sc, e, kernel='thin_plate_spline', smoothing=1e-3)(G / sc)
    d = np.min(np.linalg.norm((G[:, None, :] - X[None, :, :]) / sc, axis=2), axis=1)
    w = np.clip(1.5 - d / 0.4, 0, 1)[:, None]
    f = (f * w).reshape(len(Js), len(Bs), 2)
    from scipy.ndimage import gaussian_filter
    Ct, Cp = ct + f[..., 0], cp + f[..., 1]
    # blade pitch from J and power, for a trim: the pitch at which the power is met, between the fine stop and 46 deg. It reads
    # the power above what the stop takes, so that it meets the stop exactly, wherever that is
    Cps = [0.0, 0.0025, 0.005, 0.0075, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.065, 0.08, 0.095, 0.11, 0.125, 0.14, 0.155, 0.17, 0.185, 0.2,
           0.22, 0.24, 0.26, 0.28, 0.32, 0.4, 0.5]
    fine = X_.HUB['fine_deg']
    Cp_fine = Cp[:, Bs.index(fine)]
    bf = np.linspace(fine, 46.0, 641)
    pitch_of = np.zeros((len(Js), len(Cps)))
    for i in range(len(Js)):
        c = np.maximum.accumulate(np.interp(bf, Bs, Cp[i]))
        pitch_of[i] = np.interp(Cp_fine[i] + np.array(Cps), c + 1e-9 * np.arange(len(c)), bf)
    # on the fine stop the hub has nothing left and the speed follows the torque. Q / (rho u^2 D^3) = Cq / J^2 gives J at once for a
    # trim, from static down to the most the windmilling blades will take: past that they stall, and the curve turns back
    j = np.linspace(0.2, 1.2, 101)
    q = np.interp(j, Js, Cp_fine) / (2 * np.pi * j ** 2)
    j, q = j[:np.argmin(q) + 1][::-1], q[:np.argmin(q) + 1][::-1]
    return dict(J=Js, pitch=Bs, Ct=Ct.tolist(), Cp=Cp.tolist(), Cn=np.maximum(gaussian_filter(cn, 1.0), 0).tolist(), Cp_axis=Cps,
                pitch_of=pitch_of.tolist(), Cp_fine=Cp_fine.tolist(), sag_x=q.tolist(), sag_J=j.tolist(), theory_Ct=ct.tolist(), theory_Cp=cp.tolist())


def high_lift(bl):
    H = json.load(open(os.path.join(HERE, 'data', 'hlp_tables.json')))
    T = json.load(open(os.path.join(HERE, 'data', 'digitised', 'hlp_tunnel.json')))
    P = sorted(H['airspeed'] + H['fixed'], key=lambda p: p['J'])
    J = np.array([p['J'] for p in P])
    Js = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 2.0, 2.5, 3.0])
    out = dict(J=Js.tolist())
    for c in ('Ct', 'Cq'):
        y = np.array([p[c] for p in P])
        a = np.polyfit(J, y, 6)                                     # NASA's curve
        nasa = np.polyval(a, np.clip(Js, 0.06, 1.75))
        hi = Js > 1.75
        s = np.polyval(np.polyder(a), 1.75)
        nasa[hi] = np.polyval(a, 1.75) + s * (Js[hi] - 1.75)
        meas = np.array(T[c]['LH'] + T[c]['RH'])
        m = np.polyfit(meas[:, 0], meas[:, 1], 3)                   # the tunnel's, both hands
        tun = np.polyval(m, np.clip(Js, 0.62, 1.33))
        # the tunnel's difference from NASA fades away outside the J it covers
        fade = np.clip((Js - 0.45) / 0.17, 0, 1) * np.clip((1.6 - Js) / 0.27, 0, 1)
        out[c] = (nasa + fade * (tun - nasa)).tolist()
        out['nasa_' + c] = nasa.tolist()
        print('high lift %s: J 0.65 %.4f (NASA %.4f), 1.0 %.4f (%.4f), 1.25 %.4f (%.4f); %d tunnel points' % (
            c, np.interp(.65, Js, out[c]), np.interp(.65, Js, nasa), np.interp(1.0, Js, out[c]), np.interp(1.0, Js, nasa),
            np.interp(1.25, Js, out[c]), np.interp(1.25, Js, nasa), len(meas)))
    b = [solve(bl, j, HL_EFFECTIVE_PITCH, n=75.0) for j in Js]
    out['theory_Ct'] = [x[0] for x in b]
    out['theory_Cq'] = [x[1] / (2 * math.pi) for x in b]
    # (a smooth curve: the theory's stall and tip loss make it ragged from point to point)
    a = np.polyfit(Js[Js <= 2.0], [x[2] for x, j in zip(b, Js) if j <= 2.0], 3)
    out['Cn'] = np.polyval(a, np.clip(Js, 0, 2.0)).clip(0).tolist()
    return out


if __name__ == '__main__':
    hl = high_lift(hl_blade())
    cr = cruise(cruise_blade())
    with open(os.path.join(HERE, 'data', 'disc_tables.json'), 'w') as f:
        json.dump(dict(high_lift=hl, cruise=cr), f, separators=(',', ':'))
    J = hl['J']
    print('   J      Ct    theory     Cq     theory    Cn/rad')
    for j in (0.0, 0.3, 0.65, 1.0, 1.25, 1.5, 1.8):
        i = J.index(j) if j in J else int(np.argmin(np.abs(np.array(J) - j)))
        print('  %.2f  %.4f  %.4f   %.5f  %.5f   %.4f' % (J[i], hl['Ct'][i], hl['theory_Ct'][i], hl['Cq'][i], hl['theory_Cq'][i], hl['Cn'][i]))
    print('cruise at J 1.3, pitch 30: Ct %.4f Cp %.4f Cn %.4f; feathered 85 at J 1: Ct %.4f Cp %.4f' % (
        cr['Ct'][8][5], cr['Cp'][8][5], cr['Cn'][8][5], cr['Ct'][6][13], cr['Cp'][6][13]))
