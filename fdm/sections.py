"""Section tables for the built-up model: cl, cd and cm about the quarter chord over +-180 deg.

    table(xy, Re)                         one section, plain
    table(xy, Re, hinge=0.75, deltas=...) with a plain flap, deflected in the geometry
    slotted(t, flaps)                     a slotted flap on a table: its zero-lift shift and maximum lift, given

Attached flow is NeuralFoil's. Past stall it joins a flat plate: normal force CN90 sin(alpha), its centre moving from the quarter chord to
mid chord at 90 deg. This is the ASK 21's method. NeuralFoil's own stall is too gentle and its maximum lift too high for a real wing,
so clmax_k sets the peak, with the drop after it (ASK 21: 0.85 of NeuralFoil's clmax, 0.22 drop).

n_crit and the transition points carry the surface: 9 and free for a smooth wing, xtr 0.05 for the tunnel's trip strips.
"""
import hashlib, json, os, warnings
import numpy as np

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'data', 'sections', 'cache')
ALPHA = np.unique(np.r_[np.arange(-180, -40, 10.0), np.arange(-40, -24, 3.0), np.arange(-24, 30, 1.0), np.arange(30, 45, 3.0), np.arange(45, 181, 10.0)])
AN = np.arange(-24.0, 30.01, 0.5)


def smooth(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def naca6a(thickness, cl_design, name_t='naca632a015', name_c='naca63a210'):
    """A NACA 63A section from NASA's listed ones: the 15 % thickness form scaled, the a = 0.8 (modified) mean line of the 63A210 scaled from its
    design lift of 0.2. Selig order"""
    import aerosandbox as asb
    x = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, 100))
    t = asb.Airfoil(name_t)
    c = asb.Airfoil(name_c)
    half = 0.5 * np.array(t.local_thickness(x)) * thickness / float(t.max_thickness())
    camb = np.array(c.local_camber(x)) * cl_design / 0.2
    up = np.c_[x, camb + half]
    lo = np.c_[x, camb - half]
    return np.r_[up[::-1], lo[1:]]


def four_digit(t):
    import aerosandbox as asb
    return np.array(asb.Airfoil('naca00%02d' % round(100 * t)).coordinates)


def _nf(xy, Re, n_crit, xtr, hinge, d):
    import aerosandbox as asb
    import neuralfoil as nf
    a = asb.Airfoil(name='s', coordinates=xy)
    if hinge is not None and d:
        a = a.add_control_surface(deflection=d, hinge_point_x=hinge)
    r = nf.get_aero_from_coordinates(coordinates=np.array(a.coordinates), alpha=AN, Re=Re, n_crit=n_crit, xtr_upper=xtr, xtr_lower=xtr, model_size='xxlarge')
    return np.array(r['CL']), np.array(r['CD']), np.array(r['CM'])


def stall_shape(cl, clmax_k, drop=0.22, width=5.0, knee=0.8):
    """the peak set to clmax_k of NeuralFoil's, reached with a smooth flattening above knee of it, then a lift drop"""
    ip = np.where((AN >= 0) & (AN <= 28))[0]
    top = clmax_k * cl[ip].max()
    k = next(i for i in ip if cl[i] >= knee * top)
    slope = (cl[k] - cl[k - 2]) / (AN[k] - AN[k - 2])
    peak = AN[k] + 2.0 * (top - cl[k]) / slope
    n = max(slope * (peak - AN[k]) / (top - cl[k]), 1.5)
    out = cl.copy()
    for i in range(k, len(AN)):
        if AN[i] <= peak:
            out[i] = top - (top - cl[k]) * (1 - (AN[i] - AN[k]) / (peak - AN[k])) ** n
        else:
            out[i] = top - drop * (1 - np.exp(-(AN[i] - peak) / width))
    return out


def extend(cl, cd, cm, CN90=1.3, off=8.0, width=15.0, xcp90=0.5, shift=0.0):
    """the attached curves carried past stall into a flat plate; shift, deg of plate angle, is a deflected flap's"""
    ip = np.where((AN >= 4) & (AN <= 28))[0]
    im = np.where((AN <= -4) & (AN >= -24))[0]
    ap, am = AN[ip[np.argmax(cl[ip])]], AN[im[np.argmin(cl[im])]]
    L, Dg, Mo = (np.zeros(len(ALPHA)) for _ in range(3))
    for i, a in enumerate(ALPHA):
        r = np.radians(a)
        cn = CN90 * np.sin(np.radians(a + shift * np.cos(r) ** 2))
        aa = abs(a)
        x = 0.25 + (xcp90 - 0.25) * aa / 90 if aa <= 90 else xcp90 + (0.75 - xcp90) * (aa - 90) / 90
        pl, pd, pm = cn * np.cos(r), 0.012 + cn * np.sin(r), -cn * (x - 0.25)
        c = np.clip(a, AN[0], AN[-1])
        l1, d1, m1 = np.interp(c, AN, cl), np.interp(c, AN, cd), np.interp(c, AN, cm)
        w = smooth((a - ap - off) / width) if a > ap else smooth((am - off - a) / width) if a < am else 0.0
        L[i], Dg[i], Mo[i] = (1 - w) * l1 + w * pl, (1 - w) * d1 + w * max(pd, d1), (1 - w) * m1 + w * pm
    return L, Dg, Mo


def table(xy, Re, hinge=None, deltas=(0.0,), n_crit=9.0, xtr=1.0, clmax_k=0.9, flap_eff=0.9, dref=12.0, CN90=1.3, drop=0.22, sym=False):
    """the section's tables on ALPHA and deltas (trailing edge down positive). NeuralFoil loses a flap's lift past about 12 deg, so
    beyond that its increment is scaled from dref's by NeuralFoil's own at zero incidence"""
    key = hashlib.sha1(json.dumps([np.round(xy, 5).tolist(), Re, hinge, list(deltas), n_crit, xtr, clmax_k, flap_eff, dref, CN90, drop, sym]).encode()).hexdigest()[:16]
    path = os.path.join(CACHE, key + '.json')
    if os.path.exists(path):
        return json.load(open(path))
    base = _nf(xy, Re, n_crit, xtr, None, 0)
    i0 = int(np.argmin(np.abs(AN)))
    CL, CD, CM = (np.zeros((len(ALPHA), len(deltas))) for _ in range(3))
    for j, d in enumerate(deltas):
        if abs(d) <= dref:
            cl, cd, cm = _nf(xy, Re, n_crit, xtr, hinge, d)
        else:
            r, big = _nf(xy, Re, n_crit, xtr, hinge, np.sign(d) * dref), _nf(xy, Re, n_crit, xtr, hinge, d)
            k = (big[0][i0] - base[0][i0]) / (r[0][i0] - base[0][i0])
            cl, cm = base[0] + k * (r[0] - base[0]), base[2] + k * (r[2] - base[2])
            cd = base[1] + k * k * (r[1] - base[1])
        if d:
            cl, cm = base[0] + flap_eff * (cl - base[0]), base[2] + flap_eff * (cm - base[2])
        top = stall_shape(cl, clmax_k, drop)
        if sym:                     # a symmetric section stalls alike both ways
            n = AN >= 0
            top = np.where(n, top, -np.interp(-AN, AN[n], top[n]))
        cd = cd + 0.1 * np.clip(cl - top, 0, None) * np.clip(4 * np.sin(np.radians(AN)), 0, 1)
        CL[:, j], CD[:, j], CM[:, j] = extend(top, cd, cm, CN90=CN90, shift=0.35 * d)
    t = dict(alpha=ALPHA.tolist(), delta=list(map(float, deltas)), cl=CL.tolist(), cd=CD.tolist(), cm=CM.tolist(), Re=Re)
    a = np.array(AN)
    sel = (a >= -2) & (a <= 4)
    t['meta'] = dict(cla=float(np.polyfit(a[sel], base[0][sel], 1)[0] * 57.2958), a0=float(np.interp(0, base[0][(a > -10) & (a < 6)], a[(a > -10) & (a < 6)])),
                     clmax=float(np.array(CL)[:, deltas.index(0.0) if 0.0 in deltas else 0].max()), cdmin=float(base[1][(a > -6) & (a < 8)].min()), cm0=float(np.interp(0, a, base[2])))
    os.makedirs(CACHE, exist_ok=True)
    part = '%s.%d' % (path, os.getpid())            # checks run side by side: nobody reads a half-written file
    with open(part, 'w') as f:
        json.dump(t, f)
    os.replace(part, path)
    return t


def with_line(t, a0=4.0):
    """A wing's line needs its lift at the geometric angle unstalled, or the induced angle falls short near stall. cl_g runs on straight past a0, each column by its own start"""
    a = np.array(t['alpha'])
    cl = np.array(t['cl'])
    i0 = int(np.argmin(np.abs(a - a0)))
    i1 = int(np.argmin(np.abs(a + 2.0)))
    slope = (cl[i0] - cl[i1]) / (a[i0] - a[i1])
    g = cl.copy()
    wide = (a > a0) & (a <= 40)
    g[wide] = cl[i0][None, :] + (a[wide] - a[i0])[:, None] * slope[None, :]
    far = a > 40
    g[far] = g[np.where(a <= 40)[0][-1]][None, :]
    return dict(t, cl_g=g.tolist())


def slotted(t, flaps):
    """a slotted flap on a plain table's zero-deflection column. flaps: deflection, zero-lift lift increment, increment to maximum lift, moment and
    drag. Stall comes earlier by the difference of the two lift increments, over the section's slope: cl(alpha) is the clean curve shifted that far,
    plus the maximum lift's increment"""
    a = np.array(t['alpha'])
    j = t['delta'].index(0.0)
    cl, cd, cm = (np.array(t[k])[:, j] for k in ('cl', 'cd', 'cm'))
    slope = np.radians(t['meta']['cla'])
    CL, CD, CM = (np.zeros((len(a), len(flaps) + 1)) for _ in range(3))
    CL[:, 0], CD[:, 0], CM[:, 0] = cl, cd, cm
    fade = 1 - smooth((np.abs(a) - 25) / 30)
    for k, f in enumerate(flaps):
        da = (f['dcl0'] - f['dclmax']) / slope
        CL[:, k + 1] = np.interp(a + da, a, cl) + fade * f['dclmax']
        CD[:, k + 1] = np.interp(a + da, a, cd) + fade * f['dcd']
        CM[:, k + 1] = np.interp(a + da, a, cm) + fade * f['dcm']
    return dict(t, delta=[0.0] + [f['deg'] for f in flaps], cl=CL.tolist(), cd=CD.tolist(), cm=CM.tolist())
