"""The Tecnam P2006T: the airplane the built-up method is proved on, because it has really flown. Its fuselage and tail are the X-57's.

Sources:
  [AFM]    P2006T Aircraft Flight Manual, 4th edition, Rotax 912 S3
  [AIAA]   Nicolosi, De Marco, Della Vecchia, Stability, Flying Qualities and Parameter Estimation of a Twin-Engine CS-23/FAR 23 Certified Light Aircraft, AIAA 2010-7947
  [ICAS]   Pascale, Nicolosi, Design and Aerodynamic Analysis of a Light Twin-Engine Propeller Aircraft, ICAS 2008
  [EWADE]  Nicolosi, De Marco, Della Vecchia, The Role of Flight Tests and of Flight Simulation in Aircraft Design, EWADE 2011
  [TM]     NASA/TM-20250001715, table 1
  [VSP]    NASA's OpenVSP model of the X-57 (data/geometry.json)

Stations in inches on the OpenVSP model's axes: x aft, y right, z up.

No drawing of the wing is published. The flight manual's three-view, scaled by the fuselage's 8.7 m and the tail's 3.3 m, gives a chord of 1.44 m to y 3.11 m and 0.89 m at y 5.6 m. That
is 14.76 m2, the manual's area, and a mean chord of 1.340 m, NASA's 4.40 ft. Both came out without being fitted, so the planform stands.
"""
import json, math, os
import numpy as np
import sections as SEC

HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
IN = 0.0254
FT = 0.3048

REF = dict(S_ft2=158.88, b_ft=37.40, c_ft=4.40)                  # [AFM, TM]; AIAA rounds S to 14.8 m2
WING = dict(le_x=140.4,                                          # 2.99 m behind the nose in the three-view; the nose is at 22.66 [VSP]
            z=87.0,                                              # the X-57's; the side view's wing stands 0.56 m above the stabilator, which is the same
            y_break=3.11, y_tip=5.60, c_root=1.44, c_tip=0.89, dihedral=1.0,
            y_flap=(0.60, 3.11), y_aileron=(3.11, 4.86), aileron_hinge=0.80, thick_root=0.15, thick_tip=0.12, cl_design=0.4)
WINGLET = dict(rise=0.60, reach=0.10, c_tip=0.35, toe=(-7.0, -4.0))     # height [ICAS]; 11.4 m with them, 11.2 m without; toe and twist [ICAS]
SURFACES = dict(stabilator=(-15.0, 4.0), aileron=(-20.0, 17.0), rudder=26.0, flaps=(0.0, 20.0, 40.0))  # [AFM]; its stabilator's up and down are the wrong way round. Takeoff flap 20 deg [20220005440]
MASS = dict(kg=1180.0, Ixx=1617.0, Iyy=1927.0, Izz=2931.0, cg_mac=0.25, cg_below_chord=0.20)    # kg m2 [AIAA table 2]; CG height: 20 % of chord below the wing [ICAS]
TEST = dict(kg=1140.0, cg_mac=0.19, kias=110.0, hp=1000.0)      # the identification's flight condition [AIAA V]
NACELLE = dict(y=1.60, x=(116.0, 180.0), w=0.64, h=0.72, z=84.0, dcl_wing=-0.048)   # [AFM three-view]; dcl: its lift loss, constant with incidence [ICAS fig. 14]
PROP = dict(hub=(128.5, 1.60 / IN, 86.0), D=1.78, blades=2, rpm=2388.0, spin=1)    # [AFM]. Both turn the same way [AIAA IV.B]
ENGINE = dict(takeoff_kw=73.5, continuous_kw=69.0, idle_kw=1.5)                     # [AFM]
# Full-throttle power against altitude from the manual's cruise tables: % of take-off power, ISA, 2388 rpm or nearest
POWER_LAPSE = dict(alt_ft=[0.0, 3000.0, 6000.0, 9000.0, 12000.0, 15000.0], frac=[1.0, 0.885, 0.80, 0.72, 0.64, 0.565])
FLIGHT_GEAR_DRAG_FT2 = 4.0                                       # NASA's gear, the X-57's fit


def wing_edges():
    """strip edges by y, so that the slipstream, the flap and the aileron each start at one"""
    return [0.0, 0.60, 0.75, 1.175, 1.60, 2.025, 2.45, 3.11, 3.70, 4.30, 4.86, 5.60]


def chord(y):
    w = WING
    return w['c_root'] if y <= w['y_break'] else w['c_root'] + (w['c_tip'] - w['c_root']) * (y - w['y_break']) / (w['y_tip'] - w['y_break'])


def strips(winglets=True, span=None):
    """the wing, left to right, with its winglets. Areas add up to the reference area between the wingtips"""
    w = WING
    z0 = w['z'] * IN
    tanG = math.tan(math.radians(w['dihedral']))
    e = wing_edges()
    if span:
        e = [y * span / e[-1] for y in e]
    half = []
    for i, (y0, y1) in enumerate(zip(e, e[1:])):
        ys = np.linspace(y0, y1, 41)
        cs = np.array([chord(y) for y in ys])
        area = float(np.trapezoid(cs, ys))
        c = area / (y1 - y0)
        ym = 0.5 * (y0 + y1)
        half.append(dict(i=i, y0=y0, y1=y1, z0=z0 + y0 * tanG, z1=z0 + y1 * tanG, c=c, area=area, x4=w['le_x'] * IN + 0.25 * c, x34=w['le_x'] * IN + 0.75 * c,
                         inc=0.0, kind='wing', flap=w['y_flap'][0] - 1e-6 <= y0 and y1 <= w['y_flap'][1] + 1e-6,
                         aileron=w['y_aileron'][0] - 1e-6 <= y0 and y1 <= w['y_aileron'][1] + 1e-6,
                         thick=w['thick_root'] if ym <= w['y_break'] else w['thick_root'] + (w['thick_tip'] - w['thick_root']) * (ym - w['y_break']) / (w['y_tip'] - w['y_break'])))
    if winglets:
        g = WINGLET
        y, z, c = half[-1]['y1'], half[-1]['z1'], WING['c_tip']
        nodes = [(y, z), (y + 0.7 * g['reach'], z + 0.3 * g['rise']), (y + g['reach'], z + g['rise'])]
        cs = [c, 0.5 * (c + g['c_tip']), g['c_tip']]
        for k in (0, 1):
            (a, b), (p, q) = nodes[k], nodes[k + 1]
            cm = 0.5 * (cs[k] + cs[k + 1])
            ln = math.hypot(p - a, q - b)
            xle = WING['le_x'] * IN + (WING['c_tip'] - cm) * 0.8             # its leading edge sweeps back
            half.append(dict(i=len(half), y0=a, y1=p, z0=b, z1=q, c=cm, area=cm * ln, x4=xle + 0.25 * cm, x34=xle + 0.75 * cm, inc=g['toe'][k], kind='winglet',
                             flap=False, aileron=False, thick=0.12))
    rows = []
    for side in (-1, 1):
        for o in (half[::-1] if side < 0 else half):
            ya, yb, za, zb = (-o['y1'], -o['y0'], o['z1'], o['z0']) if side < 0 else (o['y0'], o['y1'], o['z0'], o['z1'])
            rows.append(dict(o, side=side, ya=ya, yb=yb, za=za, zb=zb, ym=0.5 * (ya + yb), zm=0.5 * (za + zb)))
    return rows


def body_stations(sections, edges_in):
    """a body cut at edges along x. The sections' heights and widths are carried in their squares, which gives round ends. Area: a rounded box. Returns the fine curves
    and each piece's plan and side area and the change of its two circles' areas, for Munk's force"""
    from scipy.interpolate import PchipInterpolator as I
    x = np.array([s['x'] for s in sections])
    h2, w2, zc = I(x, [s['h'] ** 2 for s in sections]), I(x, [s['w'] ** 2 for s in sections]), I(x, [s['z'] for s in sections])
    xs = np.linspace(x[0], x[-1], 500)
    h, w, z = np.sqrt(np.maximum(h2(xs), 0)), np.sqrt(np.maximum(w2(xs), 0)), zc(xs)
    a, b = w / 2, h / 2
    per = np.pi * (3 * (a + b) - np.sqrt((3 * a + b) * (a + 3 * b)))
    area = 0.86 * h * w
    st = []
    Sv, Sl = np.pi * w ** 2 / 4, np.pi * h ** 2 / 4
    for x0, x1 in zip(edges_in, edges_in[1:]):
        m = (xs >= x0) & (xs <= x1)
        st.append(dict(x=0.5 * (x0 + x1) * IN, z=float(np.mean(z[m])) * IN, plan=float(np.trapezoid(w[m], xs[m])) * IN ** 2, side=float(np.trapezoid(h[m], xs[m])) * IN ** 2,
                       dSv=float(np.interp(x1, xs, Sv) - np.interp(x0, xs, Sv)) * IN ** 2, dSl=float(np.interp(x1, xs, Sl) - np.interp(x0, xs, Sl)) * IN ** 2))
    return dict(stations=st, wet=float(np.trapezoid(per, xs)) * IN ** 2, volume=float(np.trapezoid(area, xs)) * IN ** 3, length=(x[-1] - x[0]) * IN,
                xmax=float(xs[np.argmax(area)]) * IN, d_eq=float(np.sqrt(4 * area.max() / np.pi)) * IN, half_h=lambda q: float(np.interp(q, xs, h)) / 2 * IN, zc=lambda q: float(np.interp(q, xs, z)) * IN,
                half_w=lambda q: float(np.interp(q, xs, w)) / 2 * IN)


def fuselage():
    return body_stations(GEO['fuselage'], [22.66, 40, 62, 90, 120, 150, 180, 210, 243, 280, 326, 351.3])


def nacelle():
    n = NACELLE
    x0, x1 = n['x']
    w, h = n['w'] / IN, n['h'] / IN
    sec = [dict(x=x0, z=n['z'], h=0, w=0), dict(x=x0 + 0.12 * (x1 - x0), z=n['z'], h=0.55 * h, w=0.6 * w), dict(x=x0 + 0.42 * (x1 - x0), z=n['z'], h=h, w=w),
           dict(x=x0 + 0.75 * (x1 - x0), z=n['z'], h=0.85 * h, w=0.85 * w), dict(x=x1, z=n['z'], h=0, w=0)]
    return body_stations(sec, [x0, x0 + 8, x0 + 16, x0 + 25, 150.0, x1])


def stabilator(n=3):
    s = GEO['stabilator']
    half = 3.3 / 2                                                 # with its tips [AFM]; the OpenVSP panels stop at 1.57 m
    e = np.linspace(0, half, n + 1)
    c = s['chord_in'] * IN
    x0 = s['le_in'][0] * IN
    out = []
    for i, (a, b) in enumerate(zip(e, e[1:])):
        out.append(dict(i=i, y=0.5 * (a + b), dy=b - a, c=c, area=c * (b - a), x4=x0 + 0.25 * c, x34=x0 + 0.75 * c, z=s['le_in'][2] * IN))
    return dict(strips=out, S=2 * half * c, b=2 * half, hinge=s['hinge_in'] * IN)


def fin(n=4):
    """fin and rudder by height, from the fuselage's top at the fin. Rudder: the hinge line's slope [VSP]"""
    P = GEO['fin']
    z = np.array([p['z0'] for p in P] + [P[-1]['z1']])
    le = np.array([p['x0'] for p in P] + [P[-1]['x1']])
    c = np.array([p['c0'] for p in P] + [P[-1]['c1']])
    hinge = GEO['surfaces']['Rudder-Hinge']
    hx, hz = hinge['loc_in'][0], hinge['loc_in'][2]
    tan = math.tan(math.radians(90 - hinge['rot_deg'][1]))
    e = np.linspace(72.0, z[-1], n + 1)
    out = []
    for i, (a, b) in enumerate(zip(e, e[1:])):
        zs = np.linspace(a, b, 21)
        cs, ls = np.interp(zs, z, c), np.interp(zs, z, le)
        cm = float(np.mean(cs))
        lm = float(np.mean(ls))
        zm = 0.5 * (a + b)
        hingex = hx - (hz - zm) * tan
        out.append(dict(i=i, z=zm * IN, dz=(b - a) * IN, c=cm * IN, area=cm * (b - a) * IN ** 2, x4=(lm + 0.25 * cm) * IN, x34=(lm + 0.75 * cm) * IN, cf=(lm + cm - hingex) / cm))
    return dict(strips=out, S=sum(s['area'] for s in out), h=(e[-1] - e[0]) * IN, cf=float(np.mean([s['cf'] for s in out])))


def propeller_table():
    """Ct and the in-plane force on J and power, for a governed propeller. The MTV-21's blades aren't published: the X-57's MT cruise blade stands in, with two blades at 1.78 m"""
    path = os.path.join(HERE, 'data', 'p2006t_prop.json')
    if os.path.exists(path):
        return json.load(open(path))
    import discs as Dd
    bl = Dd.cruise_blade()
    bl.B = PROP['blades']
    r = PROP['D'] / 2 / FT
    k = r / bl.R
    bl.c, bl.dr, bl.R = bl.c * k, bl.dr * k, r
    Js = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.7, 2.0]
    Bs = [8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0, 46.0]
    n = PROP['rpm'] / 60
    ct, cp, cn = (np.zeros((len(Js), len(Bs))) for _ in range(3))
    for i, J in enumerate(Js):
        for j, b in enumerate(Bs):
            ct[i, j], cp[i, j], cn[i, j] = Dd.solve(bl, J, b, n=n)
    Cps = [0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.06, 0.07, 0.08, 0.1]
    CT, CN = (np.zeros((len(Js), len(Cps))) for _ in range(2))
    bf = np.linspace(Bs[0], Bs[-1], 400)
    for i in range(len(Js)):
        c = np.maximum.accumulate(np.interp(bf, Bs, cp[i])) + 1e-9 * np.arange(len(bf))
        b = np.interp(Cps, c, bf)
        CT[i], CN[i] = np.interp(b, Bs, ct[i]), np.interp(b, Bs, cn[i])
    t = dict(J=Js, Cp=Cps, Ct=CT.tolist(), Cn=np.maximum(CN, 0).tolist())
    with open(path, 'w') as f:
        json.dump(t, f, separators=(',', ':'))
    return t


def tables(tunnel=False):
    """the sections. Flight: riveted metal, so transition is early [ICAS lists roughness and gaps among its drag]. Tunnel: Re 0.6 million with trip strips at 5 % [ICAS]"""
    kw = dict(n_crit=6.0, xtr=0.3) if not tunnel else dict(n_crit=9.0, xtr=0.05)
    re = (lambda c: 0.6e6 * c / 1.34) if tunnel else (lambda c: 3.8e6 * c / 1.34)     # the stall's Reynolds number: where maximum lift matters
    w = WING
    xy = {t: SEC.naca6a(t, w['cl_design']) for t in (0.15, 0.135, 0.12)}
    k = FIT.get("clmax_k", 0.92)
    T = {}
    base = SEC.table(xy[0.15], re(1.44), clmax_k=k, **kw)
    T['root'] = SEC.slotted(base, FLAPS)
    d = (-20.0, -12.0, -6.0, 0.0, 6.0, 12.0, 17.0)
    T['mid'] = SEC.table(xy[0.135], re(1.2), hinge=w['aileron_hinge'], deltas=d, clmax_k=k, **kw)
    T['tip'] = SEC.table(xy[0.12], re(0.95), hinge=w['aileron_hinge'], deltas=d, clmax_k=k, **kw)
    T['winglet'] = SEC.table(SEC.four_digit(0.12), re(0.6), clmax_k=1.0, **kw)
    T.update(tail_tables(re, kw))
    return T


def tail_tables(re, kw):
    """the stabilator's and fin's sections, shared with the X-57"""
    F = fin()
    dr = (-28.0, -19.0, -12.0, -6.0, 0.0, 6.0, 12.0, 19.0, 28.0)
    return dict(stab=SEC.table(SEC.four_digit(0.11), re(0.778), clmax_k=0.92, drop=0.5, sym=True, **kw), fin=fin_table(SEC.table(SEC.four_digit(0.12), re(1.0), hinge=1 - F['cf'], deltas=dr, clmax_k=1.0, **kw)))


def fin_table(t):
    """The rudder's increments, scaled by FIT's rudder_k. A low surface's line needs its lift at the geometric angle unstalled, or the induced angle falls short at large sideslip: cl_g
    runs on straight from 8 deg. On the section's own curve, its leading edge's vortex adds lift as the sine squared (Polhamus), by FIT's fin_vortex"""
    a = np.array(t['alpha'])
    r = np.radians(a)
    j0 = t['delta'].index(0.0)
    k = FIT['rudder_k']
    cl, cd, cm = ((lambda x: x[:, [j0]] + k * (x - x[:, [j0]]))(np.array(t[n])) for n in ('cl', 'cd', 'cm'))
    i8 = int(np.argmin(np.abs(a - 8.0)))
    lin = cl[i8, j0] / math.sin(math.radians(8.0))
    g = cl.copy()
    wide = np.abs(a) > 8.0
    g[wide] = (lin * np.sin(r[wide]))[:, None] + (cl[i8, :] - cl[i8, j0])[None, :]
    add = FIT['fin_vortex'] * np.sign(r) * np.sin(r) ** 2 * np.clip((60 - np.abs(a)) / 30, 0, 1)
    return dict(t, cl_g=g.tolist(), cl=(cl + add[:, None]).tolist(), cd=(cd + (add * np.sin(r))[:, None]).tolist(), cm=cm.tolist())


# The slotted flap, per its section: deflection, zero-lift increment, increment to maximum lift, moment, drag. Fitted: the two lift columns to the manual's stall speeds
# and the moment to let the stabilator reach them: at the forward CG it is nearly on its stop at the stall, flaps up or down
FLAPS = [dict(deg=20.0, dcl0=0.65, dclmax=0.68, dcm=-0.05, dcd=0.008), dict(deg=40.0, dcl0=1.05, dclmax=0.95, dcm=-0.08, dcd=0.06)]

FIT = dict(
    incidence=-1.0,      # deg, wing to the datum of the tunnel's alpha. Fig. 14: zero lift at -1.6 deg, wing and body; the section's is -2.7
    install=0.91,        # thrust: the flight tests' net installed efficiency, 0.70 at low speed to 0.82 at high [EWADE], against the bare stand-in blade's 0.77 and 0.90
    stab_zero=1.2,       # deg, the flight stabilator's reading below the tunnel's zero: the two disagree by this at the same CG and lift (AIAA fig. 11a, ICAS fig. 18). Fitted; the shape stays a check
    clmax_k=0.72,        # sections' maximum lift on NeuralFoil's. Fitted to the manual's flaps-up stall. The tufts show the wing separating beside the nacelles first [ICAS fig. 15]
    cd_misc=0.0106,      # on the reference area: cooling, gaps, leaks. Fitted to the manual's cruise
    # On NASA's CFD of the same fuselage and tail, on the X-57's wing, so the P2006T's flight data stay an independent check. By a build without nacelles, at flaps 0, alpha 2:
    eta_tail=0.80,       # stabilator: CFD's Cm delta, -1.86 on the P2006T's reference. It holds the fuselage's and the wing's wakes and the gaps
    sidewash=0.0,        # fin: the CFD's Cn beta and CY beta. Datcom's factor for a high wing, 0.81, leaves the fin a third short
    fin_vortex=1.0,      # the dorsal fin's vortex lift: the CFD's Cn beta steepening past 5 deg
    rudder_k=1.50,       # rudder: the CFD's CY delta r. It holds the swept hinge line and the fin's thick dorsal root, which a plain-flap section misses
)
