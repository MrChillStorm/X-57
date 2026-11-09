"""The X-57's built-up model: Mod III's wing in strips, each in its own air, on the parts proved on the P2006T. This is what ships.

    c = config(hl)          for builtup.Aero; genx57.py --built

Geometry is NASA's OpenVSP model [data/geometry.json]: 2 deg of twist at the root falling to none at the tip, the flap, the ailerons, the discs. Strips meet at each propeller's hub and edge, so each half of each disc's
slipstream, with its own swirl, has its own strips: a stopped disc takes its own lift away. The fuselage and tail are the P2006T's, so their fits carry over. Sections are NeuralFoil's on NASA's coordinates, fully turbulent as
the CFD ran.

What's fitted, all to the CFD and none to NASA's simulator but the rates:
  x57fit.py    section lift, moment and drag at flaps 0, 10, 30, power off, by the section's angle; past 8 deg the moment by the body's incidence; the moment the propellers leave, on J; the rate derivatives, to the tunnel's
  k_sat            the one fit on the slipstream. The ring-vortex wake (ringwake.py, as the V-22's) gives the axial speed at each strip, no constants: a
                   jet no taller than the chord does less than that, the more the stronger it is. k_jet stays 1. contraction is momentum theory's at 58 KCAS
  jet_keep         the part of the angle a jet doesn't straighten out, on how the lift grows with incidence
  jet_turn         the flap turns the jet by this share of its own angle. The thrust's lost axial part is the drag the flaps-30 runs show and flaps 10 don't
  k_tail_jet       the slipstream at the stabilator, on its power with the propellers on
"""
import json, math, os
import numpy as np
import builtup as BU
import liftline as LL
import p2006t as P
import propulsion as PR
import sections as SEC
import x57data as X
import x57flex as FX
import xmlfun as xf

HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
IN = 0.0254
F = 1 / 0.3048
W = GEO['wing']
C0, C1, HALF = W['panels'][0]['c0'], W['panels'][0]['c1'], W['panels'][0]['y1']
Z_WING = 87.0
TWIST = ([0.0, 24.0, 136.61, 181.97, 189.74], [2.0, 1.82, 0.71, 0.11, 0.0])         # [VSP]
Y_FLAP = (24.14, 136.61)
Y_AIL = (136.61, 181.97)
HUBS = GEO['high_lift']['hubs_in']
R_HL = GEO['high_lift']['diameter_in'] / 2
AIL_HINGE = 0.75
REF = GEO['reference']
FIT_FILE = os.path.join(HERE, 'data', 'x57_section_fit.json')

FLAPS = [dict(deg=10.0, dcl0=0.45, dclmax=0.30, dcm=-0.07, dcd=0.006), dict(deg=30.0, dcl0=1.05, dclmax=0.70, dcm=-0.17, dcd=0.03)]
FIT = dict(clmax_k=1.10, cd_misc=0.0, k_jet=1.0, k_sat=0.70, contraction=0.9, ring_wake=1, k_tail_jet=0.5, jet_turn=0.75, jet_keep=0.3, jet_out=1.0, k_jet_roll=0.03)
FIT.update(json.loads(os.environ.get('X57_FIT', '{}')))


def chord(y):
    return C0 + (C1 - C0) * abs(y) / HALF


def le(y):
    return W['origin_in'][0] + 0.7 * (C0 - chord(y))


def edges():
    e = {0.0, 12.0, Y_FLAP[0], Y_FLAP[1], Y_AIL[1], HALF, 171.0}
    for h in HUBS:
        e |= {h[1], h[1] + R_HL}
    e.add(HUBS[0][1] - R_HL)
    e = sorted(e)
    return [y for i, y in enumerate(e) if i == 0 or y - e[i - 1] > 2.0]        # the flap's and first disc's edges nearly meet


def strips():
    e = edges()
    half = []
    for i, (y0, y1) in enumerate(zip(e, e[1:])):
        ys = np.linspace(y0, y1, 21)
        cs = np.array([chord(y) for y in ys])
        area = float(np.trapezoid(cs, ys)) * IN ** 2
        c = area / ((y1 - y0) * IN)
        ym = 0.5 * (y0 + y1)
        x0 = le(ym) * IN
        flap = Y_FLAP[0] - 1.0 <= y0 and y1 <= Y_FLAP[1] + 1.0
        ail = Y_AIL[0] - 1.0 <= y0 and y1 <= Y_AIL[1] + 1.0
        half.append(dict(i=i, y0=y0 * IN, y1=y1 * IN, c=c, area=area, x4=x0 + 0.25 * c, x34=x0 + 0.75 * c, inc=float(np.interp(ym, *TWIST)), kind='wing', flap=flap, aileron=ail, thick=0.15,
                         sec='wf' if flap else 'wa' if ail else 'w0'))
    z = Z_WING * IN
    rows = []
    for side in (-1, 1):
        for o in (half[::-1] if side < 0 else half):
            ya, yb = (-o['y1'], -o['y0']) if side < 0 else (o['y0'], o['y1'])
            rows.append(dict(o, side=side, ya=ya, yb=yb, za=z, zb=z, ym=0.5 * (ya + yb), zm=z))
    return rows


def corrected(t, cols):
    """a table with the fit's section corrections added: cols gives the fit's flap for each deflection column, so flaps add to the clean wing's"""
    if not os.path.exists(FIT_FILE):
        return t
    C = json.load(open(FIT_FILE))
    a = np.array(t['alpha'])
    out = dict(t)
    fade = np.clip((36 - np.abs(a)) / 8, 0, 1)
    for k in ('cl', 'cd', 'cm'):
        v = np.array(t[k])
        for j, flap in enumerate(cols):
            for f in (0.0, flap) if flap else (0.0,):
                c = C[str(f)]
                v[:, j] += np.interp(a, c['a'], c['d' + k]) * fade
        out[k] = v.tolist()
    return out


# NACA Report 661 (Abbott and Greenberg 1939), table I: the NACA 23012 with a sealed 0.20c plain flap in the variable-density tunnel, effective
# R 8.2 to 8.6 million, upright model. Deflection (deg, down positive): section clmax, zero-lift angle (deg), lift slope (per deg).
NACA661 = {-30: (0.88, 9.4, .095), -20: (1.02, 8.0, .092), -15: (1.18, 6.1, .095), -10: (1.38, 3.8, .099), -5: (1.52, 1.4, .100),
           -2: (1.62, -0.2, .099), -1: (1.68, -0.7, .098), 0: (1.74, -1.2, .100), 5: (1.83, -3.6, .097), 10: (1.92, -6.0, .094),
           15: (2.02, -9.1, .087), 20: (2.11, -10.5, .086), 30: (2.19, -12.3, .086)}


def naca661_share(delta):
    """The share of a plain flap's lift increment that the section keeps at maximum lift, from NACA661: each deflection's clmax increment over
    the increment its own lift line gives at the plain section's zero lift (as held_cl is taken). Down about 0.4, up about 0.8. Deflections
    under 5 deg are left out: their increments are a few hundredths, at the table's rounding"""
    c0, z0, _ = NACA661[0]
    d = [x for x in sorted(NACA661) if abs(x) >= 5]
    return np.interp(delta, d, [(NACA661[x][0] - c0) / (NACA661[x][2] * (z0 - NACA661[x][1])) for x in d])


def shifted_lift(aa, plain, dcl0, j0, k_max):
    """The aileron's lift as the plain section's, corrected as the wing's, shifted by each deflection's equivalent angle, as sections.slotted
    does the flap: cl(alpha, delta) = cl0(alpha + dcl0 (1 - k) / slope) + k dcl0. dcl0 is NeuralFoil's increment at zero lift (held
    above); k (k_max, one for each column) the share of it kept at maximum lift. Below its zero lift the plain curve runs on straight, so a
    large up deflection doesn't meet the plain section's negative stall. Returned before the fit's corrections, which corrected() then adds to
    every column"""
    corr = np.zeros_like(aa)
    if os.path.exists(FIT_FILE):
        C = json.load(open(FIT_FILE))['0.0']
        corr = np.interp(aa, C['a'], C['dcl']) * np.clip((36 - np.abs(aa)) / 8, 0, 1)
    p = plain + corr
    near = (aa > -15) & (aa < 5)
    az = float(np.interp(0.0, p[near], aa[near]))
    slope = (np.interp(az + 2, aa, p) - np.interp(az - 2, aa, p)) / 4.0
    ext = lambda x: np.where(x >= az, np.interp(x, aa, p), slope * (x - az))
    fade = 1 - np.clip((np.abs(aa) - 25) / 30, 0, 1)
    out = np.zeros((len(aa), len(dcl0)))
    for j, d0 in enumerate(dcl0):
        out[:, j] = p if j == j0 else ext(aa + d0 * (1 - k_max[j]) / slope) + fade * k_max[j] * d0
    return out - corr[:, None]


def tables():
    kw = dict(n_crit=9.0, xtr=0.03)
    re = lambda c: 1.9e6 * c / 0.65
    xy = np.loadtxt(os.path.join(HERE, 'data', 'sections', 'wing_root.dat'), skiprows=1)
    xy[:, 0] -= xy[:, 0].min()
    k = FIT['clmax_k']
    base = SEC.table(xy, re(0.65), clmax_k=k, **kw)
    # The aileron, a plain flap: what each deflection adds to lift, drag and moment is NeuralFoil's at the section's own zero lift, held across the
    # attached range and handed back to NeuralFoil's own by 14 deg, where the plain section's lift peaks. Read at the strip's angle instead, it faded
    # where the lifting line takes an up aileron's strips, 7 to 8 deg up: NeuralFoil's plain section loses lift slope from about 4 deg and the
    # up-deflected one doesn't. NASA's CFD has the aileron linear from -25 to +5 deg and nearly the same at 2 and 8 deg [TM sec. 4.1.3]. At zero lift
    # NeuralFoil's up and down increments are nearly alike, as in thin-airfoil theory; at the strips' cruise angle the down one is two-thirds of the up.
    # Nothing scales them: gains of 1.5 on lift and 2.0 on moment were there before, and steepened a deflected section's lift slope to 0.18 per degree.
    # Its lift past that is the plain section's shifted by each deflection's equivalent angle (shifted_lift), keeping NACA Report 661's share of the
    # increment at maximum lift (naca661_share). Handed back to NeuralFoil's increment instead, the aileron near stall kept 0.8 of the CFD's roll
    # up and 1.5 times it down: NeuralFoil's flap at -25 deg climbs 0.13 per degree from 8 to 14 deg, twice its plain section's 0.064, where
    # Report 661's up-deflected flaps keep within 0.008 of the plain section's slope
    ail = SEC.table(xy, re(0.6), hinge=AIL_HINGE, deltas=(-25.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 18.0), clmax_k=k, **kw)
    j0 = ail['delta'].index(0.0)
    aa = np.array(ail['alpha'])
    c0 = np.array(ail['cl'])[:, j0]
    near = (aa > -15) & (aa < 5)
    a_zero = float(np.interp(0.0, c0[near], aa[near]))
    w = np.clip((14.0 - np.abs(aa)) / 6.0, 0, 1)[:, None]
    held_cl = None
    for n in ('cl', 'cd', 'cm'):
        v = np.array(ail[n])
        d = v - v[:, [j0]]
        held = np.array([[np.interp(a_zero, aa, d[:, j]) for j in range(d.shape[1])]])
        held_cl = held[0] if n == 'cl' else held_cl
        ail[n] = (v[:, [j0]] + w * held + (1 - w) * d).tolist()
    ail['cl'] = shifted_lift(aa, np.array(ail['cl'])[:, j0], held_cl, j0, naca661_share(np.array(ail['delta']))).tolist()
    T = dict(w0=corrected(base, [0.0]), wf=corrected(SEC.slotted(base, FLAPS), [0.0, 10.0, 30.0]), wa=corrected(ail, [0.0] * 8))
    T['w0'] = dict(T['w0'], delta=[0.0], cl=np.array(T['w0']['cl'])[:, :1].tolist(), cd=np.array(T['w0']['cd'])[:, :1].tolist(), cm=np.array(T['w0']['cm'])[:, :1].tolist())
    T.update(P.tail_tables(re, kw))
    return {n: SEC.with_line(t) if n in ('w0', 'wf', 'wa') else t for n, t in T.items()}


def body(sections, cuts):
    return P.body_stations(sections, cuts)


# The cruise propellers' swirl across the jet: NASA's CFD loads them by Goldstein [TM sec. 3.1], so the optimum's profile (builtup.swirl_profile), here at
# J 1.0, mid-way through their cruise and climb (across J 0.8 to 1.6 the strip beside the nacelle takes 1.74 to 1.95 of the even swirl, the next 0.95 to 1.01).
# The jet's inner edge is the nacelle, 13.8 in across [X-57.vsp3]. The high-lift discs' BEM loading isn't published: they keep the even swirl
SWIRL = dict(J=1.0, blades=GEO['cruise']['blades'], hub_r=13.8 / 2 * IN / 0.3048)


def nacelles():
    """the tip's by its discs' size; the high-lift nacelles lumped, six to a side at their mean span: a small part of a small part"""
    tip_y = GEO['cruise']['hub_in'][1]
    d = 13.8
    z = GEO['cruise']['hub_in'][2]
    x0, x1 = 126.4, 166.0
    tip = body([dict(x=x0, z=z, h=0, w=0), dict(x=x0 + 8, z=z, h=0.8 * d, w=0.8 * d), dict(x=x0 + 16, z=z, h=d, w=d), dict(x=x1 - 14, z=z, h=0.8 * d, w=0.8 * d), dict(x=x1, z=z, h=0, w=0)], [x0, x0 + 8, x0 + 16, 150.0, x1])
    d = 6.5
    z = HUBS[0][2]
    x0, x1 = 130.0, 168.0
    hl = body([dict(x=x0, z=z, h=0, w=0), dict(x=x0 + 6, z=z, h=0.8 * d, w=0.8 * d), dict(x=x0 + 12, z=z, h=d, w=d), dict(x=150.0, z=z, h=0.8 * d, w=0.8 * d), dict(x=x1, z=z, h=0, w=0)], [x0, x0 + 6, x0 + 12, 148.0, x1])
    y_hl = float(np.mean([h[1] for h in HUBS]))
    for st in hl['stations']:
        for k in ('dSv', 'dSl', 'plan', 'side'):
            st[k] *= 6
    return (tip, tip_y * IN), (hl, y_hl * IN)


def config(hl):
    rows = strips()
    S = REF['S_ft2'] / F ** 2
    a_sec = 6.2
    k_ind = BU.fit_k_ind(rows, lambda y: chord(y / IN) * IN, HALF * IN, a_sec)
    ysplit = Y_FLAP[1] * IN
    gt = BU.loads(rows, a_sec, k_ind, S)
    zw = Z_WING * IN
    fus = P.fuselage()
    mid = le(0) + 0.5 * C0
    x_le, x_te = le(0) * IN, (le(0) + C0) * IN
    c = dict(ring_wake=FIT.get('ring_wake', 0), rows=rows, S_ft2=REF['S_ft2'], tables=tables(), a_sec=a_sec, k_ind=k_ind, incidence=0.0, k_cd=1.0, y_split=ysplit, contraction=FIT['contraction'], k_jet=FIT['k_jet'], k_sat=FIT['k_sat'], jet_turn=FIT['jet_turn'], jet_keep=FIT['jet_keep'], jet_out=FIT['jet_out'], k_jet_roll=FIT['k_jet_roll'] if hl else 0.0,
             k_tail_jet=FIT['k_tail_jet'], k_wake=1.0, wake_ke=1.6 / (math.pi * 15.0), wake_te=(x_te, zw - 0.02, C0 * IN), k_munk=0.89, k_munk_aft=0.5, cdc=1.2, eps_lag=0.35,
             frozen='ctl/hold-props', fuselage_at_wing=dict(a=math.sqrt(fus['half_h'](mid) * fus['half_w'](mid)), z=fus['zc'](mid)),
             coef=dict(S=REF['S_ft2'], b=REF['b_ft'], c=REF['c_ft'], ref=REF['mrs_in']))

    def wash(st, y=0.0):
        if y and x_le < st['x'] < x_te:
            return 0.0
        return float(LL.field(rows, gt, st['x'], y, st['z']))
    for st in fus['stations']:
        st['upwash'] = wash(st)
    c['bodies'] = [dict(name='f', stations=fus['stations'], xmax=fus['xmax'])]
    (tip, ty), (hln, hy) = nacelles()
    for nm, b, y in (('t', tip, ty), ('n', hln, hy)):
        for st in b['stations']:
            st['upwash'] = wash(st, y)
        c['bodies'].append(dict(name=nm, stations=b['stations'], xmax=b['xmax'], y=y, sides=(-1, 1)))
    # discs: their slipstreams here, their forces from propulsion.py
    hub = GEO['cruise']['hub_in']
    D = [dict(name=s, hub=(hub[0] * IN, sp * hub[1] * IN, hub[2] * IN), D=GEO['cruise']['diameter_in'] * IN, spin=sp, thrust='prop/%s/thrust-lbs' % s, torque='prop/%s/q-air-ftlb' % s, tip=True, swirl=SWIRL) for s, _, sp in PR.SIDES]
    if hl:
        for i, q in enumerate(PR.hl_stations(), 1):
            D.append(dict(name='h%d' % i, hub=tuple(v * IN for v in q), D=GEO['high_lift']['diameter_in'] * IN, spin=PR.spin_hl(i), thrust='prop/hl/d%d/thrust-lbs' % i, torque='prop/hl/d%d/q-air-ftlb' % i, test='prop/hl/d%d/power' % i))
    c['discs'] = D
    c['disc_upwash'] = {}
    st = P.stabilator()
    hs = [-1.5, -1.1, -0.8, -0.5, -0.25, 0.0, 0.25, 0.5, 0.8, 1.1, 1.5, 2.0]
    Lw = BU.lumps(rows, ysplit)
    Es = BU.eps_lumps(rows, st, zw, hs, S, gt, Lw)
    fn = P.fin()
    sweep = math.atan2(fn['strips'][-1]['x4'] - fn['strips'][0]['x4'], fn['strips'][-1]['z'] - fn['strips'][0]['z'])
    fn.update(AR_eff=1.55 * fn['h'] ** 2 / fn['S'], cos_sweep=math.cos(sweep))
    c['ground'] = BU.ground_tables(rows, gt, st, zw, REF['b_ft'] / F)
    c['tyre'] = (X.GEAR['main_x_in'] / 12, X.GEAR['ground_z_in'] / 12)
    c['flex'] = FX.modal(hl)
    c['flex_module'] = FX
    # the propellers' in-plane force at incidence, up on the mode; the high-lift set shares one figure, spread by its hubs
    al = xf.p('aero/alpha-rad')
    c['flex_forces'] = {sd: [xf.M(xf.p('prop/%s/inplane-lbs' % s), al)] + ([xf.M(xf.v(sum(FX.phi_at(c['flex'], h[1] / 12.0) for h in HUBS) / 12.0), xf.p('prop/hl/inplane-lbs'), al)] if hl else []) for sd, s in (('L', 'l'), ('R', 'r'))}
    c.update(stab=st, eps_h=hs, eps_lumps=Lw, eps_lump=Es, eps_shifts=BU.WAKE_SHIFTS, e_tail=0.74, a_tail=a_sec, eta_tail=P.FIT['eta_tail'], stab_zero=0.0, fin=fn, sidewash=P.FIT['sidewash'])
    area = BU.wetted_drag(fus['length'], fus['wet'], 1 + 60 / (fus['length'] / fus['d_eq']) ** 3 + fus['length'] / fus['d_eq'] / 400)
    area += 2 * BU.wetted_drag(tip['length'], tip['wet'], 1.3)
    area += 2 * 6 * BU.wetted_drag(hln['length'], hln['wet'], 1.3)
    pod = P.body_stations(GEO['gear_pod']['sections'], [151.95, 190.55])
    area += 2 * BU.wetted_drag(pod['length'], pod['wet'], 1.3)
    c.update(drag_area_ft2=area * F * F + FIT['cd_misc'] * REF['S_ft2'], gear_ft2=X.GEAR_DRAG_AREA_FT2, drag_point=(3.5, 1.7))
    if os.path.exists(FIT_FILE):
        d = json.load(open(FIT_FILE))
        c['body_fit'], c['rate_fit'] = d.get('body'), d.get('rates')
        # with the high lift on the rates' correction holds its value from 2 deg under the CFD's power-off stall (as the checks' 'below stall'): blown, the sections don't stall
        c['rate_hold'] = dict(flaps=[0.0, 10.0, 30.0], alpha=[13.0, 10.0, 10.0])
        c['beta_fit'] = d.get('beta')
        c['rud_fit'] = d.get('rud')
        if d.get('cruise'):
            cr = json.load(open(os.path.join(HERE, 'data', 'aero_tables.json')))['cruise']
            c['cruise_roll'] = dict(J=cr['J'], pitch=cr['pitch'], dCl=cr['dCl'], k=d['cruise']['k'])
        if hl:
            c['hl_fit'] = d.get('hl')
        c['ground_fit'] = d.get('ground')
        if hl:
            c['ground_hl_fit'] = d.get('ground_hl')
    return c
