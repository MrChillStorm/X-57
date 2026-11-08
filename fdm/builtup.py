"""The built-up aerodynamic model: an airplane as parts, each with its own air. This is the ASK 21's method with propellers added. All of it is JSBSim XML.

    a = Aero(cfg); text = a.build()     cfg: see genp2006t.py

Every part's forces come from the air at that part: translation, rotation about the centre of gravity, and what the other parts do to it.
  Wing:      strips in a lifting line solved ahead of time, on their own section tables, each with its own speed. So a strip in a slipstream, or behind a stopped disc, has its own lift.
  Discs:     thrust, torque and in-plane force, a slipstream by momentum theory with a contraction, and its swirl.
  Bodies:    Munk's force and crossflow drag by stations, with the wing's upwash and downwash on them. The fuselage's crossflow also reaches the wing: a high wing gets its dihedral effect that way.
  Tail:      strips in the wing's downwash, by the wing's trailing vortices with the wake's height, and its wake's loss of dynamic pressure.
  Fin:       strips in the fuselage's sidewash, with the wing's effect by Datcom.

Forces go in about the structural origin, which is the model's AERORP.
"""
import math
import numpy as np
from xmlfun import *
import liftline as LL

R2D = 57.29577951308232
F = 1 / 0.3048                 # ft per m


def fit_k_ind(rows, chord, y_tip, a_sec, n=400):
    """the coarse strips against a fine line on the same planform: the factor that gives them the same lift slope"""
    e = np.linspace(-y_tip, y_tip, n + 1)
    fine = [dict(ya=a, yb=b, za=0.0, zb=0.0, c=chord(abs(0.5 * (a + b))), area=chord(abs(0.5 * (a + b))) * (b - a)) for a, b in zip(e, e[1:])]
    flat = [dict(r, za=0.0, zb=0.0) for r in rows if r['kind'] == 'wing']
    k = 1.0
    cf = LL.solve(fine, np.ones(n), a_sec)[2]
    for _ in range(6):
        k *= 1 + 2.0 * (LL.solve(flat, np.ones(len(flat)), a_sec, k)[2] - cf) / cf
    return k


def loads(rows, a_sec, k_ind, S, y_split):
    """the wing's circulation per unit lift, whole, and inboard and outboard of y_split by their own lift"""
    cl = LL.solve(rows, np.ones(len(rows)), a_sec, k_ind)[0]
    area = np.array([r['area'] for r in rows])
    c = np.array([r['c'] for r in rows])
    inner = np.array([abs(r['ym']) <= y_split for r in rows])
    g = 0.5 * c * cl
    return g / ((cl * area).sum() / S), g * inner / ((cl * area)[inner].sum() / S), g * ~inner / ((cl * area)[~inner].sum() / S)


def eps_tables(rows, stab, gi, go, z_wing, hs):
    """downwash at each tail strip per unit of the wing's inboard and outboard lift, by the strip's height over the wake"""
    Ei, Eo = [], []
    for h in stab['strips']:
        E = [[], []]
        for hh in hs:
            dz = h['z'] - z_wing - hh
            for k, g in enumerate((gi, go)):
                E[k].append(float(np.mean([-LL.field(rows, g, h['x4'], yy, h['z'], dz) for yy in (h['y'], -h['y'])])))
        Ei.append(E[0])
        Eo.append(E[1])
    return Ei, Eo


def ground_tables(rows, gt, stab, z_wing, b):
    """What the ground does to the wing's own downwash and at the tail, by the wake's mirror image: each a factor on wing height over span, 1 far away"""
    hb = np.array([0.02, 0.05, 0.08, 0.11, 0.14, 0.18, 0.22, 0.28, 0.35, 0.45, 0.6, 0.8, 1.0, 1.5])
    n = len(rows)
    area = np.array([r['area'] for r in rows])
    own = LL.influence(rows)
    N = LL.normals(rows)
    P = np.array([[r['ym'], r['zm']] for r in rows])
    wing, tail = [], []
    h0 = stab['strips'][1]
    e_own = float(np.mean([-LL.field(rows, gt, h0['x4'], yy, h0['z'], 0.0) for yy in (h0['y'], -h0['y'])]))
    for x in hb:
        ground = z_wing - x * b
        img = np.zeros((n, n))
        for j, r in enumerate(rows):
            for ye, ze, sgn in ((r['yb'], r['zb'], 1.0), (r['ya'], r['za'], -1.0)):
                d = P - (ye, 2 * ground - ze)
                r2 = (d ** 2).sum(axis=1)
                # the image's circulation is opposite
                img[:, j] -= ((-sgn / (4 * np.pi * r2))[:, None] * np.c_[-d[:, 1], d[:, 0]] * N).sum(axis=1)
        wing.append(float(np.clip(1 + (img @ gt * area).sum() / (own @ gt * area).sum(), 0.05, 1.0)))
        im = []
        for yy in (h0['y'], -h0['y']):
            m = [dict(r, ya=r['ya'], yb=r['yb'], za=2 * ground - r['za'], zb=2 * ground - r['zb']) for r in rows]
            im.append(LL.field(m, -gt, h0['x4'], yy, h0['z'], 0.0))
        tail.append(float(np.clip(1 + (-np.mean(im)) / e_own, 0.05, 1.0)))
    wing[-1] = tail[-1] = 1.0
    x_w = float(np.mean([r['x4'] for r in rows]))
    return dict(hb=hb.tolist(), wing=wing, tail=tail, z_wing_ft=z_wing * F, b_ft=b * F, tail_arm_ft=(h0['x4'] - x_w) * F)


def wetted_drag(L, wet, ff, k=1.0, V=65.0, nu=1.5e-5):
    """flat-plate friction by Reynolds number, a form factor, on a wetted area: drag area, m2"""
    return 0.455 / math.log10(V * L / nu) ** 2.58 * ff * wet * k


class Aero:
    def __init__(s, c):
        s.c = c
        s.g = Graph()
        s.parts = {k: [] for k in 'XYZLMN'}
        s.flexQ = {'L': [], 'R': []}
        s.keep = {'bu/CLw', 'bu/total/X', 'bu/total/Y', 'bu/total/Z', 'bu/total/L', 'bu/total/M', 'bu/total/N'}

    def fn(s, name, body, desc=None):
        return s.g.fn(name, body, desc)

    # the flow at the airplane
    def kin(s):
        s.U, s.V, s.W = p('velocities/u-aero-fps'), p('velocities/v-aero-fps'), p('velocities/w-aero-fps')
        s.Pp, s.Qq, s.Rr = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        s.RHO = p('atmosphere/rho-slugs_ft3')
        cgx = s.fn('bu/cg-x-ft', Q(p('inertia/cg-x-in'), v(12)))
        cgy = s.fn('bu/cg-y-ft', Q(p('inertia/cg-y-in'), v(12)))
        cgz = s.fn('bu/cg-z-ft', Q(p('inertia/cg-z-in'), v(12)))
        s.u0 = s.fn('bu/u0', S(s.U, M(s.Qq, cgz), M(s.Rr, cgy)))
        s.v0 = s.fn('bu/v0', S(s.V, M(s.Rr, cgx), M(v(-1), s.Pp, cgz)))
        s.w0 = s.fn('bu/w0', S(s.W, M(v(-1), s.Pp, cgy), M(v(-1), s.Qq, cgx)))
        s.V2 = s.fn('bu/V2', S(SQR(s.U), SQR(s.V), SQR(s.W), v(1e-4)))
        s.Vt = s.fn('bu/Vt', SQ(s.V2))
        # ground effect, on the wing's own height: JSBSim's h_b-mac reads the moment reference, which here is the structural origin
        g = s.c['ground']
        s.fn('bu/h-b', Q(S(p('position/h-agl-ft'), D(v(g['z_wing_ft']), cgz)), v(g['b_ft'])), 'wing height over span')
        s.ge = s.fn('bu/ge-phi', T1('bu/h-b', g['hb'], g['wing']), 'on the wing\'s induced angles: the image of its wake')
        s.fn('bu/h-b-tail', D(p('bu/h-b'), M(v(g['tail_arm_ft'] / g['b_ft']), SIN(p('attitude/theta-rad')))), 'the tail sinks as the nose comes up')
        s.get = s.fn('bu/ge-tail', T1('bu/h-b-tail', g['hb'], g['tail']), 'on the tail\'s downwash, the same way')

    def vel(s, x, y, z):
        """the airplane's velocity through the air at a point (ft, structural), in body axes"""
        lin = lambda c, rate: M(v(c), rate) if c else None
        u = S(*[t for t in (s.u0, lin(-z, s.Qq), lin(-y, s.Rr)) if t])
        vv = S(*[t for t in (s.v0, lin(-x, s.Rr), lin(z, s.Pp)) if t])
        w = S(*[t for t in (s.w0, lin(y, s.Pp), lin(x, s.Qq)) if t])
        return u, vv, w

    def add(s, tagn, x, y, z, Fx=None, Fy=None, Fz=None, Mx=None, My=None, Mz=None, L_yz=None, N_y=None):
        """body forces at a structural point (ft), moments about the structural origin (arms -x, y, -z). L_yz, N_y: the side and vertical forces' moments
        already summed, for a force along one line: one read of it, not two"""
        L, Mm, N = [], [], []
        if Fx:
            s.parts['X'].append(Fx)
            Mm.append(M(v(-z), Fx))
            if y:
                N.append(M(v(-y), Fx))
        if Fy:
            s.parts['Y'].append(Fy)
            if L_yz is None:
                L.append(M(v(z), Fy))
            if N_y is None:
                N.append(M(v(-x), Fy))
        if Fz:
            s.parts['Z'].append(Fz)
            Mm.append(M(v(x), Fz))
            if y and L_yz is None:
                L.append(M(v(y), Fz))
        L += [L_yz] if L_yz else []
        N += [N_y] if N_y else []
        L += [Mx] if Mx else []
        Mm += [My] if My else []
        N += [Mz] if Mz else []
        for k, lst in (('L', L), ('M', Mm), ('N', N)):
            if lst:
                s.parts[k].append(s.fn('bu/%s/%s' % (k, tagn), S(*lst)))

    # propellers
    def discs(s):
        s.D = []
        gated = False
        for d in s.c['discs']:
            if d.get('test') and not gated and hasattr(s.g, 'fcs'):       # the high-lift discs: at rest while they're stopped, as their motors' channel
                s.g.fcs.channel('high-lift slipstream', execute='prop/hl/live')
                gated = True
            nm = 'prop/' + d['name']
            x, y, z = (q * F for q in d['hub'])
            Dm = d['D'] * F
            A = math.pi * Dm ** 2 / 4
            u, vv, w = s.vel(x, y, z)
            if d.get('thrust'):            # a disc run by another system: only its slipstream here
                uh = s.fn('bu/disc/%s/u' % d['name'], MAX(u, v(1.0)))
                T, q = p(d['thrust']), p(d['torque'])
                if d.get('test'):          # the harness can put the CFD's own thrust and torque through the slipstream
                    on = GT(p('bu/test/hl-thrust'), v(-900))
                    w = p(d['test'])          # a failed disc stays failed
                    T, q = IFTHEN(on, M(p('bu/test/hl-thrust'), w), T), IFTHEN(on, M(p('bu/test/hl-torque'), w), q)
                a = s.fn('bu/disc/%s/a-fps' % d['name'], M(v(0.5), D(SQ(S(SQR(uh), Q(M(v(2 / A), MAX(T, v(0))), s.RHO))), uh)), 'axial speed added at the disc')
                nm = 'bu/disc/' + d['name']
                s.fn(nm + '/vt-fps', Q(MAX(q, v(0)), M(s.RHO, S(uh, a), v(A * 0.7 * Dm / 2))), 'swirl at 0.7 R')
                s.D.append(dict(d, nm=nm, x=x, y=y, z=z, u=uh, v=vv, w=w, a=a, ad=s.jet(nm, a, uh), R=Dm / 2))
                if s.c.get('jet_turn') and d.get('test'):
                    # the flap turns the jet: the thrust's axial part lost, which the airframe feels as drag
                    th = M(v(math.radians(s.c['jet_turn'])), p('ctl/flap-deg'))
                    s.add('turn-' + d['name'], x, y, z, Fx=s.fn(nm + '/turn-lbs', M(v(-1), MAX(T, v(0)), D(v(1), COS(th)))))
                continue
            uh = s.fn(nm + '/u-fps', MAX(u, v(1.0)))
            n = s.fn(nm + '/n', Q(p(nm + '/rpm'), v(60)))
            J = s.fn(nm + '/J', MIN(v(2.0), Q(uh, M(n, v(Dm)))))
            Cp = s.fn(nm + '/Cp', MIN(v(0.1), Q(M(p(nm + '/power-hp'), v(550)), M(s.RHO, tag('pow', n, v(3)), v(Dm ** 5)))))
            t = d['table']
            k4 = s.fn(nm + '/k4', M(s.RHO, SQR(n), v(Dm ** 4)))
            Ct = M(v(d.get('install', 1.0)), T2(nm + '/J', nm + '/Cp', t['J'], t['Cp'], t['Ct']))
            # the table is a governed propeller's. Below what the fine stop takes the speed sags in truth: a little power can't drive a stopped aeroplane with the thrust of a turning fine pitch.
            # So never more than the power can give: momentum theory at rest, power over speed moving, at 0.95 of the ideal
            Pw = M(p(nm + '/power-hp'), v(550 * 0.95))
            cap = MIN(Q(Pw, uh), tag('pow', M(v(2 * A), s.RHO, SQR(Pw)), v(1 / 3.0)))
            T = s.fn(nm + '/thrust-lbs', IFTHEN(GT(p(nm + '/run'), v(0.5)), MIN(M(Ct, k4), cap), M(v(-0.5 * d['feather_ft2']), s.RHO, SQR(uh))))
            s.fn(nm + '/eta', Q(M(T, uh), MAX(M(p(nm + '/power-hp'), v(550)), v(1.0))))
            a = s.fn(nm + '/a-fps', M(v(0.5), D(SQ(S(SQR(uh), Q(M(v(2 / A), MAX(T, v(0))), s.RHO))), uh)), 'axial speed added at the disc')
            q = s.fn(nm + '/torque-ftlb', Q(M(p(nm + '/power-hp'), v(550)), M(v(2 * math.pi), MAX(n, v(1)))))
            s.fn(nm + '/vt-fps', Q(q, M(s.RHO, S(uh, a), v(A * 0.7 * Dm / 2))), 'swirl at 0.7 R')
            Cn = T2(nm + '/J', nm + '/Cp', t['J'], t['Cp'], t['Cn'])
            N = s.fn(nm + '/normal-lbs-rad', M(Cn, k4, p(nm + '/run')))
            s.D.append(dict(d, nm=nm, x=x, y=y, z=z, u=uh, v=vv, w=w, T=T, N=N, q=q, a=a, ad=s.jet(nm, a, uh), R=Dm / 2))
        if gated:
            s.g.fcs.channel('built-up aerodynamics')

    def jet(s, nm, a, u):
        """the axial speed a wing feels: a jet no taller than the chord does less than strip theory says, and less the stronger it is"""
        return s.fn(nm + '/a-eff', M(a, v(s.c['k_jet']), D(v(1), M(v(s.c.get('k_sat', 0.0)), Q(a, S(u, a, v(1)))))))

    def disc_forces(s):
        for d in s.D:
            if d.get('thrust'):
                continue
            up = s.c['disc_upwash'][d['name']]
            ay = s.fn(d['nm'] + '/inc-y', ATAN2(d['v'], d['u']))
            az = s.fn(d['nm'] + '/inc-z', ATAN2(S(d['w'], M(v(up), s.CLw, s.Vt)), d['u']), 'with the wing\'s upwash')
            s.add(d['name'], d['x'], d['y'], d['z'], Fx=d['T'], Fy=M(v(-1), d['N'], ay), Fz=M(v(-1), d['N'], az), Mx=M(v(-d['spin']), d['q']))

    def wash_of(s, r):
        def sw_of(dy, Rs, d):
            core = 0.35 * Rs
            return float(np.mean(np.where((np.abs(dy) < Rs) & (np.abs(dy) > core), -np.sign(dy), 0.0))) * d['spin']
        """what the discs do to a strip: for each, the part of it in the slipstream, the speed there and the swirl. None if none reaches it"""
        out = []
        for d in s.D:
            Rs = d['R'] * s.c['contraction']
            y = np.linspace(r['ya'], r['yb'], 41) * F
            dy = y - d['y']
            inside = float(np.mean(np.abs(dy) < Rs))
            if inside <= 0 or r['kind'] != 'wing':
                continue
            x = r['x4'] * F - d['x']
            ku = 1 + x / math.hypot(x, d['R'])
            if s.c.get('ring_wake'):        # a tube of ring vortices, as the V-22's wake, over the strip's span
                import ringwake
                zz = r['zm'] * F - d['z']
                out.append((d, ringwake.mean_axial(x, dy, zz, d['R'], s.c['contraction']), sw_of(dy, Rs, d)))
                continue
            core = 0.35 * Rs
            sw = float(np.mean(np.where((np.abs(dy) < Rs) & (np.abs(dy) > core), -np.sign(dy), 0.0)))
            # a jet over an unflapped outer strip lifts less than the strip's own speed says [fitted to the contactor failure's rolling moment]
            out.append((d, inside * ku * (1.0 if r['flap'] or d.get('tip') else s.c.get('jet_out', 1.0)), sw * d['spin']))
        return out

    # the wing
    def wing(s):
        c = s.c
        rows = c['rows']
        n = len(rows)
        fxd = c.get('flex')
        fxm = c.get('flex_module')
        Kf = LL.operator(rows, c['a_sec'], c['k_ind'])
        Nrm = LL.normals(rows)
        T = c['tables']
        info = []
        fz = c['fuselage_at_wing']
        for k, r in enumerate(rows):
            nm = 'bu/w%s%d' % ('L' if r['side'] < 0 else 'R', r['i'])
            x4, x34, y, z = r['x4'] * F, r['x34'] * F, r['ym'] * F, r['zm'] * F
            ny, nz = -Nrm[k][0], Nrm[k][1]
            # the fuselage's crossflow at the strip: a circle's doublet
            yy, zz = r['ym'], r['zm'] - fz['z']
            kk = min(fz['a'] ** 2 / (yy * yy + zz * zz), 1.0) if r['kind'] == 'wing' else 0.0
            th = math.atan2(zz, yy)
            c2, s2 = math.cos(2 * th), math.sin(2 * th)
            kv = ny + nz * kk * s2 - ny * kk * c2
            kw = nz + nz * kk * c2 + ny * kk * s2
            u, vv, w = s.vel(x34, y, z)
            # the wing's bending: the strip rises with the mode, which takes flow from under it, and leans with the mode's slope
            fp = fsl = 0.0
            if fxd and r['kind'] == 'wing':
                sd = 'L' if r['side'] < 0 else 'R'
                fp, fsl = fxm.phi_at(fxd, y), r['side'] * fxm.slope_at(fxd, y)
            lean = M(v(fsl * nz), p('flex/q%s-ft' % sd)) if fsl else None
            if fp:
                w = D(w, M(v(fp), p('flex/qd%s-fps' % sd)))
            wash = s.wash_of(r)
            u, vv = s.fn(nm + '/uf', u), s.fn(nm + '/vf', vv)       # each is used more than once, and each read takes a lock in FlightGear
            U = s.fn(nm + '/u', S(u, *[M(v(a), d['ad']) for d, a, _ in wash]))
            # the jet straightens the flow through it less than adding its speed along the shaft would: part of the angle stays
            keep = c.get('jet_keep', 0.0)
            kj = s.fn(nm + '/kj', S(v(1), M(v(keep), Q(S(*[M(v(a), d['ad']) for d, a, _ in wash]), MAX(u, v(10)))))) if wash and keep else None
            wj = M(w, kj) if kj else w
            Wn = s.fn(nm + '/wn', S(M(v(kv), vv), M(v(kw), wj), *([M(lean, vv)] if lean else []), *[M(v(sw), p(d['nm'] + '/vt-fps')) for d, _, sw in wash if sw]), 'flow up through the strip')
            V2 = s.fn(nm + '/V2', S(SQR(U), SQR(Wn), v(1e-4)))
            Vl = s.fn(nm + '/V', SQ(V2))
            phi = ATAN2(Wn, U)
            ag = s.fn(nm + '/ag', M(S(phi, v(math.radians(r['inc'] + c['incidence'] + r.get('dinc', 0.0)))), v(R2D)), 'geometric section alpha (deg)')
            _, v4, w4 = s.vel(x4, y, z)
            v4 = s.fn(nm + '/v4', v4)
            if fp:
                w4 = D(w4, M(v(fp), p('flex/qd%s-fps' % sd)))
            w4 = M(w4, kj) if kj else w4
            phi4 = s.fn(nm + '/phi4', ATAN2(S(M(v(ny), v4), M(v(nz), w4), *([M(lean, v4)] if lean else [])), U), 'flow at the bound vortex: the force\'s direction (rad)')
            key = r.get('sec') or ('winglet' if r['kind'] == 'winglet' else 'root' if r['thick'] >= 0.145 else 'mid' if r['thick'] >= 0.128 else 'tip')
            tb = T[key]
            col = 'ctl/flap-deg' if r['flap'] else ('ctl/ail-left-deg' if r['side'] < 0 else 'ctl/ail-right-deg') if r['aileron'] else None
            j0 = tb['delta'].index(0.0)
            if col:
                tab = lambda arr, var, tb=tb, col=col: T2(var, col, tb['alpha'], tb['delta'], arr)
            else:
                tab = lambda arr, var, tb=tb, j0=j0: T1(var, tb['alpha'], np.array(arr)[:, j0])
            clg = s.fn(nm + '/clg', tab(tb.get('cl_g', tb['cl']), nm + '/ag'))
            s.fn(nm + '/g', M(clg, Vl))
            info.append(dict(nm=nm, Vl=Vl, V2=V2, phi4=phi4, r=r, tab=tab, tb=tb, x4=x4, y=y, z=z, ny=ny, nz=nz, fp=fp, lean=lean, sd=sd if fp else None))
        mir = [next(j for j, q in enumerate(rows) if q['side'] == -r['side'] and q['i'] == r['i']) for r in rows]
        left = [k for k in range(n) if rows[k]['side'] < 0]
        gs, ga = {}, {}
        for j in left:
            gL, gR = p(info[j]['nm'] + '/g'), p(info[mir[j]]['nm'] + '/g')
            i = rows[j]['i']
            gs[j] = s.fn('bu/wing/gs%d' % i, S(gL, gR), 'left and right strip %d: lift times speed' % i)
            ga[j] = s.fn('bu/wing/ga%d' % i, D(gL, gR))
        for k in left:
            i = rows[k]['i']
            Ks = 0.5 * (Kf[k, left] + Kf[k, [mir[j] for j in left]])
            Ka = 0.5 * (Kf[k, left] - Kf[k, [mir[j] for j in left]])
            sym = s.fn('bu/wing/ais%d' % i, S(*[M(v(a), gs[j]) for a, j in zip(Ks, left) if abs(a) > 1e-7]), 'induced angle times speed, the two sides alike')
            asy = s.fn('bu/wing/aia%d' % i, S(*[M(v(a), ga[j]) for a, j in zip(Ka, left) if abs(a) > 1e-7]))
            for kk, comb in ((k, S(sym, asy)), (mir[k], D(sym, asy))):
                d = info[kk]
                fade = T1(d['nm'] + '/ag', [-180, -70, -45, -25, 25, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])
                d['ai'] = s.fn(d['nm'] + '/ai', M(Q(comb, d['Vl']), s.ge, fade), 'induced angle (rad)')
        ref = s.c['S_ft2']
        kcd = c['k_cd']
        for d in info:
            nm, r, tb = d['nm'], d['r'], d['tb']
            s.fn(nm + '/a', D(p(nm + '/ag'), M(d['ai'], v(R2D))), 'section alpha (deg)')
            cl = s.fn(nm + '/cl', d['tab'](tb['cl'], nm + '/a'))
            cd0 = d['tab'](tb['cd'], nm + '/a')
            cm = s.fn(nm + '/cm', d['tab'](tb['cm'], nm + '/a'))
            vre = tb['Re'] * 1.57e-4 / (r['c'] * F)        # the table's speed at this chord, sea level
            fre = tag('pow', MAX(Q(M(s.RHO, d['Vl']), v(0.0023769 * vre)), v(0.2)), v(-0.15))
            cd = s.fn(nm + '/cd', M(cd0, fre, v(kcd)))
            phie = s.fn(nm + '/phie', D(d['phi4'], d['ai']))
            qA = s.fn(nm + '/qA', M(v(0.5 * r['area'] * F * F), s.RHO, d['V2']))
            Fx = s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd, COS(phie)))))
            Fn = s.fn(nm + '/Fn', M(qA, v(-1), S(M(cl, COS(phie)), M(cd, SIN(phie)))))
            Ms = s.fn(nm + '/Msec', M(qA, v(r['c'] * F), cm))
            # along one line, so its side and vertical parts' moments take one read of the force
            ny, nz, X_, Y_, Z_ = d['ny'], d['nz'], d['x4'], d['y'], d['z']
            has = abs(ny) > 1e-9
            lean = d['lean']
            Fy = (M(S(v(ny), lean), Fn) if has else M(lean, Fn)) if lean else M(v(ny), Fn) if has else None
            kL = (Z_ * ny if has else 0.0) + Y_ * nz
            L_yz = M(S(v(kL), M(v(Z_), lean)), Fn) if lean else M(v(kL), Fn)
            N_y = (M(S(v(-X_ * ny), M(v(-X_), lean)), Fn) if has else M(v(-X_), lean, Fn)) if lean else M(v(-X_ * ny), Fn) if has else None
            if d['fp']:
                s.flexQ[d['sd']].append(M(v(-d['fp'] * d['nz']), Fn))
            s.add(nm[3:].replace('/', '-'), d['x4'], d['y'], d['z'], Fx=Fx, Fy=Fy, Fz=M(v(d['nz']), Fn), My=M(v(d['nz']), Ms), Mz=M(v(-d['ny']), Ms) if abs(d['ny']) > 1e-9 else None, L_yz=L_yz, N_y=N_y)

        def part(sel):
            return S(*[M(p(d['nm'] + '/cl'), d['V2'], v(d['r']['area'] * F * F / ref)) for d in info if sel(d['r'])])
        ysplit = c['y_split']
        slow = MAX(s.V2, v(900))        # at rest with the propellers running the coefficient has no meaning: below 30 ft/s it stops growing
        s.CLi = s.fn('bu/CLw-in', Q(part(lambda r: abs(r['ym']) <= ysplit), slow), 'the wing\'s lift inboard of the flap\'s end')
        s.CLo = s.fn('bu/CLw-out', Q(part(lambda r: abs(r['ym']) > ysplit), slow))
        s.CLw = s.fn('bu/CLw', S(s.CLi, s.CLo), 'the wing\'s lift coefficient, on the reference area')
        inner = [d for d in info if d['r']['kind'] == 'wing' and abs(d['r']['ym']) < ysplit and d['r']['ym'] > 0]
        s.cdw = s.fn('bu/cd-inner', M(v(1.0 / len(inner)), S(*[p(d['nm'] + '/cd') for d in inner])))
        s.winfo = info

    def wake_shape(s):
        """the wing's wake leaves its trailing edge; its height over a point behind, and the dynamic pressure lost in it"""
        c = s.c
        al = p('aero/alpha-rad')
        ang = s.fn('bu/wake-ang', D(al, M(v(c['wake_ke']), s.CLw)), 'the wake\'s slope to body x (rad)')
        return ang, Q(SIN(ang), MAX(COS(ang), v(0.2)))

    def wake_loss(s, nm, x, z, xte, zte, cref, cd, tanw, k):
        dist = x - xte
        h = s.fn(nm + '/hw', D(v(z - zte), M(v(dist), tanw)), 'height above the wing\'s wake (m)')
        xc = dist / cref
        dq = 2.42 / (xc + 0.3)
        bw = 0.68 * cref * math.sqrt(xc + 0.15)
        return h, M(v(dq * k), SQ(cd), tag('exp', M(v(-0.69 / bw ** 2), Q(SQR(h), cd))))

    # the tail
    def tail(s):
        c = s.c
        st = c['stab']
        tb = c['tables']['stab']
        j0 = tb['delta'].index(0.0)
        ct = np.array(tb['cl'])[:, j0]
        cdt = np.array(tb['cd'])[:, j0]
        cmt = np.array(tb['cm'])[:, j0]
        xte, zte, cref = c['wake_te']
        cd = MAX(s.cdw, v(0.004))
        ang, tanw = s.wake_shape()
        tanw = s.fn('bu/wake-tan', tanw)
        dh = c['eps_h']
        lag = c['eps_lag']
        uw2 = S(SQR(s.U), SQR(s.W))
        ad = s.fn('bu/alphadot', IFTHEN(GT(p(c.get('frozen', 'bu/test/frozen')), v(0.5)), p('bu/test/alphadot'), Q(M(p('aero/alphadot-rad_sec'), uw2), S(uw2, v(100)))), 'for the downwash\'s lag (rad/s). A frozen state has none but the harness\'s')
        info = []
        ARt = st['b'] ** 2 / st['S']
        for side in (-1, 1):
            for h in st['strips']:
                nm = 'bu/h%s%d' % ('L' if side < 0 else 'R', h['i'])
                x4, x34, y, z = h['x4'] * F, h['x34'] * F, side * h['y'] * F, h['z'] * F
                hw, loss = s.wake_loss(nm, h['x4'], h['z'], xte, zte, cref, cd, tanw, c['k_wake'])
                Ei, Eo = c['eps_in'][h['i']], c['eps_out'][h['i']]
                dist = (h['x4'] - xte) * F
                eps = s.fn(nm + '/eps', D(M(S(M(T1(nm + '/hw', dh, Ei), s.CLi), M(T1(nm + '/hw', dh, Eo), s.CLo)), s.get),
                                          M(v(lag * dist), Q(ad, MAX(s.Vt, v(10))))), 'downwash (rad), with its lag')
                jet = []
                for d in s.D:
                    # a jet spreads as it runs back, about a tenth of the distance, and slows as it does
                    yy, zz = y - d['y'], z - d['z']
                    b = d['R'] + 0.1 * (x4 - d['x'])
                    if abs(yy) > 2 * b:
                        continue
                    jet.append(M(v(s.c['k_tail_jet'] * 2 * d['R'] / b * math.exp(-yy * yy / b ** 2)), d['a'], tag('exp', M(v(-1 / b ** 2), SQR(S(v(zz), M(v(d['x'] - x4), tanw)))))))
                eta = s.fn(nm + '/eta', MAX(D(v(1.0), loss), v(0.15)), 'the wake\'s dynamic pressure')
                u, vv, w = s.vel(x34, y, z)
                U = s.fn(nm + '/u', S(u, *jet))
                Wn = s.fn(nm + '/wn', D(w, M(eps, s.Vt)))
                V2 = s.fn(nm + '/V2', M(S(SQR(U), SQR(Wn), v(1e-4)), eta, v(c['eta_tail'])))
                phi = ATAN2(Wn, U)
                ag = s.fn(nm + '/ag', S(M(phi, v(R2D)), p('ctl/stab-deg'), v(c.get('stab_zero', 0.0))))
                _, v4, w4 = s.vel(x4, y, z)
                phi4 = s.fn(nm + '/phi4', ATAN2(D(w4, M(eps, s.Vt)), U))
                clg = s.fn(nm + '/clg', T1(nm + '/ag', tb['alpha'], ct))
                info.append(dict(nm=nm, V2=V2, phi4=phi4, x4=x4, y=y, z=z, h=h, clg=clg))
        CLg = s.fn('bu/ht-CLgeo', S(*[M(d['clg'], v(d['h']['area'] / st['S'])) for d in info]))
        fade = T1('bu/hR0/ag', [-180, -70, -45, -25, 25, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])
        ai = s.fn('bu/ht-ai', M(Q(CLg, v(math.pi * ARt * c['e_tail'] + c['a_tail'])), fade), 'induced angle (rad)')
        for d in info:
            nm, h = d['nm'], d['h']
            s.fn(nm + '/a', D(p(nm + '/ag'), M(ai, v(R2D))))
            cl = s.fn(nm + '/cl', T1(nm + '/a', tb['alpha'], ct))
            cd_ = s.fn(nm + '/cd', M(T1(nm + '/a', tb['alpha'], cdt), v(c['k_cd'])))
            cm = T1(nm + '/a', tb['alpha'], cmt)
            phie = s.fn(nm + '/phie', D(d['phi4'], ai))
            qA = s.fn(nm + '/qA', M(v(0.5 * h['area'] * F * F), s.RHO, d['V2']))
            s.add(nm[3:], d['x4'], d['y'], d['z'], Fx=s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd_, COS(phie))))), Fz=s.fn(nm + '/Fz', M(qA, v(-1), S(M(cl, COS(phie)), M(cd_, SIN(phie))))), My=M(qA, v(h['c'] * F), cm))

    def fin(s):
        c = s.c
        f = c['fin']
        tb = c['tables']['fin']
        info = []
        for st in f['strips']:
            nm = 'bu/v%d' % st['i']
            x4, x34, z = st['x4'] * F, st['x34'] * F, st['z'] * F
            u, vv, w = s.vel(x34, 0.0, z)
            U = s.fn(nm + '/u', u)
            Vn = s.fn(nm + '/vn', S(vv, M(v(c['sidewash']), s.V)), 'with the fuselage\'s and the wing\'s sidewash')
            V2 = s.fn(nm + '/V2', S(SQR(U), SQR(Vn), v(1e-4)))
            ag = s.fn(nm + '/ag', M(ATAN2(Vn, U), v(R2D)))
            _, v4, _w = s.vel(x4, 0.0, z)
            phi4 = s.fn(nm + '/phi4', ATAN2(S(v4, M(v(c['sidewash']), s.V)), U))
            clg = s.fn(nm + '/clg', T2(nm + '/ag', 'bu/rud-fin-deg', tb['alpha'], tb['delta'], tb.get('cl_g', tb['cl'])))
            info.append(dict(nm=nm, V2=V2, phi4=phi4, x4=x4, z=z, st=st, clg=clg))
        CYg = s.fn('bu/vt-CLgeo', S(*[M(d['clg'], v(d['st']['area'] / f['S'])) for d in info]))
        fade = T1('bu/v1/ag', [-180, -70, -45, -30, 30, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])
        ai = s.fn('bu/vt-ai', M(Q(CYg, v(math.pi * f['AR_eff'] * 0.9 + 6.0)), fade), 'induced angle (rad)')
        kc = f['cos_sweep']
        for d in info:
            nm, st = d['nm'], d['st']
            s.fn(nm + '/a', D(p(nm + '/ag'), M(ai, v(R2D))))
            cl = s.fn(nm + '/cl', T2(nm + '/a', 'bu/rud-fin-deg', tb['alpha'], tb['delta'], tb['cl']))
            cd_ = s.fn(nm + '/cd', M(T2(nm + '/a', 'bu/rud-fin-deg', tb['alpha'], tb['delta'], tb['cd']), v(c['k_cd'])))
            phie = s.fn(nm + '/phie', D(d['phi4'], ai))
            cm = T2(nm + '/a', 'bu/rud-fin-deg', tb['alpha'], tb['delta'], tb['cm'])
            qA = s.fn(nm + '/qA', M(v(0.5 * st['area'] * F * F * kc), s.RHO, d['V2']))
            s.add(nm[3:], d['x4'], 0.0, d['z'], Fx=s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd_, COS(phie))))), Fy=s.fn(nm + '/Fy', M(qA, v(-1), S(M(cl, COS(phie)), M(cd_, SIN(phie))))), Mz=M(qA, v(-st['c'] * F), cm))

    def bodies(s):
        c = s.c
        for b in c['bodies']:
            for side in b.get('sides', (0,)):
                for i, st in enumerate(b['stations']):
                    nm = 'bu/%s%s%d' % (b['name'], 'LR'[side > 0] if side else '', i)
                    x, y, z = st['x'] * F, side * b.get('y', 0.0) * F, st['z'] * F
                    kp = c['k_munk'] * (1.0 if st['x'] < b['xmax'] else c['k_munk_aft'])
                    u, vv, w = s.vel(x, y, z)
                    fp = c['flex_module'].phi_at(c['flex'], y) if c.get('flex') and side else 0.0
                    sd = 'L' if side < 0 else 'R'
                    if fp:
                        w = D(w, M(v(fp), p('flex/qd%s-fps' % sd)))
                    U = s.fn(nm + '/u', u)
                    Vv = s.fn(nm + '/v', vv)
                    up = st.get('upwash', 0.0)
                    Wv = s.fn(nm + '/w', S(w, M(v(up), s.CLw, s.Vt)) if up else w)
                    Vc = s.fn(nm + '/Vc', SQ(S(SQR(Vv), SQR(Wv), v(1e-4))))
                    Fz = s.fn(nm + '/Fz', S(M(v(-kp * st['dSv'] * F * F), s.RHO, U, Wv), M(v(-0.5 * c['cdc'] * st['plan'] * F * F), s.RHO, Wv, Vc)))
                    Fy = s.fn(nm + '/Fy', S(M(v(-kp * st['dSl'] * F * F), s.RHO, U, Vv), M(v(-0.5 * c['cdc'] * st['side'] * F * F), s.RHO, Vv, Vc)))
                    if fp:
                        s.flexQ[sd].append(M(v(-fp), Fz))
                    s.add(nm[3:], x, y, z, Fy=Fy, Fz=Fz)

    def drag(s):
        c = s.c
        D0 = c['drag_area_ft2']
        k = s.fn('bu/d0/k', M(v(-0.5), s.RHO, s.Vt, S(v(D0), M(p('gear/gear-pos-norm'), v(c['gear_ft2'])))), 'the rest of the zero-lift drag, along the wind')
        x, z = c['drag_point']
        s.add('d0', x * F, 0.0, z * F, Fx=s.fn('bu/d0/Fx', M(k, s.U)), Fy=s.fn('bu/d0/Fy', M(k, s.V)), Fz=s.fn('bu/d0/Fz', M(k, s.W)))

    def outputs(s, tot):
        """the airframe's coefficients about a moment reference, as the CFD's: stability axes for lift and drag, thrust left out"""
        m = s.c.get('coef')
        if not m:
            return
        q = s.fn('bu/qS', M(MAX(p('aero/qbar-psf'), v(0.01)), v(m['S'])))
        sa, ca = SIN(p('aero/alpha-rad')), COS(p('aero/alpha-rad'))
        X_, Y_, Z_ = tot['X'], tot['Y'], tot['Z']
        xm, zm = m['ref'][0] / 12.0, m['ref'][2] / 12.0
        s.fn('aero/coef/CL', Q(D(M(X_, sa), M(Z_, ca)), q))
        s.fn('aero/coef/CD', Q(M(v(-1), S(M(X_, ca), M(Z_, sa))), q))
        s.fn('aero/coef/CY', Q(Y_, q))
        s.fn('aero/coef/Cl', Q(D(tot['L'], M(v(zm), Y_)), M(q, v(m['b']))))
        s.fn('aero/coef/Cm', Q(S(tot['M'], M(v(zm), X_), M(v(-xm), Z_)), M(q, v(m['c']))))
        s.fn('aero/coef/Cn', Q(S(tot['N'], M(v(xm), Y_)), M(q, v(m['b']))))
        for c in ('CD', 'CL', 'CY', 'Cl', 'Cm', 'Cn'):
            s.fn('aero/%s/cruise' % c, v(0))

    def flexing(s):
        """each panel's generalized force, and what its acceleration does to the airframe"""
        fx = s.c.get('flex')
        if not fx:
            return
        for sd, sign in (('L', -1), ('R', 1)):
            s.flexQ[sd] += s.c.get('flex_forces', {}).get(sd, [])
            s.fn('bu/flex/Q%s-lbs' % sd, S(*s.flexQ[sd]), 'the air on the panel\'s mode')
            s.keep.add('bu/flex/Q%s-lbs' % sd)
            s.keep.add('bu/flex/F%s-lbs' % sd)
            s.add('flex' + sd, fx['x_c_ft'], sign * fx['y_c_ft'], fx['z_c_ft'], Fz=s.fn('bu/flex/F%s-lbs' % sd, M(v(fx['Pt_slug']), p('flex/qdd%s-fps2' % sd)), 'the panel accelerating up pushes the airframe down'))

    def tyres(s):
        """NASA's tyre deformation drag, its eq. 4 [20220005440]: (k1 V/100 + k4 (V/100)^4) times the wheel load, V in km/h, in place of most of JSBSim's constant rolling friction. kR1, kR4 as fitted there"""
        t = s.c.get('tyre')
        if not t:
            return
        # its fit stopped at 55 KIAS: not carried past that
        V = s.fn('bu/tyre/V', MIN(v(1.02), M(p('velocities/u-fps'), v(1.09728 / 100))), 'speed over the ground, per 100 km/h')
        k = S(M(v(0.102), V), M(v(7.03e-4), V, SQR(V), MAX(V, M(v(-1), V))))
        s.add('tyre', t[0], 0.0, t[1], Fx=s.fn('bu/tyre/drag', M(k, p('forces/fbz-gear-lbs'), p('gear/gear-pos-norm')), 'behind the motion; a belly on the ground is no tyre'))

    def body_fit(s):
        """a moment that goes with the body's incidence, not any section's: what the CFD's pitch break does with power on or off alike"""
        b = s.c.get('body_fit')
        if b:
            k = M(p('aero/qbar-psf'), v(s.c['coef']['S'] * s.c['coef']['c']))
            s.add('fit', 0.0, 0.0, 0.0, My=s.fn('bu/fit/M', M(k, T2('aero/alpha-deg', 'ctl/flap-deg', b['alpha'], b['flaps'], b['Cm']))))
        for key, tag, on in (('ground_fit', 'ground', None), ('ground_hl_fit', 'ground-hl', 'prop/hl/on')):
            g = s.c.get(key)
            if g:
                cf = s.c['coef']
                tb = lambda n, g=g: T3('bu/h-b', 'aero/alpha-deg', 'ctl/flap-deg', g['hb'], g['alpha'], g['flaps'], g[n])
                w = [p(on)] if on else []
                qs = M(p('aero/qbar-psf'), v(cf['S']), *w)
                s.add('g' + tag, cf['ref'][0] / 12.0, 0.0, cf['ref'][2] / 12.0, Fz=s.fn('bu/fit/Z-' + tag, M(v(-1), qs, tb('CL'))), My=s.fn('bu/fit/M-' + tag, M(qs, v(cf['c']), tb('Cm'))))
        h = s.c.get('hl_fit')
        if h:
            k = M(p('aero/qbar-psf'), v(s.c['coef']['S'] * s.c['coef']['c']), p('prop/hl/on'))
            s.add('fit-hl', 0.0, 0.0, 0.0, My=s.fn('bu/fit/M-hl', M(k, T3('prop/hl/J', 'aero/alpha-deg', 'ctl/flap-deg', h['J'], h['alpha'], h['flaps'], h['Cm']))))
        k = s.c.get('k_jet_roll')
        if k:
            # a lost jet's lift goes from nearer the centre than its strips: the root separates beside it [TM 4.3.2]. Nil while the two sides match
            cf = s.c['coef']
            terms = [M(v(math.copysign(1.0, d['y'])), Q(d['a'], S(d['u'], d['a'], v(1)))) for d in s.D if d.get('test')]
            s.add('fit-jet-roll', 0.0, 0.0, 0.0, Mx=s.fn('bu/fit/jet-roll', M(v(k * cf['S'] * cf['b']), p('aero/qbar-psf'), S(*terms))))
        b = s.c.get('beta_fit')
        if b:
            cf = s.c['coef']
            # fitted power off: blown, the sections don't stall where this comes from
            kb = M(p('aero/qbar-psf'), v(cf['S'] * cf['b']), *([D(v(1), p('prop/hl/on'))] if s.c.get('hl_fit') else []))
            tb = lambda n: T3('aero/beta-deg', 'aero/alpha-deg', 'ctl/flap-deg', b['beta'], b['alpha'], b['flaps'], b[n])
            s.add('fit-beta', 0.0, 0.0, 0.0, Mx=s.fn('bu/fit/beta-L', M(kb, tb('Cl'))), Mz=s.fn('bu/fit/beta-N', M(kb, tb('Cn'))))
        u = s.c.get('rud_fit')
        if u:
            cf = s.c['coef']
            tb = lambda n: T3('ctl/rud-deg', 'aero/alpha-deg', 'ctl/flap-deg', u['rud'], u['alpha'], u['flaps'], u[n])
            qs = M(p('aero/qbar-psf'), v(cf['S']))
            s.add('fit-rud', cf['ref'][0] / 12.0, 0.0, cf['ref'][2] / 12.0, Fy=s.fn('bu/fit/rud-Y', M(qs, tb('CY'))), Mz=s.fn('bu/fit/rud-N', M(qs, v(cf['b']), tb('Cn'))))
        cr = s.c.get('cruise_roll')
        if cr:
            cf = s.c['coef']
            kb = M(p('aero/qbar-psf'), v(cf['S'] * cf['b']))
            tb = lambda d: T2('prop/%s/J' % d, 'prop/%s/pitch-deg' % d, cr['J'], cr['pitch'], cr['dCl'])
            own = M(v(cr['k'] * cf['S'] * cf['b']), D(MAX(p('prop/r/thrust-lbs'), v(0)), MAX(p('prop/l/thrust-lbs'), v(0))))
            s.add('fit-cruise', 0.0, 0.0, 0.0, Mx=s.fn('bu/fit/cruise-L', S(M(kb, D(tb('r'), tb('l'))), own)))
        r = s.c.get('rate_fit')
        if r:
            tb = lambda n: T2('aero/alpha-deg', 'ctl/flap-deg', r['alpha'], r['flaps'], r[n])
            cf = s.c['coef']
            # JSBSim's b/2V has no floor, and at rest a frame's stale value meets a live rate: no rate term below 30 ft/s to speak of
            bv, cv = MIN(p('aero/bi2vel'), v(cf['b'] / 60)), MIN(p('aero/ci2vel'), v(cf['c'] / 60))
            ph = M(s.Pp, bv)
            rh = M(s.Rr, bv)
            qh = M(s.Qq, cv)
            kb = M(p('aero/qbar-psf'), v(cf['S'] * cf['b']))
            kc = M(p('aero/qbar-psf'), v(cf['S'] * cf['c']))
            s.add('rates', 0.0, 0.0, 0.0, Mx=s.fn('bu/fit/rate-L', M(kb, S(M(tb('Clp'), ph), M(tb('Clr'), rh)))), My=s.fn('bu/fit/rate-M', M(kc, tb('Cmq'), qh)),
                  Mz=s.fn('bu/fit/rate-N', M(kb, S(M(tb('Cnp'), ph), M(tb('Cnr'), rh)))))

    def build(s):
        s.g.fn('bu/rud-fin-deg', M(v(-1), p('ctl/rud-deg')), 'as a flap on the fin: trailing edge towards +y lifts towards -y')
        s.kin()
        s.discs()
        s.wing()
        s.disc_forces()
        if s.c.get('stab'):
            s.tail()
        if s.c.get('fin'):
            s.fin()
        s.bodies()
        s.drag()
        s.body_fit()
        s.tyres()
        s.flexing()
        tot = {k: s.fn('bu/total/%s' % k, S(*s.parts[k])) for k in 'XYZLMN'}
        s.outputs(tot)
        ex = s.c.get('extra', {})
        axes = {k: s.fn('bu/axis/%s' % k, S(tot[k], *ex[k])) if ex.get(k) else tot[k] for k in 'XYZLMN'}
        ax = '\n'.join('<axis name="%s"><function>%s</function></axis>' % (n, axes[k]) for n, k in (('X', 'X'), ('Y', 'Y'), ('Z', 'Z'), ('ROLL', 'L'), ('PITCH', 'M'), ('YAW', 'N')))
        return '<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (s.g.xml() if hasattr(s.g, 'xml') else '', ax)
