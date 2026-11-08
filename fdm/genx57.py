#!/usr/bin/env python3
"""Generate the X-57's JSBSim model, the table model: NASA's aerodynamic structure on the CFD's own tables.

    python3 genx57.py OUTDIR --mod3|--mod4 [--fg]     writes OUTDIR/x57.xml and OUTDIR/Systems/x57-*.xml; --fg: named by variant
    python3 genx57.py OUTDIR --mod3|--mod4 --built    the built-up model in place of the tables: x57built.py
    python3 genx57.py OUTDIR --p2006t                 the built-up model on the P2006T: genp2006t.py

Mod III has the wing and the wingtip cruise motors, with dummy nacelles on the leading edge. Mod IV adds the twelve
high-lift propellers, whose blades fold away when off, so with them off it is Mod III.

Forces. The CFD's coefficients hold the airframe's forces only: the propellers' thrust is not in them [TM 4.3.1.1], so it
is added at the hubs. CL and CD are in stability axes, CY, Cl, Cm and Cn in body axes, all about the moment reference,
which is the model's AERORP, so JSBSim carries them to the centre of gravity. They go in as X, Y, Z forces and moments.

Propulsion is propulsion.py: fourteen discs, their motors, the hubs and the battery.

All of it is JSBSim XML; no Nasal runs the flight model.
"""
import json, os, re, sys
import numpy as np
from xmlfun import *
import x57data as X
import propulsion as PR
import tyreforce as TF

HERE = os.path.dirname(os.path.abspath(__file__))
T = json.load(open(os.path.join(HERE, 'data', 'aero_tables.json')))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
MRS = GEO['reference']['mrs_in']
SREF, BREF, CREF = GEO['reference']['S_ft2'], GEO['reference']['b_ft'], GEO['reference']['c_ft']
SIX = ('CD', 'CL', 'CY', 'Cl', 'Cm', 'Cn')
EVEN = ('CD', 'CL', 'Cm')
ODD = ('CY', 'Cl', 'Cn')
A = 'aero/alpha-deg'
FLAP = 'ctl/flap-deg'
RPM_NM_TO_FTLB = 0.737562
D_CRUISE = 5.0
D_HL = 1.89
MOTOR_TORQUE_FTLB = X.CRUISE_MOTOR['torque_100pct_nm'] * X.CRUISE_MOTOR['controllers'] * RPM_NM_TO_FTLB

def base_table(c):
    return T2(A, FLAP, T['alpha'], T['flaps'], np.array(T['base'][c]).T.tolist())


def inc(block, c, col):
    """an increment on alpha, a deflection (or sideslip), and flap"""
    return T3(A, col, FLAP, T['nodes'], T[block]['values'], T['flaps'], T[block][c])


def hl_extra(block, c, col, w):
    """a high-lift increment on alpha, deflection and J, blended between the two flap settings by the weight w"""
    h = T['highlift']
    tabs = [T3(A, col, 'prop/hl/J', h['nodes'], h[block]['values'], h['full_J'], h[block][c][k]) for k in (0, 1)]
    return S(M(D(v(1), w), tabs[0]), M(w, tabs[1]))


def controls(hl):
    f = FCS('x57-controls')
    f.declare('ctl/instant', 0)                 # the harness: flaps and gear in place at once
    f.declare('ctl/flap-cmd-deg', 0)
    f.declare('ctl/gear-drag-ft2', X.GEAR_DRAG_AREA_FT2)
    f.declare('ctl/one-aileron', 0)
    f.declare('ctl/one-aileron-deg', 0)
    f.declare('ctl/one-rudder', 0)              # the harness: the rudder where the CFD put it, 28 deg past its travel included
    f.declare('ctl/one-rudder-deg', 0)
    f.declare('bu/test/alphadot', 0)
    f.declare('bu/test/hl-thrust', -999)
    f.declare('bu/test/hl-torque', 0)
    f.declare('/controls/flight/flaps', 0)
    f.channel('surfaces')
    cmd = f.fn('ctl/pitch-cmd', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm')), clip=('-1', '1'))
    f.fn('ctl/stab-deg', S(v(-5), M(v(10), cmd)), desc='all-moving stabilator, -15 to +5 deg, trailing edge down positive')
    f.fn('x57/stab-norm', M(v(1 / 15.0), p('ctl/stab-deg')), desc='for the 3-D model')
    a = f.fn('ctl/roll-cmd', S(p('fcs/aileron-cmd-norm'), p('fcs/roll-trim-cmd-norm')), clip=('-1', '1'))
    up, dn = -X.SURFACES['aileron'][0], X.SURFACES['aileron'][1]
    test = p('ctl/one-aileron')                 # the harness: the right aileron alone, as the CFD moved it
    f.fn('ctl/ail-right-deg', IFTHEN(GT(test, v(0)), p('ctl/one-aileron-deg'), IFTHEN(GT(a, v(0)), M(v(-up), a), M(v(-dn), a))),
         desc='right aileron, trailing edge down positive')
    f.fn('ctl/ail-left-deg', IFTHEN(GT(test, v(0)), v(0), IFTHEN(GT(a, v(0)), M(v(dn), a), M(v(up), a))),
         desc='left aileron, trailing edge down positive')
    f.fn('ctl/yaw-cmd', S(p('fcs/rudder-cmd-norm'), p('fcs/yaw-trim-cmd-norm')), clip=('-1', '1'))
    f.fn('ctl/rud-deg', IFTHEN(GT(p('ctl/one-rudder'), v(0)), p('ctl/one-rudder-deg'), M(v(X.SURFACES['rudder'][1]), p('ctl/yaw-cmd'))), desc='trailing edge left positive: left pedal')
    f.fn('x57/rudder-norm', M(v(1 / 15.0), p('ctl/rud-deg')), desc='for the 3-D model')
    TF.nose_steering(f, X.GEAR['main_x_in'] - X.GEAR['wheelbase_in'])
    f.fn('x57/pos-rudder', p('fcs/rudder-cmd-norm'), out='fcs/rudder-pos-norm')
    f.fn('x57/pos-elevator', p('ctl/pitch-cmd'), out='fcs/elevator-pos-norm')
    f.fn('x57/pos-left-aileron', M(v(1 / 25.0), p('ctl/ail-left-deg')), out='fcs/left-aileron-pos-norm')
    f.fn('x57/pos-right-aileron', M(v(1 / 25.0), p('ctl/ail-right-deg')), out='fcs/right-aileron-pos-norm')
    f.fn('ctl/rud-neg', M(v(-1), ABS(p('ctl/rud-deg'))), desc='the rudder tables run on negative deflection')
    want = S(p('ctl/flap-cmd-deg'), M(v(30), p('/controls/flight/flaps')))
    f.fn('ctl/flap-want', IFTHEN(GT(p('elec/flap-power'), v(0.5)), want, p('ctl/flap-moving')), desc='the flap motor is on the Essential Bus')
    # the flap motor, 3 deg/s: its position kept in a property, not in an actuator's own state, so that a runway start can put it at takeoff (nasal/cockpit.nas)
    dt = p('simulation/channel-dt')
    f.fn('ctl/flap-moving', MIN(v(30), MAX(v(0), S(p('ctl/flap-moving'), MIN(M(v(3.0), dt), MAX(M(v(-3.0), dt), D(p('ctl/flap-want'), p('ctl/flap-moving'))))))))
    f.fn(FLAP, IFTHEN(GT(p('ctl/instant'), v(0)), p('ctl/flap-want'), p('ctl/flap-moving')))
    f.fn('x57/pos-flap', M(v(1 / 30.0), p(FLAP)), out='fcs/flap-pos-norm')
    f.fn('ctl/hl-flap-w', tag('min', v(1), tag('max', v(0), M(v(0.05), D(p(FLAP), v(10))))), desc='flaps 10 to 30, between the two high-lift tables')
    # On the ground the gear goes no further up: a choice of ours. The manual shows only a knob to pull first, and an instructor's deck a squat switch
    f.fn('gear/gear-want', IFTHEN(GT(p('elec/gear-power'), v(0.5)), MAX(p('gear/gear-cmd-norm'), M(p('gear/wow'), p('gear/gear-moving'))), p('gear/gear-moving')), desc='and so is the gear')
    f.raw('<kinematic name="gear/gear-moving"><input>gear/gear-want</input><traverse>'
          '<setting><position>0</position><time>0</time></setting><setting><position>1</position><time>6</time></setting>'
          '</traverse></kinematic>')
    f.fn('gear/gear-pos-norm', IFTHEN(GT(p('ctl/instant'), v(0)), p('gear/gear-cmd-norm'), p('gear/gear-moving')), out='gear/unit[0]/pos-norm')
    # the P2006T's brakes, fitted to its manual's landing roll: Mods II-IV kept its gear [SIM]. Each main strut as gear_xml sets it
    from genp2006t import brakes
    brakes(f, X.MASS['weight_lb'] * X.GEAR['main_load_share'] / 2 / 0.25)
    # Each wheel's spin, as the speed its rim turns at. On the ground friction spins it up, its sliding grip times the strut's force (spring and damping,
    # as the gear's own), until it rolls at the ground speed: a wheel that only brushes the runway between two of FlightGear's frames, unheard,
    # keeps most of its slip for the touchdown that follows. In the air it spins down, its brake's pads and its bearings dragging, and a main
    # wheel's pedal stops it at once. Then each wheel's sink rate and slip, the ground speed less its rim's, as its weight
    # comes on, held till it lifts: what its touchdown sounds read (x57sound.py, mktouchdown.py). A wheel touching down still turning loses
    # I (slip / r)^2 / 2 spinning up, so a hop of a second is silent and a touchdown after a flight squeals in full. The wheel's own speed downwards:
    # the airplane's, with its roll and pitch about the centre of gravity, so a nose that comes down by pitching counts
    f.channel('touchdown')
    G = X.GEAR
    brake_nm = 4.4482216 * 6.9 * 0.0254                      # N m a lbf of ctl/brake-lbf, the force at the 6.00-6's static loaded radius (p2006t.py)
    for n, x, y in ((0, G['main_x_in'] - G['wheelbase_in'], 0.0), (1, G['main_x_in'], -G['track_in'] / 2), (2, G['main_x_in'], G['track_in'] / 2)):
        g = 'gear/unit[%d]/' % n
        w = X.WHEELS['5.00-5' if n == 0 else '6.00-6']
        dx = Q(D(p('inertia/cg-x-in'), v(x)), v(12.0))         # ahead of the CG, ft
        dy = Q(D(v(y), p('inertia/cg-y-in')), v(12.0))         # right of it
        f.fn(g + 'sink-now-fps', S(p('velocities/v-down-fps'), M(p('velocities/p-rad_sec'), dy), M(v(-1), p('velocities/q-rad_sec'), dx)),
             desc='the wheel\'s own speed downwards')
        down = GT(p(g + 'WOW'), v(0.5))
        k = w['radius_m'] / w['inertia_kgm2'] / 0.514444           # kt/s of rim speed a N m of torque takes away
        torque = v(w['drag_nm']) if n == 0 else S(v(w['drag_nm']), M(MAX(p('/controls/gear/brake-' + ('left' if n == 1 else 'right')),
                                                                           p('/controls/gear/brake-parking')), p('ctl/brake-lbf'), v(brake_nm)))
        kt = M(p('velocities/vg-fps'), v(0.592484))
        sk, sc = strut(X.MASS['weight_lb'] * (1 - G['main_load_share'] if n == 0 else G['main_load_share'] / 2))
        load = MAX(v(0), S(M(v(sk), p(g + 'compression-ft')), M(v(sc), p(g + 'compression-velocity-fps'))))       # lbf
        up = 0.5 * 4.4482216 * w['radius_m'] ** 2 / w['inertia_kgm2'] / 0.514444     # kt/s of rim speed a lbf of load gives, sliding at 0.5
        f.fn(g + 'slip-now-kt', MAX(v(0), D(kt, p(g + 'spin-kt'))), desc='the ground speed less the rim\'s')
        f.fn(g + 'spin-kt', IFTHEN(down, MIN(kt, S(p(g + 'spin-kt'), M(v(up), load, p('simulation/channel-dt')))),
                                   MAX(v(0), D(p(g + 'spin-kt'), M(v(k), torque, p('simulation/channel-dt'))))),
             desc='the rim\'s speed: spun up by the runway, rolling, spinning down in the air')
        f.fn(g + 'touchdown-sink-fps', IFTHEN(down, p(g + 'touchdown-sink-fps'), p(g + 'sink-now-fps')), desc='as the weight came on, held')
        f.fn(g + 'touchdown-slip-kt', IFTHEN(down, p(g + 'touchdown-slip-kt'), p(g + 'slip-now-kt')), desc='likewise')
    return f


def aero(hl):
    g = Graph()
    a = p(A)
    g.fn('x57/abs-beta', ABS(p('aero/beta-deg')))
    sb = g.fn('x57/sign-beta', IFTHEN(LT(p('aero/beta-deg'), v(0)), v(-1), v(1)))
    sr = g.fn('x57/sign-rud', IFTHEN(GT(p('ctl/rud-deg'), v(0)), v(-1), v(1)), desc='the tables are for the rudder trailing edge right')
    ph = g.fn('x57/p-hat', M(p('velocities/p-aero-rad_sec'), p('aero/bi2vel')))
    qh = g.fn('x57/q-hat', M(p('velocities/q-aero-rad_sec'), p('aero/ci2vel')))
    rh = g.fn('x57/r-hat', M(p('velocities/r-aero-rad_sec'), p('aero/bi2vel')))
    rt = T['rates']

    def rate(c):
        return T2(A, FLAP, rt['alpha'], T['flaps'], np.array(rt[c]).T.tolist())

    cr = T['cruise']
    if hl:
        h = T['highlift']
        on = p('prop/hl/on')
        w = p('ctl/hl-flap-w')

        def hl_main(c):
            return T3('prop/hl/J', A, FLAP, h['J'], h['alpha'], h['flaps'], h['d' + c])

        def hl_rate(c):
            r = h['rates']
            return T3('prop/hl/J', A, FLAP, r['J'], r['alpha'], r['flaps'], r[c])

    for c in SIX:
        even = c in EVEN
        terms = []
        if c in EVEN:
            terms.append(g.fn('aero/%s/base' % c, base_table(c)))
        be = inc('beta', c, 'x57/abs-beta')
        terms.append(g.fn('aero/%s/beta' % c, be if even else M(sb, be)))
        if c in EVEN:
            terms.append(g.fn('aero/%s/stab' % c, inc('stab', c, 'ctl/stab-deg')))
        ar, al = inc('ail', c, 'ctl/ail-right-deg'), inc('ail', c, 'ctl/ail-left-deg')
        terms.append(g.fn('aero/%s/ail' % c, S(ar, al) if even else D(ar, al)))
        ru = inc('rud', c, 'ctl/rud-neg')
        terms.append(g.fn('aero/%s/rud' % c, ru if even else M(sr, ru)))
        # cruise motors: NASA's increments on J and blade pitch, one motor's, on the right
        dr = T2('prop/r/J', 'prop/r/pitch-deg', cr['J'], cr['pitch'], cr['d' + c])
        dl = T2('prop/l/J', 'prop/l/pitch-deg', cr['J'], cr['pitch'], cr['d' + c])
        terms.append(g.fn('aero/%s/cruise' % c, S(dr, dl) if even else D(dr, dl)))
        if hl:
            if c in EVEN:
                terms.append(g.fn('aero/%s/hl' % c, M(on, hl_main(c))))
                terms.append(g.fn('aero/%s/hl-stab' % c, M(on, hl_extra('stab', c, 'ctl/stab-deg', w))))
            b2 = hl_extra('beta', c, 'x57/abs-beta', w)
            terms.append(g.fn('aero/%s/hl-beta' % c, M(on, b2 if even else M(sb, b2))))
            hr, hlft = hl_extra('ail', c, 'ctl/ail-right-deg', w), hl_extra('ail', c, 'ctl/ail-left-deg', w)
            terms.append(g.fn('aero/%s/hl-ail' % c, M(on, S(hr, hlft) if even else D(hr, hlft))))
            hu = hl_extra('rud', c, 'ctl/rud-neg', w)
            terms.append(g.fn('aero/%s/hl-rud' % c, M(on, hu if even else M(sr, hu))))
        if c == 'Cm':
            k = rate('Cmq') if not hl else S(rate('Cmq'), M(on, hl_rate('Cmq')))
            terms.append(g.fn('aero/Cm/rate', M(k, qh)))
        if c == 'Cl':
            kp = rate('Clp') if not hl else S(rate('Clp'), M(on, hl_rate('Clp')))
            kr = rate('Clr') if not hl else S(rate('Clr'), M(on, hl_rate('Clr')))
            terms.append(g.fn('aero/Cl/rate', S(M(kp, ph), M(kr, rh))))
        if c == 'Cn':
            kp = rate('Cnp') if not hl else S(rate('Cnp'), M(on, hl_rate('Cnp')))
            kr = rate('Cnr') if not hl else S(rate('Cnr'), M(on, hl_rate('Cnr')))
            terms.append(g.fn('aero/Cn/rate', S(M(kp, ph), M(kr, rh))))
        if c == 'CD':
            terms.append(g.fn('aero/CD/gear', M(Q(p('ctl/gear-drag-ft2'), v(SREF)), p('gear/gear-pos-norm'))))
        g.fn('aero/coef/%s' % c, S(*terms))

    q = p('aero/qbar-psf')
    ca = g.fn('x57/cos-a', COS(p('aero/alpha-rad')))
    sa = g.fn('x57/sin-a', SIN(p('aero/alpha-rad')))
    CD, CL, CY = p('aero/coef/CD'), p('aero/coef/CL'), p('aero/coef/CY')
    # the discs' forces, and a side's high-lift increment out of balance when its motors differ
    X_, Y_, Z_, L_, M_, N_ = PR.forces(hl, MRS)
    H = g.fn('prop/spin-momentum', PR.gyroscopic(hl))
    M_.append(M(v(-1), p('velocities/r-aero-rad_sec'), H))
    N_.append(M(p('velocities/q-aero-rad_sec'), H))
    g.fn('prop/thrust-total-lbs', S(*X_))
    fx = g.fn('aero/FX-lbs', S(M(q, v(SREF), D(M(CL, sa), M(CD, ca))), *X_))
    fy = g.fn('aero/FY-lbs', S(M(q, v(SREF), CY), *Y_))
    fz = g.fn('aero/FZ-lbs', S(M(v(-1), q, v(SREF), S(M(CD, sa), M(CL, ca))), *Z_))
    if hl:
        y = sum(abs(u[1]) for u in PR.hl_stations()) / 12 / 12.0
        off = D(p('prop/hl/on-r'), p('prop/hl/on-l'))
        L_.append(M(v(-0.5 * y), q, v(SREF), T3('prop/hl/J', A, FLAP, h['J'], h['alpha'], h['flaps'], h['dCL']), off))
        N_.append(M(v(0.5 * y), q, v(SREF), T3('prop/hl/J', A, FLAP, h['J'], h['alpha'], h['flaps'], h['dCD']), off))
    ml = g.fn('aero/ML-ftlb', S(M(q, v(SREF * BREF), p('aero/coef/Cl')), *L_))
    mm = g.fn('aero/MM-ftlb', S(M(q, v(SREF * CREF), p('aero/coef/Cm')), *M_))
    mn = g.fn('aero/MN-ftlb', S(M(q, v(SREF * BREF), p('aero/coef/Cn')), *N_))
    axes = '\n'.join('<axis name="%s"><function>%s</function></axis>' % (n, x) for n, x in
                     (('X', fx), ('Y', fy), ('Z', fz), ('ROLL', ml), ('PITCH', mm), ('YAW', mn)))
    return '<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (g.xml(), axes)


AERO = {}


def built_aero(hl):
    """The parts go in a system, not as <function>s under <aerodynamics>. A function read before its first frame works out all it reads, and they all it reads, and FlightGear
    reads every property at start: with a graph this deep that never ends. A system's outputs are stored values"""
    import builtup
    import x57built
    c = x57built.config(hl)
    X_, Y_, Z_, L_, M_, N_ = PR.forces(hl, (0.0, 0.0, 0.0), flex=True)
    a = builtup.Aero(c)
    AERO['system'] = FCS('built-up aerodynamics')
    AERO['system'].channel('aerodynamics')
    a.g = ChannelGraph(AERO['system'])
    H = 'prop/spin-momentum'
    a.g.fn(H, PR.gyroscopic(hl))
    a.g.fn('prop/thrust-total-lbs', S(*X_))
    c['extra'] = dict(X=X_, Y=Y_, Z=Z_, L=L_, M=M_ + [M(v(-1), p('velocities/r-aero-rad_sec'), p(H))], N=N_ + [M(p('velocities/q-aero-rad_sec'), p(H))])
    return a.build()


def strut(load):
    """spring (lbf/ft) and damping (lbf s/ft) of a strut carrying this load (lbf): it sits about 3 in down on its strut and tyre, damped at 0.6 of
    critical on its load, twice; rounded as the gear's XML writes them"""
    k = load / 0.25
    return float(round(k)), float(round(0.6 * k ** 0.5 * (load / 32.174) ** 0.5 * 2))


def gear_xml():
    G = X.GEAR
    z = G['ground_z_in']
    W = X.MASS['weight_lb']
    xm = G['main_x_in']
    xn = xm - G['wheelbase_in']
    y = G['track_in'] / 2

    def unit(name, x, yy, load, steer, brake, zz):
        return ('''<contact type="BOGEY" name="%s">
  <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
  <static_friction>0.72</static_friction><dynamic_friction>0.5</dynamic_friction><rolling_friction>0.005</rolling_friction>
  <spring_coeff unit="LBS/FT">%.0f</spring_coeff><damping_coeff unit="LBS/FT/SEC">%.0f</damping_coeff>
  <max_steer unit="DEG">%g</max_steer><brake_group>%s</brake_group><retractable>1</retractable>
  %s
</contact>''' % ((name, x, yy, zz) + strut(load) + (steer, brake, TF.table_xml('5.00-5' if steer else '6.00-6', load))))

    main = W * G['main_load_share'] / 2
    nose = W * (1 - G['main_load_share'])
    zc = z - G['sag_in']
    u = [unit('NOSE', xn, 0, nose, TF.MAX_STEER_DEG, 'NONE', zc), unit('LEFT_MAIN', xm, -y, main, 0, 'LEFT', zc), unit('RIGHT_MAIN', xm, y, main, 0, 'RIGHT', zc)]
    tip = GEO['wing']['span_in'] / 2
    for n, x, yy, zz in (('LEFT_TIP', 150, -tip, 86), ('RIGHT_TIP', 150, tip, 86), ('TAIL', 345, 0, 38), ('BELLY', 150, 0, 38),
                         ('NOSE_TIP', 30, 0, 60), ('LEFT_PROP', 138, -tip, 58), ('RIGHT_PROP', 138, tip, 58)):
        u.append('''<contact type="STRUCTURE" name="%s">
  <location unit="IN"><x>%g</x><y>%g</y><z>%g</z></location>
  <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction>
  <spring_coeff unit="LBS/FT">20000</spring_coeff><damping_coeff unit="LBS/FT/SEC">5000</damping_coeff>
</contact>''' % (n, x, yy, zz))
    return '<ground_reactions>\n%s\n</ground_reactions>\n' % '\n'.join(u)


def cg_in():
    """the centre of gravity, in: at its published fraction of the wing's mean chord, its published offset and depth"""
    m = X.MASS
    w = GEO['wing']
    p0 = w['panels'][0]
    lam = p0['c1'] / p0['c0']
    ymac = p0['y1'] / 3 * (1 + 2 * lam) / (1 + lam)
    cm = p0['c0'] + (p0['c1'] - p0['c0']) * ymac / p0['y1']
    xle = w['origin_in'][0] + ymac * np.tan(np.radians(p0['sweep'])) + p0['loc'] * (p0['c0'] - cm)
    return (xle + m['cg_mac'] * w['mac_in'], m['cg_y_in'], GEO['reference']['wing_le_in'][2] - m['cg_below_wing_le_in'])


def fdm(hl, built=False):
    m = X.MASS
    cg = cg_in()
    k = X.SLUG_FT2
    gross = m['flight_weight_lb']['mod4' if hl else 'mod3'] if '--fg' in sys.argv else m['weight_lb']     # the harness flies NASA's simulator's
    return '''<?xml version="1.0"?>
<!-- X-57 %s: generated by fdm/genx57.py from NASA's CFD results (NTRS 20250004430) -->
<fdm_config name="X-57 %s" version="2.0" release="PRODUCTION">
<fileheader>
  <author>generated</author>
  <description>X-57 Maxwell %s: %s model on NASA's CFD</description>
</fileheader>
<metrics>
  <wingarea unit="FT2">%.3f</wingarea>
  <wingspan unit="FT">%.3f</wingspan>
  <chord unit="FT">%.3f</chord>
  <location name="AERORP" unit="IN"><x>%.3f</x><y>%.3f</y><z>%.3f</z></location>
  <location name="EYEPOINT" unit="IN"><x>105.9</x><y>-10.7</y><z>82.1</z></location>
  <location name="VRP" unit="IN"><x>0</x><y>0</y><z>0</z></location>
</metrics>
<mass_balance>
  <ixx unit="SLUG*FT2">%.1f</ixx><iyy unit="SLUG*FT2">%.1f</iyy><izz unit="SLUG*FT2">%.1f</izz>
  <ixy unit="SLUG*FT2">%.2f</ixy><ixz unit="SLUG*FT2">%.2f</ixz><iyz unit="SLUG*FT2">%.2f</iyz>
  <emptywt unit="LBS">%.1f</emptywt>
  <location name="CG" unit="IN"><x>%.3f</x><y>%.3f</y><z>%.3f</z></location>
  <pointmass name="above the 3,000 lb design weight">
    <weight unit="LBS">%.1f</weight>
    <location unit="IN"><x>%.3f</x><y>%.3f</y><z>%.3f</z></location>
  </pointmass>
</mass_balance>
%s
<system file="x57-controls"/>
<system file="x57-propulsion"/>
%s%s
</fdm_config>
''' % ('Mod IV' if hl else 'Mod III', 'Mod IV' if hl else 'Mod III', 'Mod IV' if hl else 'Mod III', 'built-up' if '--built' in sys.argv else 'table', SREF, BREF, CREF, MRS[0], MRS[1], MRS[2],
       m['Ixx_lbin2'] / k, m['Iyy_lbin2'] / k, m['Izz_lbin2'] / k, m['Ixy_lbin2'] / k, m['Ixz_lbin2'] / k, m['Iyz_lbin2'] / k,
       m['design_weight_lb'], cg[0], cg[1], cg[2], gross - m['design_weight_lb'], cg[0], cg[1], cg[2], gear_xml(), '<system file="x57-flex"/>\n<system file="x57-aero"/>\n' if built else '', built_aero(hl) if built else aero(hl))


PARTS = ('controls', 'propulsion', 'flex', 'aero')


def fg_reads():
    """what FlightGear's side of the aircraft reads from the flight model: whole names, and the starts of names it builds"""
    import glob
    here = os.path.dirname(os.path.abspath(__file__))
    pk = os.path.join(here, '..', 'X-57')
    files = glob.glob(os.path.join(here, 'nasal', '*.nas')) + [os.path.join(here, n) for n in ('mkfg.py', 'mkmodel.py', 'x57panel.py', 'x57sound.py')]
    for pat in ('*.xml', 'Sounds/*.xml', 'Models/*.xml', 'Models/**/*.xml', 'gui/**/*.xml', 'Systems/x57-rules.xml'):
        files += glob.glob(os.path.join(pk, pat), recursive=True)
    exact, starts = set(), {'x57/'}
    for fn in files:
        for w in re.findall(r'[A-Za-z0-9_/\-\[\]\.%]+', open(fn, errors='replace').read()):
            w = re.sub(r'^/?fdm/jsbsim/', '', w)
            if '%' in w:
                w = w[:w.index('%')]
                if '/' in w and len(w) > 4:
                    starts.add(w)
            elif w.endswith(('/', '-', '[')):
                if '/' in w and len(w) > 4:
                    starts.add(w)
            else:
                exact.add(w)
    return exact, tuple(sorted(starts))


def main():
    if len(sys.argv) >= 3 and sys.argv[2] == '--p2006t':
        import genp2006t
        genp2006t.write(sys.argv[1])
        return
    if len(sys.argv) < 3 or sys.argv[2] not in ('--mod3', '--mod4'):
        sys.exit(__doc__)
    out, hl = sys.argv[1], sys.argv[2] == '--mod4'
    # --fg: the two aircraft side by side in one folder, so their files carry the variant's name
    name = ('x57-mod4' if hl else 'x57-mod3') if '--fg' in sys.argv else 'x57'
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    built = '--built' in sys.argv
    text = fdm(hl, built).replace('file="x57-controls"', 'file="%s-controls"' % name).replace('file="x57-propulsion"', 'file="%s-propulsion"' % name).replace('file="x57-flex"', 'file="%s-flex"' % name).replace('file="x57-aero"', 'file="%s-aero"' % name)
    systems = (controls(hl), PR.build(hl))
    final = {}
    if built:
        import x57flex
        systems += (x57flex.system(hl), AERO['system'])
    if built:           # its moments are about the structural origin
        text = re.sub(r'(<location name="AERORP" unit="IN">).*?(</location>)', r'\1<x>0</x><y>0</y><z>0</z>\2', text)
        if '--fg' in sys.argv and not os.environ.get('X57_NO_OPT'):      # FlightGear locks for each property read. The values stay the same (readcheck.py)
            final = dict(zip(PARTS, (s.xml() for s in systems)))
            names = set(re.findall(r'name="([^"]+)"', ''.join(final.values())))
            exact, starts = fg_reads()
            keep = {n for n in names if n in exact or n.startswith(starts)} | {'bu/alphadot', 'bu/CLw', 'bu/flex/QL-lbs', 'bu/flex/QR-lbs', 'bu/flex/FL-lbs', 'bu/flex/FR-lbs', 'prop/hl/live'}
            d = drop_unread(final, extra=[text], keep=keep)
            n = inline_single_use(final, extra=[text], keep=keep)
            print('  %d functions dropped, %d written into their readers' % (d, n))
    with open(os.path.join(out, name + '.xml'), 'w') as f:
        f.write(text)
    for sysf, part in zip(systems, PARTS):
        with open(os.path.join(out, 'Systems', '%s-%s.xml' % (name, part)), 'w') as f:
            f.write('<?xml version="1.0"?>\n' + (final[part] if part in final else sysf.xml()))
    print('wrote', os.path.relpath(out, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')), name)


if __name__ == '__main__':
    main()
