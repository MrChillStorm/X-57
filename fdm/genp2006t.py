#!/usr/bin/env python3
"""The P2006T validation build: the built-up model on the one airplane of the family that has flown.

    python3 genp2006t.py OUTDIR [--tunnel] [--no-tail] [--no-nacelles] [--kg N] [--cg F]       writes OUTDIR/p2006t.xml and OUTDIR/Systems/p2006t-controls.xml

--tunnel is the 1:6.5 model of ICAS 2008: Reynolds number 0.6 million, trip strips, no winglets, no propellers; its two flags take parts off as the tests did.
"""
import math, os, sys
import numpy as np
from xmlfun import *
import builtup as BU
import liftline as LL
import p2006t as P

F = 1 / 0.3048
IN = 0.0254


def brakes(f, k_main, mu_static=0.72, mu_rolling=0.005):
    """The toe brakes as discs: the pedal sets a force at each main tyre, P.BRAKE_LBF at full pedal (ctl/brake-lbf), which JSBSim takes as a friction
    coefficient on the wheel's load. So the pedal's force is shared out over the strut's load, k_main times its compression, and the surface's grip
    factor, and the tyre's grip caps it. The pedal is FlightGear's own, the parking brake as much as a full pedal: FlightGear also writes the
    command it reads, so this reads the controls, not its own output. The nose wheel has no brake"""
    f.declare('ctl/brake-lbf', P.BRAKE_LBF)
    for n in ('brake-left', 'brake-right', 'brake-parking'):
        f.declare('/controls/gear/' + n, 0)
    f.channel('brakes')
    for side, i in (('left', 1), ('right', 2)):
        g = 'gear/unit[%d]/' % i
        pedal = MAX(p('/controls/gear/brake-' + side), p('/controls/gear/brake-parking'))
        grip = MAX(D(M(v(mu_static), p(g + 'static-friction-factor')), M(v(mu_rolling), p(g + 'rolling_friction-factor'))), v(0.01))
        load = MAX(M(v(k_main), p(g + 'compression-ft')), v(10.0))
        f.fn('ctl/brake-%s' % side, Q(M(pedal, p('ctl/brake-lbf')), M(load, grip)), clip=('0', '1'), out='fcs/%s-brake-cmd-norm' % side,
             desc='the %s brake: its force over what full grip would give' % side)


def main_spring(kg, cg_mac):
    """lbf/ft of each main strut, set for the load it carries, as gear_xml sets it"""
    x = P.WING['le_x'] + cg_mac * P.REF['c_ft'] * 12
    xm, xn = 172.0, 172.0 - 2.9 / IN
    return kg * 2.20462 * (1 - (xm - x) / (xm - xn)) / 2 / 0.25


def controls(k_main=None):
    f = FCS('p2006t-controls')
    for n, val in (('ctl/instant', 1), ('ctl/flap-cmd-deg', 0), ('/controls/flight/flaps', 0), ('/controls/engines/engine[0]/throttle', 0), ('/controls/engines/engine[1]/throttle', 0), ('bu/test/alphadot', 0), ('bu/test/frozen', 1), ('prop/l/run', 1), ('prop/r/run', 1), ('prop/l/rpm', P.PROP['rpm']), ('prop/r/rpm', P.PROP['rpm'])):
        f.declare(n, val)
    f.channel('surfaces')
    cmd = f.fn('ctl/pitch-cmd', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm')), clip=('-1', '1'))
    f.fn('ctl/stab-deg', S(v(-5), M(v(10), cmd)), desc='all-moving stabilator, trailing edge down positive')
    a = p('fcs/aileron-cmd-norm')
    up, dn = -P.SURFACES['aileron'][0], P.SURFACES['aileron'][1]
    f.fn('ctl/ail-right-deg', IFTHEN(GT(a, v(0)), M(v(-up), a), M(v(-dn), a)), desc='trailing edge down positive')
    f.fn('ctl/ail-left-deg', IFTHEN(GT(a, v(0)), M(v(dn), a), M(v(up), a)))
    f.fn('ctl/rud-deg', M(v(P.SURFACES['rudder']), p('fcs/rudder-cmd-norm')), desc='trailing edge left positive: left pedal')
    f.fn('ctl/flap-want', S(p('ctl/flap-cmd-deg'), M(v(40), p('/controls/flight/flaps'))))
    f.actuator('ctl/flap-moving', 'ctl/flap-want', rate=4.0, clip=('0', '40'))
    f.fn('ctl/flap-deg', IFTHEN(GT(p('ctl/instant'), v(0)), p('ctl/flap-want'), p('ctl/flap-moving')))
    f.raw('<kinematic name="gear/gear-moving"><input>gear/gear-cmd-norm</input><traverse><setting><position>0</position><time>0</time></setting>'
          '<setting><position>1</position><time>6</time></setting></traverse></kinematic>')
    f.fn('gear/gear-pos-norm', IFTHEN(GT(p('ctl/instant'), v(0)), p('gear/gear-cmd-norm'), p('gear/gear-moving')), out='gear/unit[0]/pos-norm')
    f.channel('engines')
    e = P.ENGINE
    lapse = T1('position/h-sl-ft', P.POWER_LAPSE['alt_ft'], P.POWER_LAPSE['frac'])
    avail = f.fn('prop/avail-kw', M(v(e['takeoff_kw']), lapse), desc='at full throttle, by altitude')
    for i, n in enumerate('lr'):
        lv = p('/controls/engines/engine[%d]/throttle' % i)
        f.fn('prop/%s/power-kw' % n, M(p('prop/%s/run' % n), S(v(e['idle_kw']), M(lv, D(avail, v(e['idle_kw']))))))
        f.fn('prop/%s/power-hp' % n, M(v(1 / 0.7457), p('prop/%s/power-kw' % n)))
    if k_main:
        brakes(f, k_main)
    return f


def config(tunnel=False, tail=True, nacelles=True):
    rows = P.strips(winglets=not tunnel, span=None)
    fit = dict(P.FIT)
    a_sec = 5.6 if tunnel else 6.2
    area = np.array([r['area'] for r in rows])
    S = P.REF['S_ft2'] / F ** 2
    alpha = np.ones(len(rows))
    # the coarse strips against a fine line, on one wing: the same lift slope
    half = 400
    fine = []
    e = np.linspace(-P.WING['y_tip'], P.WING['y_tip'], half + 1)
    for a, b in zip(e, e[1:]):
        c = P.chord(abs(0.5 * (a + b)))
        fine.append(dict(ya=a, yb=b, za=0.0, zb=0.0, c=c, area=c * (b - a)))
    flat = [dict(r, za=0.0, zb=0.0) for r in rows if r['kind'] == 'wing']
    k_ind = 1.0
    for _ in range(6):
        cf = LL.solve(fine, np.ones(half), a_sec)[2]
        cc = LL.solve(flat, np.ones(len(flat)), a_sec, k_ind)[2]
        k_ind *= 1 + 2.0 * (cc - cf) / cf
    # nacelles: their measured lift loss, on the strips beside them
    if nacelles:
        beside = [r['i'] for r in rows if 0.75 <= abs(r['ym']) <= 2.45]
        sel = np.array([1.0 if r['i'] in beside else 0.0 for r in rows])
        dCL = LL.solve(rows, np.radians(sel), a_sec, k_ind)[2] / S
        dinc = P.NACELLE['dcl_wing'] / dCL
        for r in rows:
            if r['i'] in beside:
                r['dinc'] = dinc
    # the wing's load, for the flow it sends elsewhere
    cl, ai, CL, CDi = LL.solve(rows, np.ones(len(rows)), a_sec, k_ind)
    c = np.array([r['c'] for r in rows])
    ysplit = P.WING['y_break']
    inner = np.array([abs(r['ym']) <= ysplit for r in rows])
    gam = 0.5 * c * cl
    CLi, CLo = (cl * area)[inner].sum() / S, (cl * area)[~inner].sum() / S
    gi, go = gam * inner / CLi, gam * ~inner / CLo
    CLtot = CL / S
    gt = gam / CLtot
    zw = P.WING['z'] * IN
    x_te = P.WING['le_x'] * IN + P.WING['c_root']
    cfg = dict(rows=rows, S_ft2=P.REF['S_ft2'], tables=P.tables(tunnel), a_sec=a_sec, k_ind=k_ind, incidence=fit['incidence'], k_cd=1.0, y_split=ysplit,
               contraction=0.95, k_jet=0.85, k_tail_jet=0.5, k_wake=1.0, wake_ke=1.6 / (math.pi * 8.8), wake_te=(x_te, zw - 0.03, P.WING['c_root']),
               k_munk=0.89, k_munk_aft=0.5, cdc=1.2, eps_lag=0.35)
    fus = P.fuselage()
    x_w = (P.WING['le_x'] + 0.25 * 56.7) * IN
    a_eq = math.sqrt(fus['half_h'](P.WING['le_x'] + 28) * fus['half_w'](P.WING['le_x'] + 28))
    cfg['fuselage_at_wing'] = dict(a=a_eq, z=fus['zc'](P.WING['le_x'] + 28))

    def wash(st, y=0.0):
        """the wing's upflow at a body's station per unit lift, none inside the wing's chord"""
        if y and P.WING['le_x'] * IN < st['x'] < x_te:
            return 0.0                                     # a nacelle runs through the wing; the fuselage is under it
        return float(LL.field(rows, gt, st['x'], y, st['z']))
    for st in fus['stations']:
        st['upwash'] = wash(st)
    cfg['bodies'] = [dict(name='f', stations=fus['stations'], xmax=fus['xmax'])]
    if nacelles:
        na = P.nacelle()
        for st in na['stations']:
            st['upwash'] = wash(st, P.NACELLE['y'])
        cfg['bodies'].append(dict(name='n', stations=na['stations'], xmax=na['xmax'], y=P.NACELLE['y'], sides=(-1, 1)))
    cfg['ground'] = BU.ground_tables(rows, gt, P.stabilator(), zw, P.REF['b_ft'] / F)
    # discs
    cfg['discs'] = []
    cfg['disc_upwash'] = {}
    if not tunnel:
        t = P.propeller_table()
        hx, hy, hz = (q * IN for q in P.PROP['hub'])
        for name, sgn in (('l', -1), ('r', 1)):
            cfg['discs'].append(dict(name=name, hub=(hx, sgn * hy, hz), D=P.PROP['D'], spin=P.PROP['spin'], table=t, feather_ft2=0.8, install=fit['install']))
            cfg['disc_upwash'][name] = float(LL.field(rows, gt, hx, sgn * hy, hz))
    # tail
    if tail:
        st = P.stabilator()
        hs = np.array([-1.5, -1.1, -0.8, -0.5, -0.25, 0.0, 0.25, 0.5, 0.8, 1.1, 1.5, 2.0])
        cfg['eps_h'] = hs.tolist()
        Ei, Eo = [], []
        for h in st['strips']:
            E = [[], []]
            for hh in hs:
                dz = h['z'] - zw - hh
                for k, g in enumerate((gi, go)):
                    E[k].append(np.mean([-LL.field(rows, g, h['x4'], yy, h['z'], dz) for yy in (h['y'], -h['y'])]))
            Ei.append(E[0])
            Eo.append(E[1])
        # Helmbold's lifting surface: a line overstates a surface of aspect ratio 4
        cfg.update(stab=st, eps_in=Ei, eps_out=Eo, e_tail=0.74, a_tail=a_sec, eta_tail=fit['eta_tail'], stab_zero=0.0 if tunnel else fit['stab_zero'])
        fn = P.fin()
        hgt = fn['h']
        AR = hgt ** 2 / fn['S']
        sweep = math.atan2(fn['strips'][-1]['x4'] - fn['strips'][0]['x4'], fn['strips'][-1]['z'] - fn['strips'][0]['z'])
        fn.update(AR_eff=1.55 * AR, cos_sweep=math.cos(sweep))
        cfg.update(fin=fn, sidewash=fit['sidewash'])
    # the rest of the zero-lift drag: flat-plate friction by Re at cruise, a form factor, on each body's wetted area
    def cf(L, V=65.0, nu=1.5e-5):
        return 0.455 / math.log10(V * L / nu) ** 2.58
    ff = 1 + 60 / (fus['length'] / fus['d_eq']) ** 3 + fus['length'] / fus['d_eq'] / 400
    area_m2 = cf(fus['length']) * ff * fus['wet'] * 1.1              # 1.1: windows, doors, rivets
    if nacelles:
        na = P.nacelle()
        fr = na['length'] / na['d_eq']
        area_m2 += 2 * cf(na['length']) * (1 + 0.35 / fr * 3) * na['wet'] * 1.2
    pod = P.body_stations(P.GEO['gear_pod']['sections'], [151.95, 190.55])
    area_m2 += 2 * cf(pod['length']) * 1.3 * pod['wet']
    cfg['drag_breakdown_m2'] = area_m2
    misc = 0.0 if tunnel else fit['cd_misc']
    cfg.update(drag_area_ft2=area_m2 * F * F + misc * P.REF['S_ft2'], gear_ft2=0.0 if tunnel else P.FLIGHT_GEAR_DRAG_FT2, drag_point=(3.5, 1.7), tyre=(172.0 / 12, 26.6 / 12))
    return cfg


def gear_xml(kg, cgx=None):
    W = kg * 2.20462
    xm = 172.0
    xn = xm - 2.9 / IN
    y = 1.0 / IN
    z = 26.6
    nose = (xm - cgx) / (xm - xn)                  # each strut set for the load it carries at this centre of gravity, so the aeroplane sits as built

    def unit(name, x, yy, load, steer, brake):
        k = load / 0.25
        return ('''<contact type="BOGEY" name="%s">
  <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
  <static_friction>0.72</static_friction><dynamic_friction>0.5</dynamic_friction><rolling_friction>0.005</rolling_friction>
  <spring_coeff unit="LBS/FT">%.0f</spring_coeff><damping_coeff unit="LBS/FT/SEC">%.0f</damping_coeff>
  <max_steer unit="DEG">%g</max_steer><brake_group>%s</brake_group><retractable>1</retractable>
</contact>''' % (name, x, yy, z, k, 0.6 * k ** 0.5 * (load / 32.174) ** 0.5 * 2, steer, brake))
    return '<ground_reactions>\n%s\n</ground_reactions>\n' % '\n'.join((unit('NOSE', xn, 0, W * nose, 25, 'NONE'), unit('LEFT_MAIN', xm, -y, W * (1 - nose) / 2, 0, 'LEFT'), unit('RIGHT_MAIN', xm, y, W * (1 - nose) / 2, 0, 'RIGHT')))


def fdm(aero, kg, cg_mac):
    m = P.MASS
    k = kg / m['kg'] / 1.35582
    mac = P.REF['c_ft'] * 12
    x = P.WING['le_x'] + cg_mac * mac
    z = P.WING['z'] - m['cg_below_chord'] * mac
    return '''<?xml version="1.0"?>
<!-- Tecnam P2006T validation build: generated by fdm/genp2006t.py -->
<fdm_config name="P2006T" version="2.0" release="ALPHA">
<fileheader><author>generated</author><description>Tecnam P2006T: the built-up method's proving model</description></fileheader>
<metrics>
  <wingarea unit="FT2">%.3f</wingarea><wingspan unit="FT">%.3f</wingspan><chord unit="FT">%.3f</chord>
  <location name="AERORP" unit="IN"><x>0</x><y>0</y><z>0</z></location>
  <location name="EYEPOINT" unit="IN"><x>125</x><y>-14</y><z>78</z></location>
  <location name="VRP" unit="IN"><x>0</x><y>0</y><z>0</z></location>
</metrics>
<mass_balance>
  <ixx unit="SLUG*FT2">%.1f</ixx><iyy unit="SLUG*FT2">%.1f</iyy><izz unit="SLUG*FT2">%.1f</izz>
  <emptywt unit="LBS">%.1f</emptywt>
  <location name="CG" unit="IN"><x>%.3f</x><y>0</y><z>%.3f</z></location>
</mass_balance>
%s
<system file="p2006t-controls"/>
%s
</fdm_config>
''' % (P.REF['S_ft2'], P.REF['b_ft'], P.REF['c_ft'], m['Ixx'] * k, m['Iyy'] * k, m['Izz'] * k, kg * 2.20462, x, z, gear_xml(kg, x), aero)


_cache = {}


def write(out, tunnel=False, tail=True, nacelles=True, kg=1180.0, cg=0.25, cfg_edit=None):
    key = (tunnel, tail, nacelles, repr(cfg_edit))
    if key not in _cache:
        c = config(tunnel, tail, nacelles)
        if cfg_edit:
            cfg_edit(c)
        a = BU.Aero(c)
        _cache[key] = (a.build(), c)
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    with open(os.path.join(out, 'p2006t.xml'), 'w') as f:
        f.write(fdm(_cache[key][0], kg, cg))
    with open(os.path.join(out, 'Systems', 'p2006t-controls.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n' + controls(main_spring(kg, cg)).xml())
    return _cache[key][1]


def main():
    a = sys.argv
    if len(a) < 2:
        sys.exit(__doc__)
    opt = lambda n, d: float(a[a.index(n) + 1]) if n in a else d
    write(a[1], '--tunnel' in a, '--no-tail' not in a, '--no-nacelles' not in a, opt('--kg', 1180.0), opt('--cg', 0.25))
    print('wrote', os.path.relpath(a[1], os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')))


if __name__ == '__main__':
    main()
