"""The X-57's propulsion as JSBSim systems: fourteen discs, their motors, the cruise propellers' hubs, and the battery.

Cruise motors. A torque lever per motor, mapped as NASA's [SIM]: regeneration, idle at 8 Nm a controller, normal up to 127.5 Nm
a controller at 100 %, overdrive to 175. The rotor's speed comes from the torque balance. The hub is a worm-geared servo at about
1 deg/s [DITTO], driven by the speed error, from the fine stop to feather; it stays put if it loses power. Over 2750 rpm the
controllers take torque off as the square of the excess; under 1100 they add it [SIM].

High-lift motors. Each of the twelve runs a speed loop on the Fixed or Airspeed mode's rpm, with its own rotor speed. Disarmed, the
motors brake to a stop and the blades fold. A motor that loses power can't brake, and the blades stay out, windmilling, as NASA
took them for its failure cases [TM table 7].

Discs. Thrust and torque from data/disc_tables.json (discs.py) on each disc's advance ratio; each also has the in-plane force of
the crossflow through it. With ctl/hold-props the speeds and pitches stand where they would settle, for trims: quasi-steady, as the
modes are taken.

Battery. Two packs, each a Thevenin model [CHIN] of a cell, scaled by 128 in series and 20 in parallel, with its state of charge,
polarisation and temperature. Loads reach it through NASA's efficiency chain [ANLYS-CEPT-018]. Each pack feeds one controller of
each cruise motor and one contactor pallet of high-lift motors.

No Nasal: all of it is JSBSim XML.
"""
import json, math, os
import numpy as np
from xmlfun import *
import x57data as X
import x57power as PW

HERE = os.path.dirname(os.path.abspath(__file__))
DT = json.load(open(os.path.join(HERE, 'data', 'disc_tables.json')))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
D_CRUISE = GEO['cruise']['diameter_in'] / 12.0
D_HL = GEO['high_lift']['diameter_in'] / 12.0
NM = 0.737562               # ft lbf a newton metre
KGM2 = 0.737562             # slug ft2 a kg m2
RAD = 2 * math.pi / 60      # rad/s an rpm
I_CRUISE = X.ROTOR['cruise_inertia_kgm2'] * KGM2
I_HL = X.ROTOR['hl_inertia_kgm2'] * KGM2
SIDES = (('l', 0, -1.0), ('r', 1, 1.0))       # side, engine, spin about body x: the inboard blades rise, against the tip vortex [TM]
PALLET = {i: ('a' if i in (1, 3, 5, 8, 10, 12) else 'b') for i in range(1, 13)}
SIDE_OF = {i: ('l' if i <= 6 else 'r') for i in range(1, 13)}


def hl_stations():
    """hubs 1 to 12, left tip to right tip"""
    hubs = GEO['high_lift']['hubs_in']
    return [(q[0], -q[1], q[2]) for q in reversed(hubs)] + [tuple(q) for q in hubs]


def disc_air(x, y, z):
    """the air at a disc (structural inches: x aft, y right, z up) in body axes, ft/s, with the airplane's roll, pitch and yaw about its centre of
    gravity: a yaw rate speeds one wingtip's disc and slows the other's, a roll rate tilts the flow through each. With the rates nil, the CG's air"""
    dx = D(p('prop/cg-x-ft'), v(x / 12.0))              # ahead of the CG
    dy = D(v(y / 12.0), p('prop/cg-y-ft'))              # right of it
    dz = D(p('prop/cg-z-ft'), v(z / 12.0))              # below it
    P_, Q_, R_ = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
    u = S(p('velocities/u-aero-fps'), M(Q_, dz), M(v(-1), R_, dy))
    vv = S(p('velocities/v-aero-fps'), M(R_, dx), M(v(-1), P_, dz))
    w = S(p('velocities/w-aero-fps'), M(P_, dy), M(v(-1), Q_, dx))
    return u, vv, w, (dx, dy, dz)


def turning(x, y, z):
    """what the airplane's turning adds to a disc's incidence and sideslip, rad: (p dy - q dx)/V, (r dx - p dz)/V"""
    dx = D(p('prop/cg-x-ft'), v(x / 12.0))
    dy = D(v(y / 12.0), p('prop/cg-y-ft'))
    dz = D(p('prop/cg-z-ft'), v(z / 12.0))
    P_, Q_, R_ = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
    V = p('prop/V-fps')
    return Q(D(M(P_, dy), M(Q_, dx)), V), Q(D(M(R_, dx), M(P_, dz)), V)


def spin_hl(i):
    """each wing's six the same way as its cruise propeller, inboard blades rising, the wings mirrored: so NASA's geometry (X-57.vsp3 and .glb), and the
    Mod IV CFD's skin friction [TM-20230010662 fig. 63], the same in every bay of a wing and mirrored on the other, and the corotating case of Deere et al.
    [AIAA Aviation 2017], all against the tip vortex, 0.31 more lift than pairs turning towards each other. [SIM]'s "each pair of motors was counter-rotating"
    then is left against right"""
    return -1.0 if i <= 6 else 1.0


def schedule():
    H = json.load(open(os.path.join(HERE, 'data', 'hlp_tables.json')))
    return H['keas'], H['hp_ft'], np.array(H['rpm']).T.tolist()


def cruise(f, s, e):
    c = DT['cruise']
    R = X.ROTOR
    B = X.HUB
    M_ = X.CRUISE_MOTOR
    rho = p('atmosphere/rho-slugs_ft3')
    hold = p('ctl/hold-now')
    hub = GEO['cruise']['hub_in']
    u = f.fn('prop/%s/u-fps' % s, MAX(v(0), disc_air(hub[0], (-1 if s == 'l' else 1) * hub[1], hub[2])[0]), desc='through the disc, with the airplane\'s turning')
    lever = f.fn('prop/%s/lever' % s, S(p('/controls/engines/engine[%d]/throttle' % e), p('ctl/lever-extra-%s' % s)), desc='FlightGear\'s lever stops at 1 and 0: overdrive and regeneration come by a switch and the last tenth of its travel')
    armed = GT(S(p('elec/cmc-%sa' % s), p('elec/cmc-%sb' % s)), v(0.5))
    live = f.fn('prop/%s/hub-live' % s, MAX(GT(ABS(lever), v(0.003)), GT(p('velocities/vc-kts'), v(25)), M(p('ctl/cockpit'), armed)), desc='parked with the lever closed; turning at idle in flight. In the cockpit an armed controller turns the motor at idle')
    run = f.fn('prop/%s/run' % s, M(p('prop/%s/hub-live' % s), p('ctl/motor-on-%s' % s)))
    lim = lambda b: M(p('elec/cmc-%s%s' % (s, b)), p('bat/%s/avail' % b))
    n_ctrl = f.fn('prop/%s/controllers' % s, S(lim('a'), lim('b')))
    per = T1('prop/%s/lever' % s,
             [-0.174, -0.0071, -0.007, 0.063, 1.0, 1.66], [R['regen_nm'], 0.0, 8.0, 8.0, M_['torque_100pct_nm'], M_['overdrive_torque_nm']])
    # the controllers limit the torque's rate [20240005892]; the limit isn't published. In the cockpit the full range takes a second, an estimate; the harness steps it
    f.fn('prop/%s/per-nm' % s, per)
    f.actuator('prop/%s/per-slew' % s, 'prop/%s/per-nm' % s, rate=M_['torque_100pct_nm'])
    per = IFTHEN(GT(p('ctl/cockpit'), v(0.5)), p('prop/%s/per-slew' % s), p('prop/%s/per-nm' % s))
    # a stuck command holds the torque it last asked for [SIM table 1]
    f.fn('prop/%s/cmd-nm' % s, IFTHEN(GT(p('ctl/cmd-stuck-%s' % s), v(0.5)), p('prop/%s/cmd-held' % s), M(per, n_ctrl, run)))
    # each propeller lever has its own speed in the cockpit; the harness sets one for both
    want = f.fn('prop/%s/rpm-want' % s, MIN(v(B['rpm_range'][1]), MAX(v(B['rpm_range'][0]), IFTHEN(GT(p('ctl/cruise-rpm-%s' % s), v(1)), p('ctl/cruise-rpm-%s' % s), p('ctl/cruise-rpm')))))
    cmd_rpm = f.fn('prop/%s/rpm-cmd' % s, M(want, run))
    st = 'prop/%s/rpm-state' % s
    # in a trim the speed stands at its command, unless the hub is on the fine stop with too little torque to hold it: it sags to
    # where the air takes that torque
    ub = MAX(v(10), p('prop/%s/u-fps' % s))
    f.fn('prop/%s/sag-x' % s, Q(M(D(p('prop/%s/cmd-nm' % s), v(R['friction_nm'])), v(NM)), M(rho, SQR(ub), v(D_CRUISE ** 3))))
    sag = M(v(60 / D_CRUISE), Q(ub, T1('prop/%s/sag-x' % s, c['sag_x'], c['sag_J'])))
    rpm = f.fn('prop/%s/rpm' % s, IFTHEN(GT(hold, v(0.5)), MIN(cmd_rpm, sag), p(st)))
    over = M(MAX(v(0), D(rpm, v(M_['overspeed_rpm']))), v(RAD))
    under = M(MAX(v(0), D(v(M_['underspeed_rpm']), rpm)), v(RAD))
    nm = f.fn('prop/%s/torque-nm' % s, S(p('prop/%s/cmd-nm' % s), M(v(-R['kpo_nm']), SQR(over)),
                                         M(run, GT(p('prop/%s/controllers' % s), v(0.01)), MIN(v(60.0), M(v(R['kpu_nm']), SQR(under)))), M(D(v(1), run), v(-15.0), GT(rpm, v(0)))),
              clip=(str(-260.0), str(260.0)))
    # the disc: an advance ratio no higher than the tables', so that a stopped propeller still sees the air
    nf_ = f.fn('prop/%s/n' % s, MAX(M(rpm, v(1 / 60.0)), Q(p('prop/%s/u-fps' % s), v(4.0 * D_CRUISE)), v(0.02)))
    J = f.fn('prop/%s/J' % s, Q(p('prop/%s/u-fps' % s), M(nf_, v(D_CRUISE))))
    k4 = f.fn('prop/%s/k4' % s, M(rho, SQR(nf_), v(D_CRUISE ** 4)))
    # in a trim the pitch that absorbs the torque
    q_hold = M(D(nm, v(R['friction_nm'])), v(NM))
    cp_hold = f.fn('prop/%s/Cp-hold' % s, Q(M(q_hold, v(2 * math.pi)), M(p('prop/%s/k4' % s), v(D_CRUISE))))
    f.fn('prop/%s/Cp-over' % s, MAX(v(0), D(cp_hold, T1('prop/%s/J' % s, c['J'], c['Cp_fine']))), desc='above what the fine stop takes')
    pit = T2('prop/%s/J' % s, 'prop/%s/Cp-over' % s, c['J'], c['Cp_axis'], c['pitch_of'])
    pst = 'prop/%s/pitch-state' % s
    pitch = f.fn('prop/%s/pitch-deg' % s, IFTHEN(GT(p('ctl/feather-now-%s' % s), v(0.5)), v(B['feather_deg']), IFTHEN(GT(hold, v(0.5)), pit, p(pst))))
    ct = f.fn('prop/%s/Ct' % s, T2('prop/%s/J' % s, 'prop/%s/pitch-deg' % s, c['J'], c['pitch'], c['Ct']))
    cp = f.fn('prop/%s/Cp' % s, T2('prop/%s/J' % s, 'prop/%s/pitch-deg' % s, c['J'], c['pitch'], c['Cp']))
    keep = D(v(1), p('ctl/no-prop-forces-%s' % s))     # NASA's failure with the propeller's forces gone [SIM table 1]
    f.fn('prop/%s/thrust-lbs' % s, M(ct, p('prop/%s/k4' % s), keep))
    # the rotor. Torque the air takes, ft lbf: Cp rho n^2 D^5 / 2 pi
    qa = f.fn('prop/%s/q-air-ftlb' % s, M(keep, IFTHEN(GT(hold, v(0.5)), q_hold, M(cp, p('prop/%s/k4' % s), v(D_CRUISE / (2 * math.pi))))))
    f.fn('prop/%s/inplane-lbs' % s, M(T2('prop/%s/J' % s, 'prop/%s/pitch-deg' % s, c['J'], c['pitch'], c['Cn']), p('prop/%s/k4' % s), keep),
         desc='per radian of incidence')
    fric = M(v(R['friction_nm'] * NM), GT(rpm, v(0)))
    dt = p('simulation/channel-dt')
    acc = Q(D(D(M(nm, v(NM)), qa), fric), v(I_CRUISE))
    nxt = S(rpm, M(acc, dt, v(1 / RAD)))
    f.fn('prop/%s/rpm-next' % s, IFTHEN(GT(hold, v(0.5)), rpm, MAX(v(0), MIN(v(3400), nxt))), out=st)
    # the hub: rate-limited on the speed error; fixed if it fails; to feather on the switch. With its motor gone it goes on governing, so the falling speed drives it
    # toward windmilling, as NASA's failed propeller [20240006682]
    err = D(rpm, M(want, p('prop/%s/hub-live' % s)))
    rate = MIN(v(B['rate_deg_s']), MAX(v(-B['rate_deg_s']), M(v(0.02), err)))
    # AUTO governs, above 200 rpm in the cockpit; MAN moves only while START or FEATHER is held; the lever's aft stop overrides both [ICD slides 25-29]
    auto = M(rate, GT(p('prop/%s/hub-live' % s), v(0.5)), MAX(D(v(1), p('ctl/cockpit')), GT(rpm, v(200))))
    man = M(p('ctl/pitch-sw-%s' % s), v(B['rate_deg_s']))
    move = IFTHEN(GT(p('ctl/feather-%s' % s), v(0.5)), v(B['rate_deg_s']), IFTHEN(GT(p('ctl/hub-fine-%s' % s), v(0.5)), v(-B['rate_deg_s']), IFTHEN(GT(p('ctl/pitch-man-%s' % s), v(0.5)), man, auto)))
    pn = S(pitch, M(move, dt, D(v(1), p('ctl/hub-fixed-%s' % s)), p('elec/hub-%s' % s)))
    f.fn('prop/%s/cmd-next' % s, p('prop/%s/cmd-nm' % s), out='prop/%s/cmd-held' % s)
    f.fn('prop/%s/pitch-next' % s, IFTHEN(GT(hold, v(0.5)), pitch, MIN(v(B['feather_deg']), MAX(v(B['fine_deg']), pn))), out=pst)
    f.fn('prop/%s/shaft-kw' % s, M(nm, v(NM / 737.56), rpm, v(RAD)))
    f.fn('prop/%s/power-kw' % s, M(nm, v(NM / 737.56), rpm, v(RAD)))


def high_lift(f):
    h = DT['high_lift']
    R = X.ROTOR
    rho = p('atmosphere/rho-slugs_ft3')
    hold = p('ctl/hold-now')
    dt = p('simulation/channel-dt')
    keas, hps, rpm_tab = schedule()
    sched = T2('velocities/ve-kts', 'position/h-sl-ft', keas, hps, rpm_tab)
    mode = p('ctl/hl-mode')
    f.fn('prop/hl/rpm-cmd', IFTHEN(LT(mode, v(0.5)), v(0), IFTHEN(LT(mode, v(1.5)), v(X.HIGH_LIFT['fixed_mode_rpm']),
                                                                 IFTHEN(LT(mode, v(2.5)), sched, p('ctl/hl-rpm-test')))),
         desc='mode 0 disarmed, 1 fixed, 2 airspeed, 3 the harness\'s rpm')
    cmd = p('prop/hl/rpm-cmd')
    u0 = f.fn('prop/hl/u-fps', MAX(v(0), p('velocities/u-aero-fps')))
    lim = R['hl_torque_limit_nm']
    for i, q in enumerate(hl_stations(), 1):
        k = 'prop/hl/d%d/' % i
        u = f.fn(k + 'u-fps', MAX(v(0), disc_air(*q)[0]), desc='through this disc, with the airplane\'s turning')
        on = f.fn(k + 'power', M(p('elec/contactor-%s%s' % (PALLET[i], SIDE_OF[i])), p('ctl/hl-motor-%d' % i), p('bat/%s/avail' % PALLET[i])))
        st = k + 'rpm-state'
        rpm = f.fn(k + 'rpm', IFTHEN(GT(hold, v(0.5)), M(cmd, on), p(st)))
        n = f.fn(k + 'n', MAX(M(rpm, v(1 / 60.0)), Q(u, v(3.0 * D_HL)), v(0.02)))
        f.fn(k + 'J', Q(u, M(n, v(D_HL))))
        # the blades are out while it spins: they fold as it stops, and stay folded in a stream only if braked
        out = f.fn(k + 'deployed', MIN(v(1), MAX(v(0), M(v(1 / 400.0), D(rpm, v(250))))))
        k4 = f.fn(k + 'k4', M(rho, SQR(n), v(D_HL ** 4)))
        # the harness can run NASA's own table, as its simulator did, in place of the tunnel's
        nt = GT(p('ctl/hl-nasa-table'), v(0.5))
        f.fn(k + 'thrust-lbs', M(out, IFTHEN(nt, T1(k + 'J', h['J'], h['nasa_Ct']), T1(k + 'J', h['J'], h['Ct'])), k4))
        qa = f.fn(k + 'q-air-ftlb', M(out, IFTHEN(nt, T1(k + 'J', h['J'], h['nasa_Cq']), T1(k + 'J', h['J'], h['Cq'])), k4, v(D_HL)))
        # the speed loop, with the air's torque fed forward
        loop = S(M(D(cmd, rpm), v(RAD * 0.08)), Q(qa, v(NM)))
        tq = f.fn(k + 'torque-nm', IFTHEN(GT(hold, v(0.5)), Q(qa, v(NM)), M(on, MIN(v(lim), MAX(v(-lim), loop)))))
        acc = Q(D(D(M(tq, v(NM)), qa), M(v(0.2 * NM), GT(rpm, v(0)))), v(I_HL))
        nxt = f.fn(k + 'rpm-free', MAX(v(0), MIN(v(6500), S(rpm, M(acc, dt, v(1 / RAD))))))
        # stopped by the motor, it stays stopped
        stop = M(LT(nxt, v(30)), LT(cmd, v(1)), on)
        f.fn(k + 'rpm-next', IFTHEN(GT(hold, v(0.5)), rpm, IFTHEN(GT(stop, v(0.5)), v(0), nxt)), out=st)
        f.fn(k + 'shaft-kw', M(tq, v(NM / 737.56), rpm, v(RAD)))
    # for the aerodynamics: how many discs are working, and at what advance ratio. A disc works while powered and its blades are out
    work = [M(p('prop/hl/d%d/deployed' % i), p('prop/hl/d%d/power' % i)) for i in range(1, 13)]
    for s, a, b in (('l', 0, 6), ('r', 6, 12)):
        f.fn('prop/hl/on-%s' % s, M(S(*work[a:b]), v(1 / 6.0)))
    f.fn('prop/hl/on', M(v(0.5), S(p('prop/hl/on-l'), p('prop/hl/on-r'))))
    f.fn('prop/hl/J', Q(S(*[M(w, p('prop/hl/d%d/J' % i)) for i, w in zip(range(1, 13), work)]), MAX(v(0.01), M(v(12), p('prop/hl/on')))))
    f.fn('prop/hl/rpm', M(v(1 / 12.0), S(*[p('prop/hl/d%d/rpm' % i) for i in range(1, 13)])))
    f.fn('prop/hl/thrust-each-lbs', M(v(1 / 12.0), S(*[p('prop/hl/d%d/thrust-lbs' % i) for i in range(1, 13)])))
    f.fn('prop/hl/power-each-kw', M(v(1 / 12.0), S(*[p('prop/hl/d%d/shaft-kw' % i) for i in range(1, 13)])))
    f.fn('prop/hl/inplane-lbs', M(T1('prop/hl/J', h['J'], h['Cn']), rho, SQR(M(p('prop/hl/rpm'), v(1 / 60.0))), v(12 * D_HL ** 4), p('prop/hl/on')),
         desc='all twelve, per radian of incidence')


def battery(f, hl):
    E, C, Bt = X.ETA, X.CELL, X.BATTERY
    f.channel('battery', execrate=12)
    dt = p('simulation/channel-dt')
    hold = p('ctl/hold-now')
    for b in ('a', 'b'):
        g = 'bat/%s/' % b
        # each controller carries its share of its motor's shaft power; a load through the chain, a charge back through it
        terms = []
        for s, e, _ in SIDES:
            has = lambda k: M(p('elec/cmc-%s%s' % (s, k)), p('bat/%s/avail' % k))
            share = Q(has(b), MAX(v(0.01), S(has('a'), has('b'))))
            sh = M(p('prop/%s/shaft-kw' % s), share)
            eta = E['cruise_motor'] * E['cruise_controller']
            terms.append(IFTHEN(GT(sh, v(0)), M(sh, v(1 / eta)), M(sh, v(eta))))
        if hl:
            eta = E['hl_motor'] * E['hl_controller']
            sh = S(*[p('prop/hl/d%d/shaft-kw' % i) for i in range(1, 13) if PALLET[i] == b])
            terms.append(IFTHEN(GT(sh, v(0)), M(sh, v(1 / eta)), M(sh, v(eta))))
        load = S(M(v(X.AVIONICS_KW_A_SIDE), Q(p('elec/conv-%s-amps' % b), v(PW.nominal(hl)['ab'.index(b)]))), *terms)
        kw = f.fn(g + 'load-kw', IFTHEN(GT(p('ctl/bus-test'), v(0.5)), p('ctl/bus-test-kw'), M(load, v(1 / E['bus']), p('elec/bus-%s' % b))))
        soc, uth, tc = p(g + 'soc'), p(g + 'uth'), p(g + 'temp-c')
        uoc = f.fn(g + 'uoc', T1(g + 'soc', C['soc'][::-1], C['uoc'][::-1]))
        r0 = f.fn(g + 'r0', T2(g + 'soc', g + 'temp-c', C['soc'][::-1], C['temp_c'], np.array(C['r0']).T[::-1].tolist()))
        rth = f.fn(g + 'rth', T2(g + 'soc', g + 'temp-c', C['soc'][::-1], C['temp_c'], np.array(C['rth']).T[::-1].tolist()))
        # the cell's current for the load at the cell's voltage: the quadratic of P = (U - I R0) I, its lower root
        u0 = f.fn(g + 'u0', D(uoc, uth))
        pc = f.fn(g + 'cell-w', M(kw, v(1000.0 / (Bt['series'] * Bt['parallel']))))
        disc = MAX(v(0.01), D(SQR(u0), M(v(4), r0, pc)))
        i = f.fn(g + 'cell-a', Q(D(u0, SQ(disc)), M(v(2), r0)))
        ul = f.fn(g + 'cell-v', D(u0, M(i, r0)))
        f.fn(g + 'volts', M(ul, v(Bt['series'])))
        f.fn(g + 'amps', M(i, v(Bt['parallel'])))
        f.fn(g + 'soc-next', IFTHEN(GT(hold, v(0.5)), soc, MIN(v(1), MAX(v(0), D(soc, M(i, dt, v(1 / (3600.0 * Bt['cell_ah']))))))), out=g + 'soc')
        f.fn(g + 'uth-next', IFTHEN(GT(hold, v(0.5)), uth, S(uth, M(dt, Q(D(i, Q(uth, rth)), v(C['cth_f']))))), out=g + 'uth')
        heat = M(SQR(i), S(r0, rth))
        cool = M(v(0.02), D(tc, p('atmosphere/T-R') if False else v(20.0)))
        f.fn(g + 'temp-next', IFTHEN(GT(hold, v(0.5)), tc, S(tc, M(dt, Q(D(heat, cool), v(Bt['cell_mass_kg'] * 900.0))))), out=g + 'temp-c')
        # empty or sagging to the controllers' floor, it gives out
        f.fn(g + 'avail', M(p('elec/bus-%s' % b), MIN(v(1), MAX(v(0), M(v(0.05), D(p(g + 'volts'), v(Bt['min_pack_v'])))))), out=g + 'avail-out')
    PW.battery(f)


def build(hl):
    f = FCS('x57-propulsion')
    f.declare('ctl/hold-props', 0)
    f.declare('ctl/overdrive', 0)
    f.declare('ctl/regen', 0)
    f.declare('ctl/cruise-rpm', 2250)
    f.declare('ctl/bus-test', 0)
    f.declare('ctl/bus-test-kw', 0)
    for s, e, _ in SIDES:
        f.declare('/controls/engines/engine[%d]/throttle' % e, 0)
        f.declare('ctl/cruise-rpm-%s' % s, 0)
        f.declare('ctl/motor-on-%s' % s, 1)
        f.declare('ctl/cmc-%sa' % s, 1)
        f.declare('ctl/cmc-%sb' % s, 1)
        f.declare('ctl/feather-%s' % s, 0)
        f.declare('ctl/lever-extra-%s' % s, 0)
        f.declare('ctl/hub-fixed-%s' % s, 0)
        f.declare('ctl/hub-fine-%s' % s, 0)
        f.declare('ctl/feather-now-%s' % s, 0)
        f.declare('ctl/no-prop-forces-%s' % s, 0)
        f.declare('ctl/cmd-stuck-%s' % s, 0)
        f.declare('prop/%s/cmd-held' % s, 0)
        f.declare('prop/%s/rpm-state' % s, 0)
        f.declare('prop/%s/pitch-state' % s, X.HUB['fine_deg'])
    for b in ('a', 'b'):
        f.declare('ctl/bus-%s' % b, 1)
        f.declare('bat/%s/soc' % b, 1.0)
        f.declare('bat/%s/uth' % b, 0.0)
        f.declare('bat/%s/temp-c' % b, 20.0)
        f.declare('bat/%s/avail' % b, 1.0)
        f.declare('bat/%s/avail-out' % b, 1.0)
    if hl:
        f.declare('ctl/hl-mode', 0)
        f.declare('ctl/hl-nasa-table', 0)
        f.declare('ctl/hl-rpm-test', 0)
        for s in ('l', 'r'):
            for b in ('a', 'b'):
                f.declare('ctl/contactor-%s%s' % (b, s), 1)
        for i in range(1, 13):
            f.declare('ctl/hl-motor-%d' % i, 1)
            f.declare('prop/hl/d%d/rpm-state' % i, 0)
    PW.declare(f, hl)
    PW.channel(f, hl)
    f.channel('state')
    f.fn('ctl/hold-now', MAX(p('ctl/hold-props'), M(LT(p('simulation/sim-time-sec'), v(2)), GT(p('position/h-agl-ft'), v(50)))))
    for a in 'xyz':           # the discs' air turns with the airplane about its centre of gravity
        f.fn('prop/cg-%s-ft' % a, M(p('inertia/cg-%s-in' % a), v(1 / 12.0)))
    f.fn('prop/V-fps', MAX(p('velocities/vt-fps'), v(30.0)), desc='for the turning\'s angles at the discs, floored as the wing\'s')
    if hl:      # stopped and disarmed, everything the high-lift channels make is nil: they rest, holding it. Each property read takes a lock in FlightGear
        f.fn('prop/hl/live', MAX(GT(p('ctl/hl-mode'), v(0.5)), GT(p('prop/hl/rpm'), v(0)), GT(p('bu/test/hl-thrust'), v(-900))),
             desc='armed, still turning, or the harness\'s thrust going through the discs')
    for s, e, _ in SIDES:
        f.channel('cruise ' + s)
        cruise(f, s, e)
    if hl:
        f.channel('high-lift', execute='prop/hl/live')
        high_lift(f)
    battery(f, hl)
    # what the sound, the model and the instruments read. cruise[0] left, cruise[1] right; hl[1] to hl[12] from the left tip
    f.channel('outputs')
    rho = p('atmosphere/rho-slugs_ft3')
    full = X.CRUISE_MOTOR['torque_100pct_nm'] * X.CRUISE_MOTOR['controllers']
    for i, (s, e, _) in enumerate(SIDES):
        k = 'x57/cruise[%d]' % i
        f.fn(k + '/rpm', p('prop/%s/rpm' % s))
        f.fn(k + '/ct', p('prop/%s/Ct' % s))
        f.fn(k + '/torque-norm', M(p('prop/%s/torque-nm' % s), v(1 / full)))
    if hl:
        for i in range(1, 13):
            k = 'x57/hl[%d]' % i
            f.fn(k + '/rpm', p('prop/hl/d%d/rpm' % i))
            f.fn(k + '/torque-norm', M(p('prop/hl/d%d/torque-nm' % i), v(1 / X.HIGH_LIFT['rated_torque_nm'])))
    for b in ('a', 'b'):
        f.fn('x57/battery-%s/soc-pct' % b, M(p('bat/%s/soc' % b), v(100)))
    return f


def forces(hl, mrs, flex=False):
    """thrust and in-plane sums and moments about the reference, for genx57's axes: X, Y, Z, roll, pitch, yaw terms. Station x
    aft, y right, z up; body x forward, y right, z down."""
    al, be = p('aero/alpha-rad'), p('aero/beta-rad')
    X_, Y_, Z_, L_, M_, N_ = [], [], [], [], [], []
    hub = GEO['cruise']['hub_in']
    for s, e, spin in SIDES:
        dx, dy, dz = (mrs[0] - hub[0]) / 12.0, spin * hub[1] / 12.0, -(hub[2] - mrs[2]) / 12.0
        T = p('prop/%s/thrust-lbs' % s)
        N = p('prop/%s/inplane-lbs' % s)
        ta, tb = turning(hub[0], spin * hub[1], hub[2])
        fy, fz = M(v(-1), N, S(be, tb)), M(v(-1), N, S(al, ta))
        X_.append(T)
        Y_.append(fy)
        Z_.append(fz)
        L_.append(S(M(v(dy), fz), M(v(-dz), fy)))
        M_.append(S(M(v(dz), T), M(v(-dx), fz)))
        if flex:                # bending lifts the thrust line; it doesn't turn it, since it turns about body x
            M_.append(M(v(-1), p('flex/q%s-ft' % s.upper()), T))
        N_.append(S(M(v(dx), fy), M(v(-dy), T)))
        L_.append(M(v(-spin), p('prop/%s/q-air-ftlb' % s)))              # the motor's reaction on the airframe
    if hl:
        st = hl_stations()
        for i, q in enumerate(st, 1):
            dy, dz = q[1] / 12.0, -(q[2] - mrs[2]) / 12.0
            T = p('prop/hl/d%d/thrust-lbs' % i)
            X_.append(T)
            M_.append(M(v(dz), T))
            N_.append(M(v(-dy), T))
            L_.append(M(v(-spin_hl(i)), p('prop/hl/d%d/q-air-ftlb' % i)))
        dx = (mrs[0] - float(np.mean([q[0] for q in st]))) / 12.0
        N = p('prop/hl/inplane-lbs')
        mid = [float(np.mean([q[k] for q in st])) for k in range(3)]
        ta, tb = turning(mid[0], 0.0, mid[2])         # the twelve together; their spread in span gives the roll's damping below
        fy, fz = M(v(-1), N, S(be, tb)), M(v(-1), N, S(al, ta))
        Y_.append(fy)
        Z_.append(fz)
        M_.append(M(v(-dx), fz))
        N_.append(M(v(dx), fy))
        # a roll rate tilts each disc's flow by p y / V, so each one's in-plane force: about the mean, sum y^2 over the twelve
        y2 = sum((q[1] / 12.0) ** 2 for q in st) / 12.0
        L_.append(M(v(-y2), N, Q(p('velocities/p-aero-rad_sec'), p('prop/V-fps'))))
    return X_, Y_, Z_, L_, M_, N_


def gyroscopic(hl):
    """the rotors' angular momentum about body x, slug ft2 rad/s: its moments are -r H in pitch, q H in yaw"""
    h = [M(v(spin * I_CRUISE * RAD), p('prop/%s/rpm' % s)) for s, e, spin in SIDES]
    if hl:
        h += [M(v(spin_hl(i) * I_HL * RAD), p('prop/hl/d%d/rpm' % i)) for i in range(1, 13)]
    return S(*h)
