"""The X-57 Mod III/IV numbers entered by hand, each with its source. What comes from NASA's files by program is in
data/ (geometry.py, cfd.py, digitise_fq.py).

Sources:
  [SIM]  Wallace et al., Development of the X-57 Mod III/IV Piloted Simulator, NTRS 20240006496
  [TM]   Frederick et al., Development of the X-57 Aerodynamic Database, NASA/TM-20250001715
  [PERF] Flight Performance Estimates for the X-57, NTRS 20240010807
  [ASYM] Mitigation of High Lateral Asymmetry Rates, NTRS 20240006682
  [DITTO] Distributed Thrust Takeoff, NTRS 20240010539
  [PROP] ANLYS-CEPT-023, High-Lift Propeller Operating Conditions, NTRS 20240014295
  [TEST] High-Lift Propeller Testing and Model Development, NTRS 20210016834
  [CHIN] Chin et al., Battery Performance Modeling on Maxwell X-57, 2019
  [GROUND] X-57 Mod II ground dynamics and handling, NTRS 20220005440
  [AFM]  Tecnam P2006T Aircraft Flight Manual, 912S3, 4th edition
"""
SLUG_FT2 = 32.174 * 144      # lb in2 per slug ft2

# Mass, projected from NASA's CAD model, Mod III and IV alike [SIM]. The paper prints the moments' units as lb/in2:
# they are lb in2, as its products of inertia. The design weight is 3,000 lb [TM], and later performance work went to
# 1452 kg [PERF]
# What FlightGear flies is the published gross weight: 3,000 lb [TM; PERF's 1361 kg] for Mod III and 3,200 lb projected for Mod IV, its 12 motors of
# about 16 lb on top [20240007439; PERF's 1452 kg]. NASA's simulator ran both at 3,075 lb, so the checks against it stay there. 28.6 % is the only CG published.
MASS = dict(weight_lb=3075.0, design_weight_lb=3000.0, flight_weight_lb=dict(mod3=3000.0, mod4=3200.0),
            Ixx_lbin2=18.6812e6, Iyy_lbin2=8.35934e6, Izz_lbin2=25.2127e6,
            Ixy_lbin2=1.5528e3, Ixz_lbin2=3.213e4, Iyz_lbin2=3.7987e2,
            cg_mac=0.286, cg_y_in=-1.19, cg_below_wing_le_in=21.2)

# Control surfaces, deg. Stabilator and aileron positive trailing edge down, rudder positive trailing edge left [TM].
# Stabilator travel [SIM]. Aileron and tab: the full deflection ranges the CFD swept [TM table 4], aileron for the right
# surface alone. The tab was swept at flaps 0 only. The rudder 26 each way: the P2006T's flight manual (26, tolerance 2
# [AFM]), NASA's ground-handling simulation (+-26 [GROUND]) and its Mod III simulator, whose pilots hold it at 26 after
# a motor failure [ASYM fig. 14]; the tail is the P2006T's on every Mod. The CFD swept it to 28, the tolerance's far end. The
# flight manual gives ailerons up 20, down 17, and the stabilator's trailing edge up 4, down 15, the opposite way round
# to NASA's; NASA's figures are used.
SURFACES = dict(stabilator=(-15.0, 5.0), aileron=(-25.0, 18.0), rudder=(-26.0, 26.0), tab=(-12.0, 11.0),
                flaps=(0.0, 10.0, 30.0))         # 20 deg also has 11 power-off cases

# Speeds, KCAS unless marked. Mod III [SIM, ASYM, PERF]; Mod IV with high lift on [SIM]; takeoff [DITTO, KIAS]
SPEEDS = dict(
    vne=172.0,
    mod3=dict(vs_clean=88.0, vs1=80.0, vs0=73.0, vref=94.0, vmc=85.0, vr=88.0, vy=97.0),
    mod4=dict(vs1_hl=70.0, vs0_hl=58.0, vref=75.0, vmc=84.0, vr_kias=77.0, vy_kias=84.0),
)
CEILING_FT = dict(capable=14000.0, operational=10000.0)     # [SIM]

# Cruise motors, torque lever in percent [SIM, 20230004174]. Each motor has two controllers, and the 127.5 Nm at 100 % is one
# controller's: both give 255 Nm, 72 kW at 2700 rpm, the 72-kW setting of the taxi test plan [20240007413]. The 175 Nm
# overdrive is for one controller alone. Idle is 8 Nm a controller. The regeneration region's torque isn't settled: the
# interface document had 0 Nm, with values planned for Mod IV [DITTO]; -38 Nm a controller is the -18 kW of the CFD's lowest case
CRUISE_MOTOR = dict(torque_100pct_nm=127.5, controllers=2, rpm_100pct=2700.0, overdrive_torque_nm=175.0,
                    regen_lever_pct=(-17.4, -0.7), nominal_lever_pct=(-0.7, 100.0), idle_lever_pct=(-0.7, 6.5),
                    overspeed_rpm=2750.0, underspeed_rpm=1100.0, controller_efficiency=0.97,
                    propeller='MT-Propeller MTV-7-152/64', pitch_range_deg=(14.0, 40.0))

# High-lift motors [SIM, PROP, TEST]. Numbered 1-12 across the span; a contactor loses three [TM]
HIGH_LIFT = dict(fixed_mode_rpm=4800.0, airspeed_mode_kcas=(58.0, 90.0), design_rpm=4549.0, design_power_kw=10.5, rated_torque_nm=20.5,
                 design_thrust_lbf=49.5, contactors=dict(A=((1, 3, 5), (8, 10, 12)), B=((2, 4, 6), (7, 9, 11))))

# Battery. Two packs, A and B, of 2,560 cells each [20240009560]: 128 in series, 20 in parallel, 60 Ah. (Chin's "128 x 40" is
# the two together.) 10 Wh a cell usable at 10 A, 25.6 kWh a pack, 51.2 kWh in all [ANLYS-CEPT-018]. Pack A feeds the A controller
# of each cruise motor and contactor pallet A's high-lift motors; pack B the same on B [20240009560, TM]
BATTERY = dict(cell='Samsung SDI 18650-30Q', cell_ah=3.0, cell_v=3.6, series=128, parallel=20, packs=2,
               usable_kwh=51.2, min_pack_v=330.0, cell_mass_kg=350.0 / 5120)
# The cell's Thevenin model, read by eye from Chin fig. 8 (ohm, cell): open-circuit volts on state of charge; R0 and RTh on state
# of charge and temperature. CTh isn't plotted: 1300 F gives RTh CTh of about 40 s, an estimate
CELL = dict(soc=[1.0, 0.97, 0.93, 0.85, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.0],
            uoc=[4.16, 4.11, 4.08, 4.06, 4.02, 3.92, 3.82, 3.74, 3.66, 3.55, 3.40, 3.20, 3.08, 2.93],
            temp_c=[0.0, 20.0, 45.0],
            r0=[[.065, .065, .065, .065, .065, .065, .065, .065, .065, .07, .085, .13, .19, .25],
                [.025, .025, .025, .025, .025, .025, .025, .025, .025, .025, .026, .045, .065, .09],
                [.025, .025, .025, .025, .025, .025, .025, .025, .025, .025, .025, .03, .05, .07]],
            rth=[[.04, .04, .055, .075, .075, .07, .07, .07, .08, .06, .06, .09, .09, .09],
                 [.02, .025, .03, .03, .04, .04, .03, .05, .04, .04, .045, .055, .07, .085],
                 [.02, .02, .02, .02, .03, .03, .02, .025, .03, .03, .04, .045, .05, .07]],
            cth_f=1300.0)
# Efficiencies, and the avionics' draw on each side [ANLYS-CEPT-018]
ETA = dict(cruise_motor=0.92, cruise_controller=0.97, hl_motor=0.93, hl_controller=0.98, bus=0.995)
AVIONICS_KW_A_SIDE = 1.0
# Estimates: no inertia is published. The high-lift rotor: five blades of 2.56 to 2.70 oz [TEST table 4], radius of gyration 0.45 R, 0.0064 kg m2, and the rotor, magnets
# and hub of NASA's reference motor [20190029267 table 8], 0.517 kg at about 60 mm and 0.255 kg at about 30 mm, 0.0021. The cruise rotor, range 0.58 to 0.73: an MTV-7 is 0.59 with 160 cm blades [MT data sheet], about 0.5 at 152 cm, and the motor's rotor 0.12 to 0.2, worked in docs/PLAN.md.
# Over and underspeed gains: the laws are NASA's, the gains aren't published
ROTOR = dict(cruise_inertia_kgm2=0.65, hl_inertia_kgm2=0.0085, friction_nm=1.5, kpo_nm=0.9, kpu_nm=0.2, regen_nm=-38.0, hl_torque_limit_nm=30.0)
HUB = dict(rate_deg_s=1.0, fine_deg=14.0, feather_deg=85.0, rpm_range=(1700.0, 2700.0))      # [DITTO, 20240007413, CFD range]

# Landing gear, from the P2006T's flight manual: track 2.0 m, wheelbase 2.9 m. Mods II-IV kept it [SIM]. The lowest ground
# plane in NASA's CFD, 5.76 ft below the wing's leading edge, is taken as the ground under the wheels. The main gear's
# station is estimated from the gear pods in the OpenVSP model (x 151-190 in). Spring and damper rates are estimates
GEAR = dict(track_in=78.74, wheelbase_in=114.17, main_x_in=172.0, ground_z_in=87.491 - 5.76 * 12, main_load_share=0.85, sag_in=3.0)
# Drag of the gear down: from NASA's P2006T flight identification, not published [SIM]. 4 ft2 of drag area is fitted to NASA's
# predicted phugoid damping (figs. 18, 19): one value serves every speed, and it is a CD of 0.025 on the P2006T's wing
GEAR_DRAG_AREA_FT2 = 4.0
# The wheels' spin, for the touchdowns (genx57.controls): each wheel's inertia about its axle and its rolling radius, PyWopWop's for the same tyres
# (tyre.py wheel_inertia: the P2006T's 6.00-6 mains and 5.00-5 nose, Goodyear's sizes, the tyres' catalogue weights; mktouchdown.py checks they
# agree), and the torque that spins a free wheel down in the air: an estimate, no measurement found. A main wheel's brake pads ride its disc and
# its bearings drag, 3 N m, stopping it from 85 kt in some 14 s; one pilot's account has a light aircraft's wheels still within seconds of lift-off.
# The nose wheel has no brake: bearings and seals, 0.5 N m, some 45 s
WHEELS = {'6.00-6': dict(inertia_kgm2=0.1952, radius_m=0.207, drag_nm=3.0), '5.00-5': dict(inertia_kgm2=0.0859, radius_m=0.168, drag_nm=0.5)}
