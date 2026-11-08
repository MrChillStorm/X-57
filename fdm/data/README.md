# X-57 data

Everything here is generated from NASA's public files, which stay in the git-ignored `refs/`. To rebuild it and run
the checks:

```bash
sh fdm/build_data.sh
```

| Path | From |
|---|---|
| `geometry.json`, `sections/` | NASA's OpenVSP model (NASA-3D-Resources), by `fdm/geometry.py` |
| `cfd/` | NASA's CFD spreadsheets (NTRS 20250004430), by `fdm/cfd.py` |
| `digitised/flying_qualities.json` | Figs. 18-24 of NTRS 20240006496, by `fdm/digitise_fq.py` |
| `digitised/tunnel_rates.json` | The wind tunnel's curves in TM-20250001715 figs. 58-62, by `fdm/digitise_tunnel.py` |
| `digitised/hlp_tunnel.json` | The high-lift propeller's 2022 tunnel retest, slides 6 and 7, by `fdm/digitise_hlp.py` |
| `hlp_tables.json` | ANLYS-CEPT-023 appendices B and C, the final high-lift schedule, by `fdm/hlp_data.py` |
| `aero_tables.json` | The table model's tables, built from all of these by `fdm/aerotables.py` |
| `disc_tables.json` | Thrust, torque and in-plane force of each kind of propeller, by `fdm/discs.py` |
| `p2006t_prop.json` | The P2006T's propeller, by `fdm/p2006t.py` |
| `x57_section_fit.json` | What the built-up wing's sections need to give the CFD's curves, by `fdm/x57fit.py` |

The flying qualities are NASA's simulator predictions, not measurements. Mods II, III and IV never flew.

## Slips in NASA's files

Found by `fdm/phase0_checks.py`, and what we do about each.

| Where | What | What we do |
|---|---|---|
| Mod-IV, Flaps 10, 58 and 75 KCAS | Alpha reads `4,6` as text. Every solver is at 4 deg except STAR-CCM+ power on at 58 KCAS, at 6 deg | Rows set to 4 deg; that value is its own 6 deg row, and its delta, 6 deg less 4 deg, is dropped |
| Mod-IV, Flaps 10, Kestrel, 58 KCAS | 34 delta values aren't power on minus power off, by up to 0.117 | Use on minus off, not the delta column |
| Mod-IV notes, 100 KCAS | Re/ft reads 104800; the rest give 1047716 | Not used |
| Mod-III cruise power, 15,000 ft | Stated thrust and power disagree with Ct and Cp at 123.8 KCAS beta 26, 125.8 KCAS beta 30 and 38, by up to 4 % | Flagged; Phase 2's blade-element model decides |
| Rates, cruise, 8,000 ft, 110 KCAS, beta 35 | Cq holds 0.2654, ten times its definition, and Cp equals Ct | Don't trust that row's power |
| Rates, power off, alpha 12 | Amplitude reads `5,3` as text | Kept as text; the solvers may have used different amplitudes |
| Eigenvalue figures 18, 20, 23 | The imaginary axis is labelled Hz | It is rad/s: see `digitise_fq.py` |
| 20240006496, section on mass | Moments of inertia in lb/in2 | They are lb in2 |
| TM-20250001715, section 2.1 | Gives the P2006T wing's aspect ratio as 5.51 | Its table 1 gives 8.81, which matches b²/S |
| TM-20250001715 table 6 | The CFD's high-lift rpm is an older schedule than ANLYS-CEPT-023 appendix B (58 KCAS: 4702 against about 5020; it doesn't turn up above 90 KEAS) | The flight model flies the final schedule; the checks use the CFD's own rpm |
| 20240006496, propulsion | 127.5 Nm at 100 % reads as the motor's; it is one controller's, of two | 255 Nm a motor, 72 kW at 2700 rpm |
| Mod-IV, 90 KCAS, flaps 10, alpha 8, STAR-CCM+ | Stabilator -12 and -15 give nose-down moments and more lift, the wrong way | Left out of the checks |
| Power-off ground planes, STAR-CCM+, 7.87 ft, flaps 30 | Some rows read +0.65 in dCL, far from LAVA and the neighbouring heights | Left out of the checks and the fit |
| 20240006496 table 1, zero power and freeze | Its description says half the motor thrust; the row's name says zero | Built as zero |
| TM-20250001715 table 7 | Windmilling rpm of a failed high-lift disc is 7 to 16 % under what the blades do; the tunnel measured 2483 and 2654 rpm at 75 KEAS | The tunnel is the target. The checks print the difference from table 7 |

## Reading the CFD

- Power on minus power off, within one solver, is the only safe increment. STAR-CCM+ and USM3D ran Mod-III power on as
  half models, whose coefficients read half the aircraft's, and LAVA ran the whole aircraft. Their single-motor
  increments agree (median ratio 0.98).
- Mod-IV runs have no cruise propulsors, so no case has both motor systems on.
- Ground plane heights are feet below the wing's leading edge, not wheel heights.
- The release has no failure cases and no hinge moments. The TM only plots the three-motor failures (figs. 46-53).
- Negative sideslip appears only with a control deflected, so the release can't test the mirror symmetry directly.
- Past stall, 12 runs with symmetric inputs show rolling moment, up to Cl 0.47 at alpha 16, flaps 0: the CFD's wing stalling
  unevenly. A table can't hold that, and the model doesn't have it.

## Choices in the table model

- **Solvers.** All four averaged up to stall; past it, Kestrel and USM3D, the time-accurate two. With trim, this gives NASA's
  stall speeds without fitting anything.
- **Rate derivatives.** The wind tunnel's, not the CFD's, with the CFD's change between flap settings on top. NASA's modes
  fit the tunnel's values: with the CFD's, short period and Dutch roll come out about 20 % over-damped. That NASA's
  simulator used them is inferred from this; the TM doesn't say. The tunnel ran at a chord Reynolds number of 150,000,
  uncorrected for installation effects, so which is right for the real aircraft isn't known.
- **Gear drag.** NASA's value isn't published. 4 ft² is fitted to NASA's predicted phugoid, so that check is partly circular.
- **Trim tab.** Not modelled: pitch trim moves the stabilator. The tab's gearing isn't published, and it was run at flaps 0 only.

## Propulsion

`fdm/propulsion.py`, checked by `fdm/phase2_checks.py`. All of it is JSBSim XML.

- **Discs.** Fourteen, each with its own rpm, thrust, torque and in-plane force from blade-element theory on NASA's blades, with
  NeuralFoil polars. Not per blade.
- **Cruise propellers.** Theory, carried onto NASA's 37 points. The hub moves 1 deg/s on a worm gear and freezes without power.
- **High-lift propellers.** The tunnel's measurements over J 0.62-1.33, NASA's table outside them. So up to J 1.1 thrust sits 4 to 20 % under
  NASA's table, and above it, on small forces, the propeller unloads sooner. Blades fold forward spinning: an effective 20.0 deg.
- **Wing lift from the props.** Still the CFD's increments on J. A failed disc stops counting, and an uneven failure rolls the aircraft
  through those increments.
- **Motors.** NASA's lever map, torque limits and speed protection. A trim stands where the rotor would settle: at its command,
  or on the fine stop with the speed sagging where the torque is too light to hold it.
- **Battery.** Two packs of 128s20p with a Thevenin cell, Chin's, and NASA's efficiency chain. Each feeds one controller of each cruise motor
  and one pallet of high-lift motors, so a lost bus takes alternate discs, evenly.
- **Estimates, not published.** Rotor inertias, speed-loop gains, friction, regeneration torque, the cell's heat capacity. The cell's
  tables are read by eye from Chin's figure 8.
- **In-plane force.** Real, and it moves the modes a little further from NASA's: its simulator may not have it. A roll or yaw rate's
  crossflow at the discs isn't modelled.
- **Planning powers.** ANLYS-CEPT-018 doesn't give the configuration. The pattern's 60 kW fits clean and the climb's 120 kW gear down;
  the setup's 90 kW fits neither.

## The X-57's built-up model

`fdm/x57built.py`, on `fdm/builtup.py`: what ships, with the table model kept as its reference. Strips meet at each propeller's hub and edge, so a stopped disc takes its own lift away. `fdm/phase4_checks.py` tests it.

- **Fitted to the CFD.** Sections at flaps 0, 10 and 30, power off, so those curves are no test. A pitch moment past 8 deg on the body's incidence, and with the propellers on, on J: the CFD's pitch break comes at the same incidence on or off, though a blown section is short of stall. How much a jet
  speeds the wing, how much of the angle it keeps, how far the flap turns it, how much reaches the tail. The aileron's gains. The rate derivatives, carried from the strips' to the tunnel's, because NASA's modes follow the tunnel; the strips alone land on the CFD's.
- **Not fitted, so real tests.** Flaps 20 (CL within 0.055). Sideslip, stabilator and rudder, within 7-22 % RMS power off and 9-27 % power on. CL max at 58 KCAS, 4.52 against NASA's 4.40-4.58, comes from the jet factors, which were fitted lower down the curve.
- **Thrust.** The CFD's discs used NASA's table; the model's follow the tunnel, which is lower at the same rpm. Checks use the CFD's thrust. At NASA's final rpm schedule the model's thrust is the CFD's.
- **Misses.** Aileron pitch and rolling moment from sideslip with power on, on my own 25 % limit. Stall speeds 5.3 KCAS out against a limit of 5. Mod III's frequencies 8-10 %.

## Ground and failures

`fdm/phase5_checks.py` runs these against NASA's simulator and CFD, which are predictions: Mods II to IV never flew.

- **Ground effect.** The wing and tail each take a mirror image of the wake, unfitted. A pitch moment on wing height and incidence, fitted to the CFD's power-off ground planes, carries what that misses. With power on nothing is fitted.
- **Tyres.** NASA's eq. 4 from 20220005440, held past the 55 KIAS its fit reached. Grip 0.72 for asphalt. JSBSim's rolling friction is cut to 0.005 so as not to count rolling twice. The brakes are limited only by that grip, so full pedal stops shorter than the manual's roll.
- **Failures.** All 13 rows of NASA's table 1, under `ctl/`: `cmc-la` and `cmc-lb` for a controller, `motor-on-l`, `no-prop-forces-l`, `hub-fixed-l`, `feather-now-l`, `cmd-stuck-l`, `feather-l`, `hub-fine-l`, and `contactor-al`, `contactor-bl` for the high-lift motors. Right side likewise. A failed motor's hub goes on governing, so it drives toward windmilling.
- **Misses.** In the upset, pitch and sideslip: the table model misses the same way, so the built-up parts aren't the cause. Also Mod III's full-power climb, DiTTo with a motor out, minimum control speed 90.7 KCAS against 85 within 5, and ground-effect worst points. Details in the plan.

## Wing bending

`fdm/x57flex.py`, the ASK 21's method: each panel in its first bending mode, held at the H-frame, driven by the strips' forces and the panel's inertia in the moving airframe. Each strip rises with the mode and leans with its slope, and the airframe feels the panel's acceleration. `fdm/phase6_checks.py` tests it.

- **Fitted to NASA's vibration test of the wing.** Stiffness to symmetric first bending with the tip masses, 2.26 Hz, and without, 6.55 Hz. The root's give in roll to antisymmetric first bending, 2.14 Hz. Masses from the test's table 1.
- **Not fitted, so tests.** Clean antisymmetric 6.05 Hz against 5.76, clean second bending within 7 %, and NASA's whole-aircraft model 2.39 against 2.31 Hz, 2.37 against 2.29 for Mod IV. Second bending with the tip masses misses by 17 %: that mode has torsion in it, which a beam hasn't.
- **Free in the air the antisymmetric mode is 3.55 Hz.** The fuselage rolls under the heavy tips. So it has little air damping, 0.6-1.3 % of critical, and raw roll rate fed to the ailerons drives it unstable. Hands don't, but a roll-rate autopilot would.
- **The panels' coupling to the airframe is solved in the system**, through an effective mass for each kind of motion. Reading JSBSim's own accelerations gave wrong frequencies at 120 Hz and negative damping at every rate tried, up to 960 Hz. Solved, the results are the same at 120, 240 and 480 Hz, so 120 Hz is enough.
- **Rigid in the checks against CFD and NASA's modes**, which are rigid. `flex/enable` is 1 in the aircraft and in `fly.py`.
- **Assumed.** H-frame 30 lb, its outer points at 24 in; Mod IV's motors 16 lb with nothing for their nacelles; damping 0.57 % from the Mod II wing. No torsion, second bending or fore-aft, so nothing here says anything about flutter.

## The FlightGear aircraft

`fdm/mkfg.py` and `fdm/mkmodel.py` write `X-57/`. They need FGAddon's Tecnam P2006T unpacked in `refs/fgaddon-p2006t`. The Tecnam's parts in `X-57/Models` keep the GPL v2, in `COPYING-P2006T`.

The aerodynamics go out as a system, not as functions under `<aerodynamics>`. FlightGear reads every property under `/fdm` before the first frame, and a function read before its first frame works out everything it reads. With this graph that never finishes.

## The built-up method, on the P2006T

`fdm/builtup.py` builds an airplane from parts, each in its own air: wing strips in a pre-solved lifting line, discs with slipstreams, bodies by Munk and crossflow, a tail in the wing's downwash. It is the ASK 21's method with propellers added. It is proved on the P2006T,
because the Mods never flew: `fdm/genp2006t.py` builds it, `fdm/phase3_checks.py` tests it against the flight manual, the flight tests of Nicolosi, De Marco and Della Vecchia (AIAA 2010-7947) and the tunnel tests of Pascale and Nicolosi (ICAS 2008).

- **Geometry.** Fuselage and tail come from NASA's OpenVSP model: they are the P2006T's own. The wing and nacelles are measured off the flight manual's three-view. That gives 14.76 m2 and a mean chord of 1.340 m, the manual's and NASA's, without fitting.
- **Sections.** NACA 63A415 to 63A412 [ICAS], built from the listed 63A forms, by NeuralFoil. The tunnel build runs them at Re 0.6 million with trip strips.
- **What passes.** Every check on the flight manual: cruise, climb on one and two engines, stall. Phugoid, Dutch roll, neutral point, stabilator to trim, sideslip to 15 deg, Cm delta, Cm q, Cn r. Part-by-part tunnel results.
- **What misses, and why.** The short period's root and damping: the paper's 3.41 rad/s comes from a method that needs a true step, and the same method on this model with a pilot's 0.6 s step gives 3.69. CL alpha and Cm alpha: the ratio is right and the tunnel slope is right, but the flight's nose vane
  appears to read about 1.3 of true alpha, by the load per degree in the paper's fig. 24. That is inferred. NASA also replaced the published CL alpha with a lifting line's, which fitted its own flight data better. The tunnel's CY beta of -0.69: the model reads -0.48, and NASA's CFD on this fuselage and fin reads -0.50.
- **Fits.** On the P2006T's data: wing incidence, misc drag, sections' maximum lift, flap lift and moment, stabilator zero. On NASA's CFD of the same fuselage and tail, leaving the flight data independent: stabilator effectiveness, fin sidewash, rudder effectiveness, fin vortex lift. Propeller thrust is scaled to the flight tests' installed efficiency. The
  propeller is the X-57's MT blade, standing in. Take-off and landing distances aren't checked yet.

Slips found in the P2006T's sources:

| Where | What | What we do |
|---|---|---|
| AIAA 2010-7947 table 9 | Cm0 reads -0.922; the tunnel's is -0.08 | Taken as -0.0922 |
| AIAA 2010-7947, section IV.D | Cn beta 0.52; table 10 and the formula beside it need about 0.05 | 0.05 |
| AIAA 2010-7947 table 10 | Flight Cn beta 0.0135 gives about 1 rad/s, against the measured Dutch roll's 2.0. The authors call it unreliable | Tunnel and Dutch roll instead |
| AIAA 2010-7947 tables 7, 9 | Table 7's short period is 3.41 rad/s; table 9's derivatives give 4.95 | See above |
| AIAA 2010-7947 table 7 | A time constant of 0.0088 s: equation 1 leaves out the speed | Not used |
| AIAA 2010-7947 tables 9, fig. 25 | Tunnel CL0 reads 0.153; the figure's line reads about 0.08, as ICAS fig. 14 does | The figures |
| AIAA 2010-7947 fig. 12 | Called steady heading, but the bank stays near level: flat skids | Moments only |
| AIAA 2010-7947, EWADE 2011 | CY delta r is -0.04 in one, +0.04 in the other | Not limited |
| Flight manual, stall table | The heading says 1180 kg, the table 1230 kg | 1230 kg |
| Flight manual, control travel | Stabilator up 4, down 15: the flight data run to -11, trailing edge up | -15 to +4 |

## Sound

`X-57/Sounds` holds PyWopWop's bank, wired in by `fdm/x57sound.py`; `fdm/soundcheck.py` tests it on the generated XML.

- **SimGear's 128 sources.** A looped sound holds a source while its condition is true, however quiet. Loops are gated so
  that what can't be heard isn't true. Mod III's file has no high-lift loops, a stopped propeller reads 0 rpm, and every fade
  has a flat top, because the governor holds rpm on a grid point and the bank's neighbouring loops would otherwise stay
  true at next to nothing: 40 % of the sources held in flight were under -60 dB. Now none in the cockpit. At most 18 run at once
  in Mod III and 42 in Mod IV, 10 and 24 on average over a flight.
- **Altitude.** Fades run on true airspeed. Level goes as a power of the density, measured in PyWopWop by `fdm/soundalt.py`, at
  draft quality over four conditions. The high-lift propellers in the cockpit scatter about it by 1.5 dB at 8,000 ft.
- **Level.** A full-scale sample at volume 1 is 112.55 dB. Outside loops play at 0.405 to share that scale.
- **Not checked.** None of this has been heard against a recording. Motor levels are assumed. Rotation is NASA's: each wing's seven propellers turn the same way, inboard blades rising, the wings mirrored.

