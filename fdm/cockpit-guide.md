# X-57 cockpit guide

How to work the X-57's cockpit in FlightGear: what each control does, how to start and stop, and how far the simulation goes.

NASA never flew the X-57, and Mod IV never existed as a whole aircraft. So this guide comes from NASA's Cockpit Interface Control Document (ICD-CEPT-006 rev E), NASA's papers and the Tecnam P2006T flight manual. NASA's own Pilot's Operations Handbook isn't public. The Mod IV parts are NASA's design, not a record of a working aircraft.

## Quick start

- **X-57 menu > Autostart** runs NASA's start-up order, one switch at a time, in about 25 seconds.
- **X-57 menu > Autostop** runs the shutdown, on the ground only.

The aircraft starts with everything on and the motors idling, so you only need these after shutting down.

Mod III carries the high-lift motors on its wing, but they're inactive with the blades folded, so its high-lift panel stays dark and its switches do nothing.

| Key | Action |
|---|---|
| Page Up / Page Down | Torque levers |
| Delete or Backspace | Regeneration: the throttles then run from idle at the top to full regeneration at the bottom. Again to go back |
| G | Landing gear |
| V | Cycle the views, among them the Battery View (the aft cabin's batteries, their control modules and vent) and, after it, the Pilot Face View (the pilot's face, headset and sunglasses, from over the yoke) |
| [ and ] | Flaps |
| H | High-lift ARM or DISARM/RESET |
| Y | Yoke disarm (or click the red button on the yoke) |
| Click a door | Opens or shuts it, from inside or out: the main door on the left, the rear door on the right. On the ground only, below 19 kt. Hover over a door to see whether it is open and whether a click will work. Each door clicks open and thumps shut, and while one is open the cockpit hears the outside |

Torque, propeller and other lever bindings are FlightGear's usual throttle and propeller controls. The propeller lever is FlightGear's propeller pitch axis.

## Start-up

Everything is on the panel. Each low voltage switch goes up, working from left to right.

| Step | Control | What happens |
|---|---|---|
| 1 | `ESS BATT` | Essential battery powers the Essential Bus. Annunciator, D175, G-meter and radios come on |
| 2, 3 | `DC CONV A`, `B` | Each 13.8 V converter comes online. Needs the Essential Bus |
| 4, 5 | `CROSS BUS A`, `B` | Converters feed the Essential Bus, share the load and charge the battery |
| 6, 7 | `WING AVIONICS A`, `B` | 23 V to the motor controllers and high-lift controllers |
| 8, 9 | `AVIONICS BUS A`, `B` | Transponder and data system |
| | `DAU ENABLE`, `RF ENABLE`, `RECORD` | Data system, if wanted |
| 10, 11 | `MAIN BATT A`, `B` (overhead) | Traction battery contactors close |
| 12 | `LA`, `LB`, `RA`, `RB` (overhead) | High voltage to each controller. Wait for each LED to turn green |
| | `HIGH-LIFT BUS A`, `B` (overhead, Mod IV) | High voltage to the high-lift controllers. H-LA to H-RB turn green |
| 13 | `RIGHT CMC A`, `B`, then `LEFT CMC A`, `B` | Arm the controllers: the motor turns at idle. The right side goes first, to keep the left door clear |

Before starting a motor the blades must be at START, because AUTO only works above 200 rpm. If the green START light is out, set MAN and hold START.

## Shutdown

Normal shutdown is the reverse: torque levers to idle, disarm left then right, open the contactors, then `MAIN BATT`. Then work the low voltage switches from right to left, with `ESS BATT` last.

**Emergency shutdown:** torque levers to idle, propeller levers to feather, `MAIN BATT A` and `B` off. Then `CROSS BUS A` and `B`, `DC CONV A` and `B`, and `ESS BATT` as soon as practical.

## Takeoff: DiTTo

Distributed Thrust Takeoff is a technique for the pilot, not software. Nothing automatic mixes the motors or reacts to the rudder and ailerons. Holding the wingtip motors to about two thirds power keeps a motor failure controllable, and the high-lift motors make up the thrust.

1. During the run-up at raised torque, set the propeller levers to 1,800 rpm and leave them there.
2. Line up, select `FIXED` and move to `ARM`. All twelve high-lift motors spin up to 4,800 rpm. RDY goes out and RUN turns green.
3. Push the torque levers to the `FULL` detent, the top of the throttle unless Overdrive is ticked. That's 255 Nm and about 48 kW per motor, 67 % of rated power.
4. Rotate at 77 KIAS and climb at 84 KIAS.
5. At a safe height, accelerate to best rate of climb, then raise gear and flaps and disarm the high-lift motors.
6. Cruise climb at 2,250 rpm and 255 Nm, about 60 kW per motor.

| Speed (KIAS) | Mod II | Mod III | Mod IV with DiTTo |
|---|---|---|---|
| Stall, Vs1 | 60 | 80 | 70 |
| Rotate | | 88 | 77 |
| Initial climb, Vy | 72 | 96 | 84 |
| Minimum control, Vmca | 60 | 86 | 73 |

If a cruise motor fails after takeoff, the aircraft rolls and yaws about three times as hard as a Mod II. Fly it with aileron and rudder, and pull the torque levers back if you need to cut the asymmetry.

## Levers

| Lever | Position | Effect |
|---|---|---|
| Torque | `REGEN` | Below idle, past a detent. Negative torque: the cruise propellers windmill, adding drag and recharging the battery |
| | `IDLE` | 8 Nm per controller |
| | `FULL` | 127.5 Nm per controller, 255 Nm per motor |
| | `OVERDRIVE` | Up to 175 Nm, for one controller left. Past a detent |
| Propeller | Forward | 2,700 rpm |
| | Cruise | 2,250 rpm |
| | Low | 1,700 rpm |
| | Aft stop | `FEATHER`: drives the blades to feather, overriding MAN and AUTO |
| Parking brake | Out | Set: holds the brakes on. A round knob at the foot of the pedestal, as on the P2006T: click it, or press `B`. Set when the aircraft loads. The brakes, set or pedalled, hold the aircraft to about half torque at 2700 rpm: more and it slides |
| | In | Released |

FlightGear's throttle stops at idle and full. For regeneration press Delete (Backspace on a Mac): the throttles then run from idle at the top to full regeneration at the bottom, more as they come back, and the levers on the pedestal move back past idle with them. Switched on with the throttles at idle, it starts at full: push them up to ease it off. A joystick's reverser button does the same. On a Thrustmaster Warthog throttle, whose axis reads nothing past the idle detent, bind the idle-detent button to Reverser Toggle: each pass over the detent switches regeneration on or off, and the levers set how much. With FlightGear's own Warthog binding that button gives the lever's motor full regeneration while it is held past the detent. Overdrive, and regeneration in the throttle's bottom tenth, are ticked in **X-57 menu > Systems and failures**.

## Left panel

| Control | What it does |
|---|---|
| Annunciator | See below |
| `AUDIO ANN. ACK` | Silences the current aural alerts |
| `AUDIO CHECK` | Tests the aural alert while held |
| `LIGHT CHECK` | Tests the annunciator lights while held |
| Airspeed, attitude, altimeter, turn, heading, climb | Stock P2006T. Attitude, heading and turn are on DC Converter Bus A |
| UHF radio | COM 1, on the Essential Bus |
| G-meter (GT-50) | `Mode` steps through g, Essential Bus volts, timer, flight timer and clock. `Action` shows maximum and minimum g, or starts the timer. `Reset` clears them |
| Trim indicators | Stabilator and rudder |

### High-lift panel (Mod IV)

| Control | What it does |
|---|---|
| `ARM`, `DISARM/RESET` | Arms or disarms all twelve motors. Moving to `DISARM/RESET` also clears faults and a yoke disarm |
| `AIRSPEED`, `FIXED` | Motor speed from the airspeed schedule, or fixed. You can change it without disarming |
| `RESET REQ'D` (yellow) | The yoke switch was used. Move ARM to DISARM/RESET |
| `RDY` (blue) | Steady: ready. Flashing: starting up or waiting for high voltage |
| `RUN` (green) | Steady: running. Flashing: spinning up or down |
| `FAULT` (red) | Steady: armed with no high voltage. Flashing: fault while armed |

**Landing:** select `AIRSPEED` while above 120 KEAS, and `ARM` between 120 and 95 KEAS. Below 95 KEAS the motors' torque rises from 3 Nm towards 22 Nm at 58 KEAS. Disarm above 120 KEAS and the blades fold.

If high-lift motors fail on one side, press the yoke's disarm button to remove the asymmetry, as NASA's pilots did in the simulator.

### Annunciator

| Top row | Bottom row | Colour | Lights when |
|---|---|---|---|
| `DC CONV A` | `DC CONV B` | Yellow | Converter failed, or traction battery below 310 V |
| `MAIN BATT A` | `MAIN BATT B` | Red | Cell above 65 °C |
| `TRAC BUS A` | `TRAC BUS B` | Yellow | Traction bus at or below 350 V |
| `LEFT CMC A` | `LEFT CMC B` | Yellow | Controller failed, or armed with no traction voltage |
| `RIGHT CMC A` | `RIGHT CMC B` | Yellow | The same |
| `MAIN DOOR OPEN` | `REAR DOOR OPEN` | Red | Door open |
| `PITOT HEAT` (green) | `PITOT HEAT` (yellow) | | Not modelled |
| `GEAR PUMP ON` | | Yellow | Gear moving |
| | `ESS BATT` | Red | Battery failed (steady), or below 12.8 V (slow flash) |

Red means act now. Yellow means caution, and return to base if it can't be cleared.

## Centre stack

| Control | What it does |
|---|---|
| Crew panel (Mod IV) | Blue, green and red lights for each of the twelve high-lift motors. The right switch turns the display on. The left switch is a continuity check, with no effect here |
| Navigator | Shows COM 2 and NAV 1 frequencies only |
| Transponder | Shows the code |
| Page knob | D175 page. 1 is the pilot's page, the others are secondary |
| `FAULT`/`MAIN` | Stays at `MAIN`. It no longer does anything on the aircraft either |
| `START` (green) | Blades at the fine stop. On the aircraft it means the end of travel |
| `FEATHER` (blue) | Feather is commanded, not that the blades have arrived |
| `LH FAULT`, `RH FAULT` (yellow) | Propeller hub fault |
| D175 | See below |
| Audio panel | Not modelled |

### D175 pages

| Page | Shows |
|---|---|
| 1 | Torque lever %, estimated torque, motor power, battery volts, state of charge |
| 2 | Controller temperatures (dashes: not modelled) |
| 3 | Motor temperatures (dashes) |
| 4 | Software versions and propeller pitch |
| 5 to 7 | Windings and nacelle temperatures (dashes) |

The ten lights along the top show CAN bus traffic. From left: left data acquisition, left CMC A and B, left BMS, left torque encoder, right torque encoder, right BMS, right CMC A and B, right data acquisition. NASA treats the D175 as test information only: trust the energy meters for battery state, because NASA's state of charge drifts.

## Right panel

| Control | What it does |
|---|---|
| Tachometers | Propeller rpm. Digits from 300 rpm, bar green from 500, red above 3,080. Left on Bus A, right on Bus B |
| Traction bus meters | Volts, amps and power for each controller. They autoscale: past 1,000 W the power reads in kW with only a small mark at the left, as on the aircraft |
| Avionics meters | Volts and amps of DC converter A and B. Bar red outside 13 to 14.6 V or 0 to 70 A |
| `MAN`/`AUTO` | `AUTO`: the propeller lever sets rpm, from 1,700 to 2,700. `MAN`: blades move only while you hold `START` or `FEATHER` |
| `START`, `FEATHER` tags | Click and hold. `START` drives to fine pitch, `FEATHER` to coarse. In MAN only |
| OAT | Outside air temperature, °C |

## Lower panels

| Control | What it does |
|---|---|
| Gear lever | Up or down. Gear runs on the Essential Bus. On the ground the gear stays down whatever the lever says, and comes up as soon as the wheels leave the ground if the lever is up. That guard is a simulator choice: sources disagree on whether the real aircraft has a squat switch |
| `NOSE`, `LH`, `RH` (green) | Gear down and locked |
| `TRANS` (red) | Gear moving |
| `LT TEST` | Tests the gear lights while held |
| Flap switch and indicator | As on NASA's Mod III and IV, the flaps run while the switch is held and stop where it's let go: press and hold just above the lever to raise them, just below it to lower them, and the lever springs back to the middle. They move 3 degrees a second. The scroll wheel over the lever nudges them 2.5 degrees; `[` and `]` step 0, 10, 20 and 30 from wherever they stand. The dial reads in degrees: NASA takes off with 10 and lands with 30. Started on a runway, they're already at 10; at a parking position or in the air, up. NASA's data has flaps 0, 10 and 30, so in between the model interpolates |
| `LANDING`, `TAXI`, `NAV`, `STROBE` | The exterior lights, all on DC Converter Bus B in NASA's power drawings: 3 A breakers, 7.5 A for the strobe. Those drawings collar out landing, taxi and nav, the one with the Mod IV additions included, so only the strobe was to be live. Here all four work. They don't say where the lights sit: the tip nacelles, the tail and the fin are this model's choices. Landing and taxi are in the left wing's leading edge, where NASA's photographs show them on Mod II's stock wing. No source shows them on this wing, so they keep the same distance from the centreline |
| `INSTR` (fitted only) | Not on the X-57: X-57 menu > Tecnam cockpit lights fits the P2006T's. The fifth rocker in the LIGHTS row, as on the P2006T: the instrument lights, on DC Converter Bus B through the `INSTR LIGHT` breaker, which NASA's drawings collar out. Three dimmers appear under the `EXPERIMENTAL` placard: switch lights, avionics key lights, and the two ceiling cockpit lights over the left and right panels. The cabin light and the emergency light are in the ceiling, each with its switch on the lamp. Unfitted, none of it is drawn |
| `CABIN FAN` | No effect |
| `DAU ENABLE` | Data system. Green Time Lock light |
| `RF ENABLE` | Telemetry. No effect here |
| `RECORD` | Light: green ready, red archiving, amber fault |

## Breaker panels

Click a breaker to pull it. Four do something: `LANDING GEAR`, `FLAP MOTOR` and `PROP CONT. L` and `R`. NASA's emergency stop for a runaway propeller is to pull `PROP CONT`, which freezes that hub. The left panel is from NASA's photograph. Both sit in cream mouldings between the instrument panel and the door frames, as in NASA's cockpit photograph. The cabin as modelled has about half the photograph's room there, so the plates are a little smaller than NASA's. The right panel's names are from NASA's power drawing, but their order is a guess.

## Power system

| Bus | Fed by | Carries |
|---|---|---|
| Essential Bus | Essential battery, cross buses | Gear, flaps, propeller control, annunciator, D175, G-meter, radios, contactor power |
| DC Converter Bus A | Converter A | Attitude, turn and heading, left tachometer |
| DC Converter Bus B | Converter B | Right tachometer, rudder trim, strobe, cabin fan |
| Wing Avionics A, B | Essential Bus at 23 V | Controllers LA, RA and high-lift A; LB, RB and high-lift B |
| Avionics Bus A | DC Converter Bus A | Transponder |
| Avionics Bus B | DC Converter Bus B | Instrumentation |

The Essential Bus is the last one to lose: with it dead the contactors open and all traction goes. To shed load, turn off `AVIONICS BUS A` and `B` first.

## Pilot and rudder pedals

The rudder pedals are the pilot's only: NASA removed the copilot's with his seat and yoke. They swing with the rudder, and their toe brakes tilt with the brakes. Their look follows NASA's photograph; their size and travel are estimates.

Outside the Cockpit View a pilot sits in the left seat. His left hand rides the yoke, his right hand the torque levers and his feet the pedals; his arms and legs follow them. He is 1.75 m tall, a median man; the seat is 8 cm lower than the P2006T model had it, as the real seat adjusts, so his head clears the roof lining by about 3 cm, and the rudder pedals sit far enough forward for his knees to clear the yoke. He wears the P2006T's three-point seatbelt (a lap belt and a shoulder strap to a buckle at his right hip) and black leather work shoes with two hook-and-loop straps, scanned from a real pair. He is built from MakeHuman's CC0 body and assets; his flight suit, headset, belt, shoes and gold-framed aviator sunglasses with green lenses, of the US military pattern, are this aircraft's own.

## What the simulation leaves out

- The power system runs at fixed voltages, with no sag. Bus loads and the essential battery's 20 Ah are estimates.
- Motor and controller temperatures aren't modelled. Battery temperature is.
- NASA's controllers limit how fast torque changes, but the limit isn't published. The sim takes one second for the full range, which is an estimate. The rotors' inertias are estimates too, 0.65 kg m² for a cruise rotor and 0.0085 for a high-lift one: neither is published.
- Aural alerts are one tone in place of NASA's six spoken messages, whose wording isn't published.
- The gear pump, flap motor, gear locks and switch clicks are stock samples, at levels that are estimates. The propellers, motors, wind, the gear in the airflow, the tyres rolling and each wheel's touchdown are computed. Nobody has heard these aircraft: only the tyres' cockpit levels come from a recording, of a landing in a P2006T's cabin, and the nose tyre's squeal from a glider's. A touchdown's squeal follows the sink rate as that wheel's weight came on and its slip, the ground speed less the speed the wheel was still turning at: a gentle one is long, a firm one short, and a wheel back down after a hop is quiet. Each strut thumps by its sink rate.
- Controller lights cover failures and arming without traction voltage, not internal faults.
- The cabin fan and most breakers have no effect.
- FlightGear's controls can't lock a switch, so the lever locks aren't modelled.
- Mod III shares this cockpit, so its high-lift controls are there but do nothing.

## Sources

- ICD-CEPT-006 rev E, X-57 Cockpit Interface Control Document: ntrs.nasa.gov/citations/20230016539
- Power System Human Machine Interfaces for the X-57 Mod II Aircraft: ntrs.nasa.gov/citations/20240006533
- X-57 Traction Power and Command Systems Development: NTRS 20240009560
- X-57 Cockpit Display System Development and Features: NTRS 20230005312
- Distributed Thrust Takeoff for the NASA X-57 Mod IV Flight Demonstrator: NTRS 20240010539
- X-57 Flight Controls Lessons Learned: ntrs.nasa.gov/citations/20240014847
- Tecnam P2006T Aircraft Flight Manual, Doc. 2006/044
