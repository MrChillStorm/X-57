# X-57 cockpit logic: what lights, when, and which bus feeds what. The buses and contactors are the flight model's (fdm/x57power.py);
# this reads them, drives the panel's lamps and FlightGear's instruments, and turns the high-lift switches into the mode the flight model reads.
# Sources: ICD-CEPT-006 rev E (triggers: slides 36-39; high lift: 63-71), 20240006533, 20240009560 fig. 5.
#
# Simplified: a controller's fault lamp lights for a failure, or armed without traction voltage; not for its internal faults. The aural system is one tone
# standing in for six spoken messages. The pitot heat lamps stay dark: NASA's drawing has that breaker collared out.

var J = "/fdm/jsbsim/";
var X = "/sim/model/x57/";
var LAMP = "/instrumentation/x57/lamp/";
var ALIVE = 9;

var get = func(p, d = 0) { var v = getprop(p); return v == nil ? d : v; };
var sw = func(n) { return get(J ~ "ctl/sw/" ~ n) > 0.5; };
var now = func { return get("/sim/time/elapsed-sec"); };
var flash = func(on, off) { return math.mod(now(), on + off) < on; };

# lamps are lit by the Essential Bus, and bright enough for the time of day
var level = 350;
var lamp = func(name, on, test = 0) {
    setprop(LAMP ~ name, (on or test) ? level : 0);
};

# ---------------------------------------------------------------- Autostart and Autostop: NASA's sequences, a switch at a time [slides 43-49]
var C = J ~ "ctl/sw/";
var START = [[C ~ "ess-batt", 1, 1.0], [C ~ "dc-conv-a", 1, 0.8], [C ~ "dc-conv-b", 1, 0.8], [C ~ "cross-bus-a", 1, 0.6], [C ~ "cross-bus-b", 1, 0.6],
             [C ~ "wing-av-a", 1, 0.6], [C ~ "wing-av-b", 1, 0.6], [C ~ "av-bus-a", 1, 0.6], [C ~ "av-bus-b", 1, 0.6],
             [X ~ "dau-enable", 1, 0.5], [X ~ "rf-enable", 1, 0.5], [X ~ "record-enable", 1, 0.5],
             [J ~ "ctl/bus-a", 1, 1.0], [J ~ "ctl/bus-b", 1, 1.0],
             [C ~ "tc-la", 1, 0.8], [C ~ "tc-lb", 1, 0.8], [C ~ "tc-ra", 1, 0.8], [C ~ "tc-rb", 1, 0.8], [C ~ "hl-tc-a", 1, 0.8], [C ~ "hl-tc-b", 1, 0.8],
             [C ~ "arm-ra", 1, 2.0], [C ~ "arm-rb", 1, 2.0], [C ~ "arm-la", 1, 2.0], [C ~ "arm-lb", 1, 1.0]];
# the left motor stops first, for the way out through the door on that side
var STOP = [[C ~ "arm-lb", 0, 1.0], [C ~ "arm-la", 0, 2.0], [C ~ "arm-rb", 0, 1.0], [C ~ "arm-ra", 0, 2.0],
            [C ~ "hl-tc-b", 0, 0.6], [C ~ "hl-tc-a", 0, 0.6], [C ~ "tc-rb", 0, 0.6], [C ~ "tc-ra", 0, 0.6], [C ~ "tc-lb", 0, 0.6], [C ~ "tc-la", 0, 0.6],
            [J ~ "ctl/bus-b", 0, 0.8], [J ~ "ctl/bus-a", 0, 0.8], [X ~ "record-enable", 0, 0.5], [X ~ "rf-enable", 0, 0.5], [X ~ "dau-enable", 0, 0.5],
            [C ~ "av-bus-b", 0, 0.6], [C ~ "av-bus-a", 0, 0.6], [C ~ "wing-av-b", 0, 0.6], [C ~ "wing-av-a", 0, 0.6],
            [C ~ "cross-bus-b", 0, 0.6], [C ~ "cross-bus-a", 0, 0.6], [C ~ "dc-conv-b", 0, 0.6], [C ~ "dc-conv-a", 0, 0.6], [C ~ "ess-batt", 0, 0.5]];
var running = 0;

var sequence = func(steps, i, done) {
    if (i >= size(steps)) {
        running = 0;
        gui.popupTip(done);
        return;
    }
    setprop(steps[i][0], steps[i][1]);
    setprop(X ~ "click/rocker", !getprop(X ~ "click/rocker"));        # the switch's sound (Sounds/x57-*-sound.xml)
    settimer(func sequence(steps, i + 1, done), steps[i][2]);
};

var levers_idle = func {
    setprop("/controls/engines/engine[0]/throttle", 0);
    setprop("/controls/engines/engine[1]/throttle", 0);
    regen_set(0);
    setprop(X ~ "hl-arm", 0);
};

# ---------------------------------------------------------------- regeneration by FlightGear's reverser
# The real torque lever reaches regeneration past the idle detent, with a finger lift [20240006533]. FlightGear's throttle stops at idle, so its
# reverser stands in for the finger lift: on, the throttles run from idle at the top to full regeneration at the bottom, more as they come
# back, as the X-57's lever does below its detent (Systems/x57-rules.xml). Delete or
# Backspace toggles it, and so does a joystick's reverser button. FlightGear's own reverser-all hands its value to the engines as an axis
# would (on gives them 0, off 0.5), so it is read here and set right.
var REV = "/controls/engines/engine[%d]/reverser";
var regen_set = func(on) {
    foreach (var e; [0, 1]) setprop(sprintf(REV, e), on ? 1 : 0);
};
var regen_toggle = func { regen_set(!get(sprintf(REV, 0))); };
setlistener("/controls/engines/reverser-all", func(n) { regen_set(get("/controls/engines/reverser-all") > 0.5); }, 0, 0);
setlistener(sprintf(REV, 0), func(n) {
    gui.popupTip(n.getBoolValue() ? "Regeneration: the throttles now run from idle at the top to full regeneration at the bottom" : "Regeneration off: the throttles drive the cruise motors");
}, 0, 0);

# A Warthog throttle's idle switches (FlightGear's own binding, buttons 28 and 29) set an engine's cutoff while its lever is lifted past the
# idle detent: the X-57's finger lift. Where the lever crossed the detent is kept, and the travel below it runs to full regeneration at the
# stop (Systems/x57-rules.xml); held at the detent with no travel below it, the switch gives full regeneration.
var watch_detent = func(e) {
    setlistener("/controls/engines/engine[" ~ e ~ "]/cutoff", func(n) {
        if (n.getBoolValue()) setprop(X ~ "regen-detent[" ~ e ~ "]", get("/controls/engines/engine[" ~ e ~ "]/throttle"));
    }, 0, 0);
};
watch_detent(0);
watch_detent(1);

var autostart = func {
    if (running) return;
    running = 1;
    levers_idle();
    foreach (var e; [0, 1]) {
        if (get("/controls/engines/engine[" ~ e ~ "]/propeller-pitch") < 0.3) setprop("/controls/engines/engine[" ~ e ~ "]/propeller-pitch", 0.685);
        setprop(J ~ "ctl/pitch-man-" ~ ["l", "r"][e], 0);
    }
    sequence(START, 0, "X-57: started. Levers at idle, high lift disarmed");
};

var autostop = func {
    if (running) return;
    if (!get("/gear/gear[1]/wow")) {
        gui.popupTip("X-57: Autostop works on the ground only");
        return;
    }
    running = 1;
    levers_idle();
    sequence(STOP, 0, "X-57: shut down");
};

# ---------------------------------------------------------------- high lift [slides 63-71]
var hl_power_on = {a: nil, b: nil};
var hl_latch = 0;

var hl_state = func(b, arm) {
    var wing = get(J ~ "elec/wing-" ~ b ~ "-v") > ALIVE;
    if (!wing) { hl_power_on[b] = nil; return "off"; }
    if (hl_power_on[b] == nil) hl_power_on[b] = now();
    if (now() - hl_power_on[b] < 10) return "init";
    var hv = get(J ~ "elec/bus-" ~ b) > 0.5 and sw("hl-tc-" ~ b);
    var failed = get(J ~ "ctl/contactor-" ~ b ~ "l") < 0.5 or get(J ~ "ctl/contactor-" ~ b ~ "r") < 0.5;
    if (arm) {
        if (failed) return "armed-fault";
        if (!hv) return "hv-fault";
        var want = get(J ~ "prop/hl/rpm-cmd");
        var have = get(J ~ "prop/hl/rpm");
        if (abs(have - want) > 0.1 * math.max(want, 300)) return "spin";
        return get(X ~ "hl-airspeed") ? "airspeed-run" : "fixed-run";
    }
    if (get(J ~ "prop/hl/rpm") > 100) return "spin";
    if (!hv) return "waiting";
    return "ready";
};

var high_lift = func(test) {
    if (getprop(J ~ "prop/hl/rpm") == nil) return;     # Mod III has none: its panel stays dark
    if (get(X ~ "hl-yoke")) {
        hl_latch = 1;
        setprop(X ~ "hl-yoke", 0);
    }
    if (!get(X ~ "hl-arm")) hl_latch = 0;              # ARM to DISARM/RESET clears it
    var arm = get(X ~ "hl-arm") and !hl_latch;
    setprop(J ~ "ctl/hl-mode", arm ? 1 + (get(X ~ "hl-airspeed") ? 1 : 0) : 0);
    foreach (var b; ["a", "b"]) {
        var s = hl_state(b, arm);
        var blink = flash(0.5, 0.5);
        lamp("hl-rdy-" ~ b, s == "ready" or s == "airspeed-run" or ((s == "init" or s == "waiting") and blink), test);
        lamp("hl-run-" ~ b, s == "fixed-run" or s == "airspeed-run" or (s == "spin" and blink), test);
        lamp("hl-fault-" ~ b, s == "hv-fault" or (s == "armed-fault" and blink), test);
    }
    lamp("hl-reset", hl_latch, test);
    foreach (var k; ["la", "lb", "ra", "rb"]) {
        var b = substr(k, 1, 1);
        tc_led("hl-" ~ k, sw("hl-tc-" ~ b), get(J ~ "elec/bus-" ~ b) > 0.5 and get(J ~ "ctl/contactor-" ~ b ~ substr(k, 0, 1)) > 0.5);
    }
};

# a bicolor LED on the overhead: green engaged, red for a fault during engagement [20240006533]
var tc_led = func(name, on, hv) {
    lamp("tc-" ~ name, on);
    setprop(X ~ "tc-red/" ~ name, on and !hv);
};

# ---------------------------------------------------------------- the aural alerts: six channels, repeating every two seconds until acknowledged [slide 40]
var alerts = {batt: 0, conv: 0, trac: 0, cmc: 0, overspeed: 0, pitch: 0};
var acked = {batt: 0, conv: 0, trac: 0, cmc: 0, overspeed: 0, pitch: 0};
var last_tone = -9;

var tone = func {
    setprop(X ~ "alert-tone", 1);       # the sound file plays it, on the same scale as everything else
    settimer(func setprop(X ~ "alert-tone", 0), 0.5);
};

var aural = func(ess) {
    var ring = 0;
    foreach (var k; keys(alerts)) {
        if (!alerts[k]) acked[k] = 0;
        if (alerts[k] and get(X ~ "audio-ack")) acked[k] = 1;
        if (alerts[k] and !acked[k]) ring = 1;
    }
    if (get(X ~ "audio-check")) ring = 1;
    if (ess and ring and now() - last_tone >= 2) {
        last_tone = now();
        tone();
    }
};

# ---------------------------------------------------------------- the panel
var meter_v = func(k) {           # the energy meter's reading: downstream of the battery contactor and the controller's contactor
    var b = substr(k, 1, 1);
    return get(J ~ "elec/v28") > 20 ? get(J ~ "bat/" ~ b ~ "/volts") * get(J ~ "elec/bus-" ~ b) * (sw("tc-" ~ k) ? 1 : 0) : 0;
};

var update = func {
    var g = get("/gear/gear[0]/position-norm");        # for the sounds: a lock clicks only on a gear that has moved
    if (g > 0.01 and g < 0.99) setprop(X ~ "gear-moved", 1);
    var e = get(J ~ "elec/ess-v");
    var ess = e > ALIVE;
    level = ess ? 0.6 * get("/instrumentation/x57/day-nits", 2500) : 0;
    var test = get(X ~ "light-check") and ess;
    var soc_low = e > ALIVE and get(J ~ "elec/ess-batt-v") < 12.8;
    var fail = 0;

    # the annunciator [slides 37-39]
    foreach (var b; ["a", "b"]) {
        var B = string.uc(b);
        var conv = sw("dc-conv-" ~ b) and (get(J ~ "elec/conv-" ~ b ~ "-low") > 0.5 or get(J ~ "ctl/fail/conv-" ~ b) > 0.5);
        lamp("dc-conv-" ~ b, conv, test);
        var batt = get(J ~ "bat/" ~ b ~ "/temp-c") > 65;
        lamp("main-batt-" ~ b, batt, test);
        var trac = meter_v("l" ~ b) <= 350 and get(J ~ "elec/v28") > 20;
        lamp("trac-bus-" ~ b, trac, test);
        if (b == "a") { alerts.conv = conv; alerts.batt = batt; alerts.trac = trac; }
        else { alerts.conv = alerts.conv or conv; alerts.batt = alerts.batt or batt; alerts.trac = alerts.trac or trac; }
    }
    var cmc = 0;
    foreach (var k; ["la", "lb", "ra", "rb"]) {
        var b = substr(k, 1, 1);
        var armed = sw("arm-" ~ k);
        var failed = get(J ~ "ctl/cmc-" ~ k) < 0.5 or get(J ~ "ctl/motor-on-" ~ substr(k, 0, 1)) < 0.5;
        var f = failed or (armed and meter_v(k) < 1);
        lamp("cmc-" ~ k, f, test);
        cmc = cmc or f;
        tc_led(k, sw("tc-" ~ k), get(J ~ "elec/bus-" ~ b) > 0.5);
    }
    alerts.cmc = cmc;
    lamp("main-door", get("/sim/model/door-left") > 0.01, test);
    lamp("rear-door", get("/sim/model/door-right") > 0.01, test);
    lamp("pitot-on", 0, test);
    lamp("pitot-fail", 0, test);
    var pos = get(J ~ "gear/gear-pos-norm");
    var moving = pos > 0.01 and pos < 0.99;
    lamp("gear-pump", moving, test);
    # steady for a short, slow flash for over-discharge [slide 39]
    lamp("ess-batt", get(J ~ "ctl/fail/ess-batt") > 0.5 or (soc_low and flash(5, 5)), test);

    # the landing gear's lights, and their push-to-test
    var lt = get(X ~ "lt-test") and ess;
    foreach (var n; ["ldg-nose", "ldg-lh", "ldg-rh"]) lamp(n, pos > 0.99, lt);
    lamp("ldg-trans", moving, lt);

    # propeller pitch: START at the end of travel, FEATHER is the command, FAULT the hub [slide 29]
    var pitch = 0;
    foreach (var s; ["l", "r"]) {
        lamp("start-" ~ s, get(J ~ "prop/" ~ s ~ "/pitch-deg") < 14.5);
        lamp("feather-" ~ s, get(J ~ "ctl/feather-" ~ s) > 0.5);
        var f = get(J ~ "ctl/hub-fixed-" ~ s) > 0.5 or get(J ~ "ctl/hub-fine-" ~ s) > 0.5 or get(J ~ "ctl/feather-now-" ~ s) > 0.5 or get(J ~ "ctl/cmd-stuck-" ~ s) > 0.5;
        lamp("fault-" ~ s, f);
        pitch = pitch or f;
    }
    alerts.pitch = pitch;
    alerts.overspeed = get(J ~ "prop/l/rpm") > 3080 or get(J ~ "prop/r/rpm") > 3080;

    # the D175's ten CAN lights [slide 53]
    var wa = get(J ~ "elec/wing-a-v") > ALIVE;
    var wb = get(J ~ "elec/wing-b-v") > ALIVE;
    var daq = get(J ~ "elec/av-b-v") > ALIVE and get(X ~ "dau-enable");
    var can = [daq, wa and sw("tc-la"), wb and sw("tc-lb"), ess, ess, ess, ess, wa and sw("tc-ra"), wb and sw("tc-rb"), daq];
    forindex (var i; can) lamp("can-" ~ i, can[i]);

    # the data system [slide 22]
    var dau = get(X ~ "dau-enable") and get(J ~ "elec/av-b-v") > ALIVE;
    lamp("time-lock", dau);
    var rec = get(X ~ "record-enable");
    setprop(X ~ "rec-colour", !dau ? (rec ? 3 : 0) : (rec ? 2 : 1));     # green ready, red archiving, amber a fault
    lamp("rec", get(X ~ "rec-colour") > 0);

    high_lift(test);
    aural(ess);

    # FlightGear's instruments on their buses
    var a = get(J ~ "elec/dc-a-v");
    var out = "/systems/electrical/outputs/";
    setprop("/systems/electrical/volts", e);
    setprop(out ~ "turn-coordinator", a);
    setprop(out ~ "comm[0]", e);
    setprop(out ~ "comm[1]", e);
    setprop(out ~ "nav[0]", e);
    setprop(out ~ "nav[1]", e);
    setprop(out ~ "transponder", get(J ~ "elec/av-a-v"));
    # the attitude and heading gyros are electric here, on DC Converter Bus A: FlightGear's vacuum pump turns with their power
    var gyro = a > ALIVE ? 2400 : 0;
    setprop("/engines/engine[0]/rpm", gyro);
    setprop("/engines/engine[1]/rpm", gyro);
};

# The instruments at load. Before the aircraft is placed, FlightGear's outside air reads about 42 inHg; then it jumps to the
# real value, and what follows it wound visibly for the first seconds of every flight:
# - the static system follows the outside air with a 1 s lag, so the altimeter climbed from -3300 ft for some 5 s: until 2 s
#   after the flight model starts, the static pressure is the outside air's;
# - the generic VSI integrates that pressure: it read -35 000 fpm, then wound back at 1000 fpm a second for half a minute,
#   and parked it showed the weather's pressure ramp (-63 fpm). Here: a conventional VSI's lag on the flight model's vertical
#   speed, from 0. A VSI takes 6 to 9 s to show a new rate (FAA's Pilot's Handbook): a time constant of 2.5 s;
# - the generic turn coordinator's needle drifted to -6 standard rates before power came and took over 10 s back. Here: the
#   yaw rate in standard rates (3 deg/s), as FlightGear's, its gyro spun up from the start (the aircraft loads powered), then
#   spinning up and down with Avionics Bus A's power, and the needle damped.
var VSI_TAU = 2.5;
var STATIC_HOLD = 2.0;
var TC_TAU = 0.5;
var TC_SPIN_UP = 5.0;
var TC_RUN_DOWN = 30.0;
var t_init = nil;
var vsi = 0;
var tc = 0;
var tc_spin = 1;
var lag = func(x, target, dt, tau) { return x + (target - x) * (1 - math.exp(-dt / tau)); };
var instruments = func {
    if (t_init == nil or now() - t_init < STATIC_HOLD)
        setprop("/systems/static/pressure-inhg", get("/environment/pressure-inhg", 29.92));
    var dt = get("/sim/time/delta-sec");
    if (dt <= 0)
        return;
    vsi = lag(vsi, get("/velocities/vertical-speed-fps") * 60, dt, VSI_TAU);
    setprop("/instrumentation/x57/vsi-fpm", vsi);
    var powered = t_init == nil or get("/systems/electrical/outputs/turn-coordinator") > ALIVE;
    tc_spin = lag(tc_spin, powered ? 1 : 0, dt, powered ? TC_SPIN_UP : TC_RUN_DOWN);
    var rate = get("/orientation/yaw-rate-degps") / 3;
    rate = rate > 2.5 ? 2.5 : (rate < -2.5 ? -2.5 : rate);
    tc = lag(tc, rate * (1 - math.pow(1 - tc_spin, 4)), dt, TC_TAU);
    setprop("/instrumentation/x57/turn-rate", tc);
};
var instruments_timer = maketimer(0, instruments);       # every frame: the needles move smoothly
instruments_timer.start();

# A runway start puts the flaps at takeoff, 10 deg [ICD slide 58: TO 10], already there: on the ground with no parking position named.
# The flight model keeps the flap motor's position in a property for this; set at a parking position or in the air, they start up
var runway_start = func {
    if (!getprop("/sim/presets/onground")) return;
    var park = getprop("/sim/presets/parkpos");
    if (park != nil and park != "") return;
    setprop("/controls/flight/flaps", 10 / 30);
    setprop(J ~ "ctl/flap-moving", 10);
};
setlistener("/sim/signals/fdm-initialized", func(n) { if (n.getBoolValue()) runway_start(); }, 0, 0);

var timer = nil;
var ready_l = setlistener("/sim/signals/fdm-initialized", func(n) {
    if (n.getBoolValue()) {
        removelistener(ready_l);
        t_init = now();
        setprop(J ~ "ctl/cockpit", 1);
        timer = maketimer(0.2, update);        # kept: an unreferenced timer is collected
        timer.start();
    }
}, 1, 0);

# The flap switch [ICD slide 58]: Mods III and IV run the flaps in "toggle" mode: held down they run down, held up they run up, and they stop
# where the switch is let go, read on the gauge. The motor follows /controls/flight/flaps (x57-controls: 3 deg/s), so holding sets that to the
# end it runs toward and letting go sets it to where the flaps stand. The wheel nudges them 2.5 deg; [ and ] step 0, 10, 20 and 30 from
# wherever they stand.
var flap_switch = func(d) {
    setprop(X ~ "flap-switch", d);
    setprop("/controls/flight/flaps", d > 0 ? 1 : d < 0 ? 0 : get(J ~ "ctl/flap-deg") / 30);
};
var flap_nudge = func(d) {
    setprop("/controls/flight/flaps", math.min(1, math.max(0, get("/controls/flight/flaps") + d / 12)));
};
controls.flapsDown = func(step) {
    if (step == 0) return;
    var f = get("/controls/flight/flaps");
    var v = [];
    foreach (var n; props.globals.getNode("/sim/flaps").getChildren("setting")) append(v, n.getValue());
    var t = f;
    if (step > 0) {
        foreach (var x; v) if (x > f + 0.01) { t = x; break; }
    } else {
        for (var k = size(v) - 1; k >= 0; k -= 1) if (v[k] < f - 0.01) { t = v[k]; break; }
    }
    setprop("/controls/flight/flaps", t);
};
