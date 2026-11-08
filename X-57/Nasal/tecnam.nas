# The Tecnam's cockpit lights, written by fdm/x57tecnam.py (see there). The X-57 has none: X-57 menu > Tecnam cockpit lights
# fits them. Unfitted, every level stays 0 and nothing of them is drawn. Instrument lights: the INSTR rocker, on DC Converter
# Bus B through the INSTR LIGHT breaker; each part on its dimmer. Cabin light: its switch, the Essential Bus. Emergency light:
# its switch, on its lamp, the Essential battery itself. Levels 0-1 in LV for the HDR lights, cd/m2 for what glows.
var LV = "/instrumentation/x57/tecnam/";
var C = "/controls/lighting/";
var FIT = "/sim/model/x57/tecnam-lights";
var CB = "/sim/model/x57/cb/r-2-4";
var INTEGRAL = 1.9099;
var COCKPIT_NITS = 16696.3;
var CABIN_NITS = 3246.8;
var EMERG_NITS = 55262.1;
var clamp01 = func(v) { v == nil ? 0 : (v < 0 ? 0 : (v > 1 ? 1 : v)); };
var volts = func(n) { (getprop("/fdm/jsbsim/elec/" ~ n) or 0) > 10; };

var update = func {
    var fit = getprop(FIT) or 0;
    var instr = fit and volts("dc-b-v") and !(getprop(CB) or 0) and getprop(C ~ "instr-light");
    var dim = func(n) { instr ? clamp01(getprop(C ~ n ~ "-norm")) : 0; };
    setprop(LV ~ "switches-nits", dim("switches") * INTEGRAL);
    setprop(LV ~ "avionics-nits", dim("avionics") * INTEGRAL);
    var c = dim("cockpit");
    setprop(LV ~ "cockpit", c);
    setprop(LV ~ "cockpit-nits", c * COCKPIT_NITS);
    var cabin = (fit and volts("ess-bus-v") and getprop(C ~ "cabin-light")) ? 1 : 0;
    setprop(LV ~ "cabin", cabin);
    setprop(LV ~ "cabin-nits", cabin * CABIN_NITS);
    var em = (fit and volts("ess-batt-v") and getprop(C ~ "emergency-light")) ? 1 : 0;
    setprop(LV ~ "emergency", em);
    setprop(LV ~ "emergency-nits", em * EMERG_NITS);
};
foreach (var p; [FIT, CB, C ~ "instr-light", C ~ "switches-norm", C ~ "avionics-norm", C ~ "cockpit-norm", C ~ "cabin-light", C ~ "emergency-light"])
    setlistener(p, func { update(); }, 0, 0);
var timer = maketimer(0.5, update);      # the buses
timer.start();
