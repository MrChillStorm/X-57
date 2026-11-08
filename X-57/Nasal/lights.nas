# The exterior lights: each needs its switch and DC Converter Bus B, which carries all four in NASA's power drawings. The position lights' halos show in
# poor light, as the Bo 105's; the strobes flash whenever they're on.
var X = "/sim/model/x57/lighting/";
var C = "/controls/lighting/";
var strobes = props.globals.initNode(X ~ "strobes", 0, "BOOL");
aircraft.light.new(X ~ "strobe-left", [0.05, 1.00], strobes);
aircraft.light.new(X ~ "strobe-right", [0.05, 1.00], strobes);
aircraft.light.new(X ~ "strobe-tail", [0.05, 1.03], strobes);

var update = func {
    var bus = (getprop("/fdm/jsbsim/elec/dc-b-v") or 0) > 10;
    var dim = (getprop("/environment/visibility-m") or 20000) < 5000 or (getprop("/sim/time/sun-angle-rad") or 0) > 1.4;
    strobes.setBoolValue(bus and getprop(C ~ "strobe"));
    setprop(X ~ "nav", bus and getprop(C ~ "nav-lights") and dim);
    setprop(X ~ "landing", bus and getprop(C ~ "landing-lights"));
    setprop(X ~ "taxi", bus and getprop(C ~ "taxi-lights"));
};
var timer = maketimer(0.5, update);
timer.start();
