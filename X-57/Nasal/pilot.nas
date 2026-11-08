# The pilot's right hand rides the two torque levers' mean angle (pedestal.xml's table): written by fdm/x57pilot.py
var TABLE = [[-0.174, 0], [0, 6], [1, 40], [1.66, 50]];
var deg = func(x) {
    if (x <= TABLE[0][0]) return TABLE[0][1];
    for (var i = 1; i < size(TABLE); i += 1)
        if (x <= TABLE[i][0])
            return TABLE[i - 1][1] + (x - TABLE[i - 1][0]) / (TABLE[i][0] - TABLE[i - 1][0]) * (TABLE[i][1] - TABLE[i - 1][1]);
    return TABLE[size(TABLE) - 1][1];
}
var node = props.globals.getNode("/sim/model/x57/pilot/lever-deg", 1);
node.setDoubleValue(0);
var timer = maketimer(0, func {
    node.setDoubleValue((deg(getprop("fdm/jsbsim/prop/l/lever") or 0) + deg(getprop("fdm/jsbsim/prop/r/lever") or 0)) / 2);
});
timer.start();
