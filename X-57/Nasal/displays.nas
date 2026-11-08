# X-57 cockpit displays: the Canvas pages of the panel's digital units, on the screens of Models/Interior/Panel/X57
# (written by fdm/x57panel.py), after ICD-CEPT-006 rev E and NASA's photographs. Each screen's light is the lit effect's,
# /instrumentation/x57/nits/<unit> in cd/m2, and each unit goes dark when its bus does [20240009560 fig. 5].
#
# Not in the flight model, so left as dashes on the D175: motor and controller temperatures, pages 2, 3 and 5 to 7.

var SANS = "LiberationFonts/LiberationSans-Bold.ttf";
var MONO = "LiberationFonts/LiberationMono-Bold.ttf";
var SEG = "DSEG/DSEG7/Classic/DSEG7Classic-Bold.ttf";
var WHITE = [0.95, 0.95, 0.95];
var GREEN = [0.45, 1.0, 0.25];
var RED = [1.0, 0.16, 0.08];
var ICE = [0.70, 0.86, 1.0];
var BLUE = [0.15, 0.35, 1.0];
var J = "/fdm/jsbsim/";
var X = "/sim/model/x57/";
var D2R = math.pi / 180;

var get = func(path, d = 0) {
    var v = getprop(path);
    return v == nil ? d : v;
};

var screen = func(name, w, h, r = 0, g = 0, b = 0) {
    var c = canvas.new({"name": name, "size": [w, h], "view": [w, h], "mipmapping": 1});
    c.addPlacement({"node": name ~ ".screen"});
    c.setColorBackground(r, g, b, 1);
    return c;
};

var label = func(g, x, y, size, colour, align, font, words) {
    return g.createChild("text").setFont(font).setFontSize(size, 1).setColor(colour[0], colour[1], colour[2])
        .setAlignment(align).setTranslation(x, y).setText(words);
};

var line = func(g, pts, colour, w) {
    var p = g.createChild("path").moveTo(pts[0][0], pts[0][1]);
    for (var i = 1; i < size(pts); i += 1) p.lineTo(pts[i][0], pts[i][1]);
    return p.setColor(colour[0], colour[1], colour[2]).setStrokeLineWidth(w);
};


var nits = func {
    var e = 90 - get("/sim/time/sun-angle-rad", 1) / D2R;
    return 80 * math.pow(2500 / 80, math.max(0, math.min(1, (e + 6) / 16)));
};

var alive = func(path) { return get(J ~ path) > 9; };
var sw = func(n) { return get(J ~ "ctl/sw/" ~ n) > 0.5; };

# a screen that goes dark with its bus
var Unit = {
    init: func(m, name, bus) {
        m.name = name;
        m.bus = bus;
        m.lit = -1;
    },
    tick: func(day) {
        var on = alive(me.bus) ? 1 : 0;
        if (on != me.lit) {
            me.lit = on;
            on ? me.g.show() : me.g.hide();
        }
        setprop("/instrumentation/x57/nits/" ~ me.name, on * day);
        if (on) me.update();
    },
};

# Aerospace Logic TM200 [slide 14]: digits from 300 rpm, a green bar from 500, red above 3080
var Tach = {
    new: func(name, prop, bus) {
        var m = {parents: [Tach, Unit], prop: prop};
        Unit.init(m, name, bus);
        m.g = screen(name, 256, 256, 0.01, 0.015, 0.03).createGroup();
        m.v = label(m.g, 232, 128, 78, ICE, "right-baseline", SEG, "0");
        label(m.g, 128, 172, 30, ICE, "center-baseline", SANS, "RPM");
        label(m.g, 128, 208, 26, ICE, "center-baseline", SANS, "TACH");
        m.bar = m.g.createChild("path").rect(0, 0, 200, 16).setColorFill(GREEN[0], GREEN[1], GREEN[2]).setTranslation(28, 222);
        return m;
    },
    update: func {
        var r = get(me.prop);
        me.v.setText(r < 300 ? "0" : sprintf("%.0f", r));
        var c = r > 3080 ? RED : GREEN;
        me.bar.setColorFill(c[0], c[1], c[2]).setColor(c[0], c[1], c[2]);
        me.bar.setScale(math.max(0.001, math.min(1, (r < 500 ? 0 : r) / 3500)), 1);
    },
};

# AcuDC 243 [slides 16, 17; 20240006533]: volts, amps and power on 28 V from the BCM. It autoscales: the decimal point moves, and past 1,000 W it shows kW, with only a small mark to say so
var four = func(x) {
    var a = abs(x);
    var s = a >= 1000 ? sprintf("%.0f", x) : a >= 100 ? sprintf("%.1f", x) : a >= 10 ? sprintf("%.2f", x) : sprintf("%.3f", x);
    return s;
};

var Meter = {
    new: func(name, bus, side) {
        var m = {parents: [Meter, Unit], b: bus, side: side};
        Unit.init(m, name, "elec/v28");
        m.g = screen(name, 256, 288, 0.0, 0.035, 0.0).createGroup();
        m.l = [];
        for (var i = 0; i < 3; i += 1) {
            append(m.l, label(m.g, 226, 82 + i * 88, 56, GREEN, "right-baseline", SEG, "0.000"));
        }
        m.k = label(m.g, 6, 240, 20, GREEN, "left-baseline", SANS, "");
        return m;
    },
    update: func {
        var k = me.side ~ me.b;
        var closed = get(J ~ "elec/bus-" ~ me.b) > 0.5 and sw("tc-" ~ k);
        var v = closed ? get(J ~ "bat/" ~ me.b ~ "/volts") : 0;
        var n = get(J ~ "elec/cmc-" ~ me.side ~ "a") + get(J ~ "elec/cmc-" ~ me.side ~ "b");
        var w = (closed and get(J ~ "elec/cmc-" ~ k) > 0.5) ? 1000 * get(J ~ "prop/" ~ me.side ~ "/power-kw") / math.max(n, 1) : 0;
        me.l[0].setText(four(v));
        me.l[1].setText(four(v > 1 ? w / v : 0));
        me.l[2].setText(four(abs(w) >= 1000 ? w / 1000 : w));
        me.k.setText(abs(w) >= 1000 ? "k" : "");
    },
};

# Aerospace Logic VA200K [slide 19]: a DC converter's volts and amps, the bar red outside 13 to 14.6 V and 0 to 70 A
var Avionics = {
    new: func(name, b) {
        var m = {parents: [Avionics, Unit], b: b};
        Unit.init(m, name, "elec/dc-" ~ b ~ "-v");
        m.g = screen(name, 256, 256, 0.02, 0.0, 0.0).createGroup();
        label(m.g, 68, 56, 26, WHITE, "center-baseline", SANS, "VOLTS");
        label(m.g, 188, 56, 26, WHITE, "center-baseline", SANS, "AMPS");
        m.v = label(m.g, 68, 126, 38, WHITE, "center-baseline", SEG, "0.0");
        m.a = label(m.g, 188, 126, 38, WHITE, "center-baseline", SEG, "0.0");
        m.bv = m.g.createChild("path").rect(0, 0, 96, 12).setTranslation(20, 150);
        m.ba = m.g.createChild("path").rect(0, 0, 96, 12).setTranslation(140, 150);
        label(m.g, 128, 216, 32, WHITE, "center-baseline", SANS, "BATTERY");
        return m;
    },
    update: func {
        var v = get(J ~ "elec/conv-" ~ me.b ~ "-v");
        var a = get(J ~ "elec/conv-" ~ me.b ~ "-amps");
        if (v < 1) v = get(J ~ "elec/dc-" ~ me.b ~ "-v");       # the meter reads the voltage that powers it
        me.v.setText(sprintf("%.1f", v));
        me.a.setText(sprintf("%.1f", a));
        var cv = (v < 13 or v > 14.6) ? RED : GREEN;
        var ca = (a < 0 or a > 70) ? RED : GREEN;
        me.bv.setColorFill(cv[0], cv[1], cv[2]).setColor(cv[0], cv[1], cv[2]);
        me.ba.setColorFill(ca[0], ca[1], ca[2]).setColor(ca[0], ca[1], ca[2]);
        me.bv.setScale(math.max(0.001, math.min(1, v / 16)), 1);
        me.ba.setScale(math.max(0.001, math.min(1, a / 70)), 1);
    },
};

# Flight Data Systems GT-50 [slide 21]: Mode steps through g, the Essential Bus's volts, a timer, a flight timer and the clock.
# Action shows the g limits, or runs the timer; Reset clears. The OAT mode is inoperative in this installation
var MODES = ["G", "VOLT", "TIMER", "FLIGHT", "CLOCK"];
var GMeter = {
    new: func(name) {
        var m = {parents: [GMeter, Unit], mode: 0, gmax: 1, gmin: 1, show: 0, run: 0, t: 0, ft: 0, last: 0};
        Unit.init(m, name, "elec/ess-v");
        m.g = screen(name, 256, 128, 0.72, 0.56, 0.20).createGroup();
        m.v = label(m.g, 240, 104, 74, [0.05, 0.04, 0.02], "right-baseline", SEG, "1.0");
        m.u = label(m.g, 8, 34, 26, [0.05, 0.04, 0.02], "left-baseline", SANS, "");
        setlistener(X ~ "gmeter/mode", func(n) { if (n.getValue()) { m.mode = math.mod(m.mode + 1, size(MODES)); m.show = 0; } }, 0, 0);
        setlistener(X ~ "gmeter/action", func(n) {
            if (!n.getValue()) return;
            if (m.mode == 0) m.show = math.mod(m.show + 1, 3);
            if (m.mode == 2) m.run = !m.run;
        }, 0, 0);
        setlistener(X ~ "gmeter/reset", func(n) {
            if (!n.getValue()) return;
            if (m.mode == 0) { m.gmax = 1; m.gmin = 1; m.show = 0; }
            if (m.mode == 2) { m.t = 0; m.run = 0; }
            if (m.mode == 3) m.ft = 0;
        }, 0, 0);
        return m;
    },
    hms: func(t) { return sprintf("%d:%02d", math.floor(t / 3600), math.mod(math.floor(t / 60), 60)); },
    update: func {
        var g = get("/accelerations/pilot-gdamped", 1);
        var t = get("/sim/time/elapsed-sec");
        var dt = t - me.last;
        me.last = t;
        me.gmax = math.max(me.gmax, g);
        me.gmin = math.min(me.gmin, g);
        if (me.run) me.t += dt;
        if (get("/velocities/groundspeed-kt") > 40) me.ft += dt;
        var word = MODES[me.mode];
        var text = "";
        if (me.mode == 0) {
            var x = me.show == 1 ? me.gmax : me.show == 2 ? me.gmin : g;
            text = sprintf("%.1f", x);
            word = me.show == 1 ? "MAX" : me.show == 2 ? "MIN" : "G";
        } elsif (me.mode == 1) text = sprintf("%.1f", get(J ~ "elec/ess-v"));
        elsif (me.mode == 2) text = me.hms(me.t);
        elsif (me.mode == 3) text = me.hms(me.ft);
        else text = sprintf("%02d:%02d", get("/sim/time/utc/hour"), get("/sim/time/utc/minute"));
        me.v.setText(text);
        me.u.setText(word);
    },
};

var Uhf = {
    new: func(name) {
        var m = {parents: [Uhf, Unit]};
        Unit.init(m, name, "elec/ess-v");
        m.g = screen(name, 256, 128, 0.03, 0.0, 0.0).createGroup();
        label(m.g, 12, 54, 46, RED, "left-baseline", SEG, "CH20");
        label(m.g, 12, 116, 46, RED, "left-baseline", SEG, "399.97");
        return m;
    },
    update: func {},
};

var Gtn = {
    new: func(name) {
        var m = {parents: [Gtn, Unit]};
        Unit.init(m, name, "elec/ess-v");
        m.g = screen(name, 512, 288, 0.0, 0.02, 0.05).createGroup();
        label(m.g, 16, 40, 28, ICE, "left-baseline", SANS, "COM");
        m.c = label(m.g, 16, 92, 52, GREEN, "left-baseline", MONO, "");
        m.s = label(m.g, 496, 92, 40, WHITE, "right-baseline", MONO, "");
        label(m.g, 16, 160, 28, ICE, "left-baseline", SANS, "NAV");
        m.n = label(m.g, 16, 212, 52, GREEN, "left-baseline", MONO, "");
        m.t = label(m.g, 496, 212, 40, WHITE, "right-baseline", MONO, "");
        return m;
    },
    update: func {
        me.c.setText(sprintf("%.3f", get("/instrumentation/comm/frequencies/selected-mhz")));
        me.s.setText(sprintf("%.3f", get("/instrumentation/comm/frequencies/standby-mhz")));
        me.n.setText(sprintf("%.2f", get("/instrumentation/nav/frequencies/selected-mhz")));
        me.t.setText(sprintf("%.2f", get("/instrumentation/nav/frequencies/standby-mhz")));
    },
};

var Gtx = {
    new: func(name) {
        var m = {parents: [Gtx, Unit]};
        Unit.init(m, name, "elec/av-a-v");
        m.g = screen(name, 384, 104, 0.0, 0.0, 0.10).createGroup();
        label(m.g, 16, 70, 40, ICE, "left-baseline", SANS, "ALT");
        m.v = label(m.g, 368, 76, 56, ICE, "right-baseline", SEG, "7000");
        return m;
    },
    update: func { me.v.setText(sprintf("%04d", get("/instrumentation/transponder/id-code", 7000))); },
};

# the high lift ground crew panel [slide 64]: twelve motors, blue ready, green running, red fault. Its right switch turns the display on
var Crew = {
    new: func(name) {
        var m = {parents: [Crew, Unit], led: []};
        Unit.init(m, name, "elec/ess-v");
        m.g = screen(name, 512, 128, 0.08, 0.085, 0.09).createGroup();
        var cols = [0.02, 0.15, 0.85];
        for (var i = 0; i < 12; i += 1) {
            var x = 34 + i * 37 + (i >= 6 ? 8 : 0);
            var row = [];
            forindex (var r; cols) {
                var c = [[0.05, 0.1, 0.8], [0.05, 0.6, 0.15], [0.8, 0.05, 0.05]][r];
                append(row, m.g.createChild("path").circle(13, x, 22 + r * 42).setColorFill(c[0], c[1], c[2]).setColor(0, 0, 0).hide());
            }
            append(m.led, row);
        }
        return m;
    },
    update: func {
        var on = get(X ~ "crew-display");
        var run = get(J ~ "prop/hl/rpm") > 100;
        for (var i = 0; i < 12; i += 1) {
            var b = math.mod(i, 2) == 0 ? "a" : "b";
            var rdy = get("/instrumentation/x57/lamp/hl-rdy-" ~ b) > 0;
            var flt = get("/instrumentation/x57/lamp/hl-fault-" ~ b) > 0;
            on and rdy ? me.led[i][0].show() : me.led[i][0].hide();
            on and run ? me.led[i][1].show() : me.led[i][1].hide();
            on and flt ? me.led[i][2].show() : me.led[i][2].hide();
        }
    },
};

# the D175 [slides 53-55; 20230005312]. Its first page as NASA's photograph; the pages after it as listed, with dashes for what the model doesn't hold
var Dial = {
    new: func(g, x, words) {
        var m = {parents: [Dial], x: x};
        label(g, x, 26, 17, WHITE, "center-baseline", SANS, "Est");
        label(g, x, 44, 17, WHITE, "center-baseline", SANS, words);
        var pts = [];
        for (var a = -135; a <= 135; a += 15) append(pts, [x + 42 * math.sin(a * D2R), 96 - 42 * math.cos(a * D2R)]);
        line(g, pts, WHITE, 2);
        m.n = line(g, [[0, 0], [0, -36]], RED, 3).setTranslation(x, 96);
        m.v = label(g, x, 170, 26, WHITE, "center-baseline", SANS, "0");
        label(g, x, 192, 17, WHITE, "center-baseline", SANS, "Nm");
        return m;
    },
    set: func(nm) {
        me.n.setRotation((-135 + 270 * math.max(0, math.min(1, (nm + 130) / 450))) * D2R);
        me.v.setText(sprintf("%.0f", nm));
    },
};

var ROWS = {
    2: ["CMC Max Temps", "MOSFET", "FPGA", "CPU Board", "Driver Board", "Vicor"],
    3: ["Motor Max Temps", "Winding", "Fwd Bear", "Mid Bear", "Aft Bear"],
    5: ["Motor Windings", "MW 1", "MW 2", "MW 3", "MW 4", "MW 5", "MW 6"],
    6: ["Left Nacelle", "Duct In1", "Duct In2", "Duct Out1", "CM Out1", "CM Out2", "Aft HS1"],
    7: ["Right Nacelle", "Duct In1", "Duct In2", "Duct Out1", "CM Out1", "CM Out2", "Aft HS1"],
};

var D175 = {
    new: func(name) {
        var m = {parents: [D175, Unit], page: 0, groups: {}};
        Unit.init(m, name, "elec/ess-v");
        var c = screen(name, 640, 400, 0.01, 0.01, 0.06);
        m.g = c.createGroup();
        var g = m.g.createChild("group");
        m.groups[1] = g;
        label(g, 320, 46, 36, WHITE, "center-baseline", SANS, "Torque Levers");
        label(g, 262, 92, 34, WHITE, "center-baseline", SANS, "L");
        label(g, 378, 92, 34, WHITE, "center-baseline", SANS, "R");
        line(g, [[320, 62], [320, 196]], WHITE, 3);
        m.l = label(g, 262, 150, 36, WHITE, "center-baseline", SANS, "0 %");
        m.r = label(g, 378, 150, 36, WHITE, "center-baseline", SANS, "0 %");
        m.tl = Dial.new(g, 70, "Torque L");
        m.tr = Dial.new(g, 570, "Torque R");
        label(g, 320, 262, 32, WHITE, "center-baseline", SANS, "Motor Power");
        label(g, 320, 316, 32, WHITE, "center-baseline", SANS, "A  Battery  B");
        label(g, 320, 370, 32, WHITE, "center-baseline", SANS, "A  SoC  B");
        m.pl = label(g, 150, 262, 30, WHITE, "right-baseline", SANS, "");
        m.pr = label(g, 490, 262, 30, WHITE, "left-baseline", SANS, "");
        m.va = label(g, 150, 316, 30, WHITE, "right-baseline", SANS, "");
        m.vb = label(g, 490, 316, 30, WHITE, "left-baseline", SANS, "");
        m.sa = label(g, 150, 370, 30, WHITE, "right-baseline", SANS, "");
        m.sb = label(g, 490, 370, 30, WHITE, "left-baseline", SANS, "");
        foreach (var k; keys(ROWS)) {
            var q = m.g.createChild("group");
            m.groups[k] = q;
            label(q, 320, 50, 38, WHITE, "center-baseline", SANS, ROWS[k][0]);
            label(q, 340, 86, 26, WHITE, "right-baseline", SANS, "L");
            label(q, 520, 86, 26, WHITE, "right-baseline", SANS, "R");
            for (var i = 1; i < size(ROWS[k]); i += 1) {
                label(q, 70, 86 + i * 46, 30, WHITE, "left-baseline", SANS, ROWS[k][i]);
                label(q, 340, 86 + i * 46, 30, WHITE, "right-baseline", SANS, "--");
                label(q, 520, 86 + i * 46, 30, WHITE, "right-baseline", SANS, "--");
            }
        }
        var q = m.g.createChild("group");
        m.groups[4] = q;
        label(q, 320, 110, 30, WHITE, "center-baseline", SANS, "MFD Software Version: 3.4.22");
        label(q, 320, 160, 30, WHITE, "center-baseline", SANS, "ACL Software Version: 0.0");
        m.pitch_l = label(q, 200, 280, 36, WHITE, "right-baseline", SANS, "");
        m.pitch_r = label(q, 440, 280, 36, WHITE, "left-baseline", SANS, "");
        label(q, 320, 280, 34, WHITE, "center-baseline", SANS, "Pitch");
        m.groups[8] = m.g.createChild("group");
        return m;
    },
    show: func(n) {
        foreach (var i; keys(me.groups)) i == n ? me.groups[i].show() : me.groups[i].hide();
        me.page = n;
    },
    update: func {
        var n = get(X ~ "d175-page", 1);
        if (n != me.page) me.show(n);
        if (n == 1) {
            me.l.setText(sprintf("%.0f %%", 100 * get(J ~ "prop/l/lever")));
            me.r.setText(sprintf("%.0f %%", 100 * get(J ~ "prop/r/lever")));
            me.tl.set(get(J ~ "prop/l/torque-nm"));
            me.tr.set(get(J ~ "prop/r/torque-nm"));
            me.pl.setText(sprintf("%.1f kW", get(J ~ "prop/l/power-kw")));
            me.pr.setText(sprintf("%.1f kW", get(J ~ "prop/r/power-kw")));
            me.va.setText(sprintf("%.1f Vdc", get(J ~ "bat/a/volts")));
            me.vb.setText(sprintf("%.1f Vdc", get(J ~ "bat/b/volts")));
            me.sa.setText(sprintf("%.0f %%", get(J ~ "x57/battery-a/soc-pct")));
            me.sb.setText(sprintf("%.0f %%", get(J ~ "x57/battery-b/soc-pct")));
        } elsif (n == 4) {
            me.pitch_l.setText(sprintf("%.0f deg", get(J ~ "prop/l/pitch-deg")));
            me.pitch_r.setText(sprintf("%.0f deg", get(J ~ "prop/r/pitch-deg")));
        }
    },
};

var units = [];
var timer = nil;

var tick = func {
    var day = nits();
    setprop("/instrumentation/x57/day-nits", day);
    foreach (var u; units) u.tick(day);
};

var start = func {
    units = [Tach.new("rpm-l", J ~ "prop/l/rpm", "elec/dc-a-v"), Tach.new("rpm-r", J ~ "prop/r/rpm", "elec/dc-b-v"),
             Meter.new("acc-al", "a", "l"), Meter.new("acc-ar", "a", "r"), Meter.new("acc-bl", "b", "l"), Meter.new("acc-br", "b", "r"),
             Avionics.new("avi-a", "a"), Avionics.new("avi-b", "b"), GMeter.new("gmeter"), Uhf.new("uhf"), Gtn.new("gtn"), Gtx.new("gtx"), Crew.new("crew"), D175.new("d175")];
    timer = maketimer(0.2, tick);        # kept: an unreferenced timer is collected and the screens freeze
    timer.start();
};

var ready = setlistener("/sim/signals/fdm-initialized", func(n) {
    if (n.getBoolValue()) {
        removelistener(ready);
        start();
    }
}, 1, 0);
