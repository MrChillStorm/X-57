# The two doors, from the P2006T: the crew door on the left, the passenger door on the right. Their handles are in the
# model, the annunciator's MAIN DOOR and REAR DOOR lamps in cockpit.nas.
var crew = aircraft.door.new("/sim/model/door-positions/crew", 2, 0);
var passenger = aircraft.door.new("/sim/model/door-positions/passenger", 2, 0);
props.globals.getNode("/sim/model/door-left", 1).alias("/sim/model/door-positions/crew/position-norm");
props.globals.getNode("/sim/model/door-right", 1).alias("/sim/model/door-positions/passenger/position-norm");

# each door's sounds (x57sound.py): sim/model/x57/door/NAME starts at -1, which neither plays on, goes to 1 as the door leaves its frame and to 0 as it shuts
var door_sound = func(name, door) {
    var snd = props.globals.getNode("/sim/model/x57/door/" ~ name, 1);
    snd.setIntValue(-1);
    setlistener(door.positionN, func(n) {
        var v = n.getValue() or 0;
        if (v > 0.001 and snd.getValue() != 1)
            snd.setIntValue(1);
        elsif (v <= 0.001 and snd.getValue() == 1)
            snd.setIntValue(0);
    }, 0, 0);
}
door_sound("main", crew);
door_sound("rear", passenger);
