"""The rudder pedals' geometry, shared by the pilot (mkpilot.py, in Blender) and the pedals' model (x57pilot.py). Plain Python: no numpy.

The P2006T's pedals: "a master cylinder is attached to each pilot/co-pilot's rudder pedal" [P2006T flight manual, 7-25], and NASA's
X-57 simulators have "a rudder pedal assembly which includes toe brakes", one on each pedal, for differential braking
[20230004174; 20240006496]. NASA's photograph of the Mod IV cockpit [20240010539 fig. 4] shows them under the left half of the
panel: hung on a cross tube at the floor, each pedal two black arms leaning forward to a bar under the ball of the foot, a
tan master cylinder beside each, braided steel lines. The copilot's are gone: "The copilot seat, yoke, and rudder pedals
have been removed and replaced with a research instrumentation pallet" [20240007413]. Neither gives a dimension, so these
are mine: sized for a median pilot (1.75 m) on the seat as x57cabin.py sets it, the balls of his feet on the bars, his
knees bent 140 deg under the yoke, the leg that pushes full rudder still reaching its pedal; the floor runs on to the
forward bulkhead at x -3.30, 22 cm ahead of the bars. Interior frame (x aft, y right, z up), metres.
"""
import math

HINGE_X, HINGE_Z = -2.94, -0.80          # the floor tube's axis
ARM = 0.198                              # hinge to the bar
LEAN = 45.0                              # the arms' lean forward from upright, degrees, at neutral
RUDDER_DEG = 15.0                        # each pedal's swing at full rudder: forward on the side pushed, back on the other (estimate)
BRAKE_DEG = 15.0                         # the toe brake's tilt at full brake (estimate)
SPREAD = 0.105                           # each pedal's centre from its seat's centreline
PILOT_Y = -0.285                         # the pilot's centreline
WIDTH = 0.095                            # a pedal's bar, across
TOE = 0.065                              # the toe brake plate above the bar, along the arm


def bar(lean=LEAN):
    """the bar's centre (x, z) at a lean"""
    a = math.radians(lean)
    return HINGE_X - ARM * math.sin(a), HINGE_Z + ARM * math.cos(a)


def pedals():
    """(name, y) of each pedal: the pilot's left and right"""
    return [('pl', PILOT_Y - SPREAD), ('pr', PILOT_Y + SPREAD)]


def swing(name, rudder):
    """a pedal's extra lean forward, degrees, at a rudder input (-1 left .. 1 right): the left pedals forward for left rudder"""
    s = -1.0 if name.endswith('l') else 1.0
    return s * rudder * RUDDER_DEG
