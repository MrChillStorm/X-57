"""JSBSim harness for the X-57: load a build, set a flight state, trim, and find the small-perturbation modes.

    f = load('mod3')
    t = trim(f, 94, flap=12, gear=1, hp=2500)
    m = modes(f, t)
"""
import math, os, subprocess, sys
import jsbsim
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
FPS_KT = 1.6878099
WARM = ('accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2',
        'accelerations/pdot-rad_sec2', 'accelerations/qdot-rad_sec2', 'accelerations/rdot-rad_sec2')
INTEG = ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
         'simulation/integrator/position/rotational', 'simulation/integrator/position/translational')


HZ = float(os.environ.get('X57_HZ', 120))       # frames a second: FlightGear's default


class Sim(jsbsim.FGFDMExec):
    mod = None


def root(mod):
    return os.path.join(HERE, 'root-' + mod + os.environ.get('X57_ROOT_TAG', ''))     # check_all.sh: a build folder for each process


def build(mod):
    d = os.path.join(root(mod), 'aircraft', 'x57')
    built = ['--built'] if os.environ.get('X57_BUILT') else []       # the built-up model, which ships; the Phase 1 and 2 checks default to the table model
    subprocess.run([sys.executable, os.path.join(HERE, 'genx57.py'), d, '--' + mod] + built, check=True, stdout=subprocess.DEVNULL)


def load(mod, dt=None, rebuild=True):
    dt = dt or 1 / HZ
    if rebuild:
        build(mod)
    f = Sim(root(mod))
    f.set_debug_level(0)
    f.set_dt(dt)
    f.load_model('x57')
    f.mod = mod
    f['ctl/instant'] = 1
    f['ctl/hold-props'] = 1       # speeds and pitches at their commands: trims and modes. Flights clear it
    if f.get_property_manager().hasNode('flex/enable'):
        f['flex/enable'] = 0      # the rigid airplane unless a check asks: what the CFD and NASA's modes are compared with
    return f


def freeze(f, on=True):
    for k, v in zip(INTEG, (0, 0, 0, 0) if on else (1, 3, 1, 4)):
        f[k] = v


def configure(f, flap=0.0, gear=0.0, hl_mode=0, rpm=2250.0, lever=0.0, stab=None, elev=0.0, ail=0.0, rud=0.0):
    f['ctl/flap-cmd-deg'] = flap
    f['gear/gear-cmd-norm'] = gear
    f['ctl/cruise-rpm'] = rpm
    f['/controls/engines/engine[0]/throttle'] = lever
    f['/controls/engines/engine[1]/throttle'] = lever
    if f.mod == 'mod4':
        f['ctl/hl-mode'] = hl_mode
    f['fcs/elevator-cmd-norm'] = elev if stab is None else (stab + 5.0) / 10.0
    f['fcs/aileron-cmd-norm'] = ail
    f['fcs/rudder-cmd-norm'] = rud
    if f.get_property_manager().hasNode('ctl/one-rudder'):
        f['ctl/one-rudder'] = 0       # the pedals' again, after a check that set the rudder itself


def state(f, u, v, w, p=0.0, q=0.0, r=0.0, phi=0.0, theta=0.0, hp=2500.0):
    """body velocities (ft/s), rates (rad/s), attitude (rad): the state, the models run once, nothing integrated"""
    f['ic/terrain-elevation-ft'] = -3000.0
    f['ic/h-sl-ft'] = hp
    f['ic/psi-true-rad'] = 0.0
    f['ic/theta-rad'] = theta
    f['ic/phi-rad'] = phi
    f['ic/u-fps'] = u
    f['ic/v-fps'] = v
    f['ic/w-fps'] = w
    f['ic/p-rad_sec'] = p
    f['ic/q-rad_sec'] = q
    f['ic/r-rad_sec'] = r
    freeze(f)
    f.run_ic()
    f.run()
    return np.array([f[k] for k in WARM])


def air(f, V_fps, alpha_deg, beta_deg=0.0, theta=None, **kw):
    """the same, from airspeed, angle of attack and sideslip"""
    a, b = math.radians(alpha_deg), math.radians(beta_deg)
    u, v, w = V_fps * math.cos(a) * math.cos(b), V_fps * math.sin(b), V_fps * math.sin(a) * math.cos(b)
    return state(f, u, v, w, theta=a if theta is None else theta, **kw)


def tas_of_cas(f, kcas, hp):
    """true airspeed for a calibrated airspeed: by JSBSim's own conversion"""
    f['ic/h-sl-ft'] = hp
    f['ic/vc-kts'] = kcas
    return f['ic/vt-fps']


def trim(f, kcas, flap=0.0, gear=0.0, hl_mode=0, hp=2500.0, gamma_deg=0.0, rpm=2250.0, lever=None, x0=None):
    """Level flight at that speed: angle of attack, stabilator and lever for no acceleration along the flight path or
    across it, and no pitching. With the lever given (glides, regeneration), the flight path angle is solved instead."""
    V = tas_of_cas(f, kcas, hp)
    g = math.radians(gamma_deg)
    free_lever = lever is None

    def res(x):
        a, stab = x[0], x[1]
        lv = x[2] if free_lever else lever
        gam = g if free_lever else math.radians(x[2])
        configure(f, flap, gear, hl_mode, rpm, lv, stab=stab)
        ar = math.radians(a)
        acc = state(f, V * math.cos(ar), 0.0, V * math.sin(ar), theta=ar + gam, hp=hp)
        return [acc[0] * math.cos(ar) + acc[2] * math.sin(ar), -acc[0] * math.sin(ar) + acc[2] * math.cos(ar), acc[4] * 10]

    x0 = x0 or [5.0, -5.0, 0.3 if free_lever else 0.0]
    s = least_squares(res, x0, x_scale=[2, 2, 0.2 if free_lever else 2], xtol=1e-7, ftol=1e-9, gtol=1e-9)
    res(s.x)
    a, stab = s.x[0], s.x[1]
    return dict(kcas=kcas, V=V, alpha=a, stab=stab, lever=s.x[2] if free_lever else lever,
                gamma=gamma_deg if free_lever else s.x[2], flap=flap, gear=gear, hl_mode=hl_mode, hp=hp, rpm=rpm,
                residual=float(np.abs(s.fun).max()), CL=f['aero/coef/CL'], CD=f['aero/coef/CD'],
                power_kw=f['prop/r/power-kw'] + f['prop/l/power-kw'], thrust=f['prop/thrust-total-lbs'])


def modes(f, t):
    """Eigenvalues of the rigid body about a trim: the Jacobian of the model's own accelerations, central differences,
    states u, v, w, p, q, r, phi, theta. Propulsion is quasi-steady, so it follows the speed at once."""
    configure(f, t['flap'], t['gear'], t['hl_mode'], t['rpm'], t['lever'], stab=t['stab'])
    a = math.radians(t['alpha'])
    th = a + math.radians(t['gamma'])
    x0 = np.array([t['V'] * math.cos(a), 0.0, t['V'] * math.sin(a), 0, 0, 0, 0, th])

    def dx(x):
        acc = state(f, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], hp=t['hp'])
        ph, the = x[6], x[7]
        return np.r_[acc, x[3] + (x[4] * math.sin(ph) + x[5] * math.cos(ph)) * math.tan(the), x[4] * math.cos(ph) - x[5] * math.sin(ph)]

    h = np.array([1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    A = np.zeros((8, 8))
    for i in range(8):
        e = np.zeros(8)
        e[i] = h[i]
        A[:, i] = (dx(x0 + e) - dx(x0 - e)) / (2 * h[i])
    E = np.eye(8)
    if f.get_property_manager().hasNode('bu/alphadot'):       # the downwash's lag: alpha-dot is part of the forces, so its column goes on the left side
        f['bu/test/alphadot'] = 0.1
        up = dx(x0)
        f['bu/test/alphadot'] = -0.1
        dn = dx(x0)
        f['bu/test/alphadot'] = 0
        E[:, 2] -= (up - dn) / 0.2 / t['V']
    ev = np.linalg.eigvals(np.linalg.solve(E, A))
    osc = sorted([e for e in ev if e.imag > 1e-4], key=lambda e: abs(e))
    real = sorted([e.real for e in ev if abs(e.imag) <= 1e-4])
    out = dict(eig=ev, A=A, real=real)
    if len(osc) >= 3:
        ph_, dr, sp = osc[0], osc[1], osc[2]
        out.update(phugoid=ph_, dutch_roll=dr, short_period=sp)
    if real:
        out['roll_tau'] = -1.0 / real[0]
    return out


def zeta(e):
    return -e.real / abs(e)
