#!/usr/bin/env python3
"""How the propellers' level changes with height: PyWopWop's own model at the same true airspeed, rpm and Ct at sea level and
aloft. x57sound.py's ALT_POWER comes from this: the fall in dB(A) over 20 log sigma.

    PYTHONPATH=<PyWopWop> <PyWopWop>/.venv/bin/python soundalt.py       (several minutes, draft quality)
"""
import sys, math
import numpy as np
from pywopwop.propeller import load_prop_aircraft, render_group, PropCondition
from pywopwop.model import Observer
from pywopwop.atmosphere import Atmosphere
from pywopwop.synth import _db, a_weighted_ms
ac=load_prop_aircraft('x57')
place=dict(cockpit=Observer('cockpit',position=tuple(ac.cockpit)),side=Observer('side',90.0,-20.0,30.0))
fs=48000.0
rho0=Atmosphere(0.0).rho
def level(group,kt,rpm,ct,h,alpha,cl,view):
    cruise=group.startswith('cruise')
    rot=next(r for r in ac.rotors if r.name==ac.groups[group][0])
    D=2*rot.radius
    T=None if ct is None else ct*Atmosphere(h).rho*(rpm/60)**2*D**4
    c=PropCondition('t',kt,h,alpha,cl,rpm if cruise else 0.0,22.0,0.0 if cruise else rpm)
    r=render_group(ac,c,group,place[view],fs=fs,duration=2.0,seed=5,thrust=T,quality='draft')
    x=r.signals['total']; p=r.meta['rotors'][ac.groups[group][0]]
    return _db(float(np.mean(x**2))),_db(a_weighted_ms(x,fs)),p['thrust_N'],p['power_kW']
for h in (2438.0,4572.0):
    s=Atmosphere(h).rho/rho0
    print('altitude %.0f m: sigma %.3f, 20log sigma %.2f dB, speed of sound ratio %.3f'%(h,s,20*math.log10(s),Atmosphere(h).c/Atmosphere(0).c))
    for g,kt,rpm,ct,al,cl in (('cruise_left',150,2250,0.05,2.0,0.4),('cruise_left',100,2700,0.12,2.0,0.6),('hl_b_left',70,4800,None,4.0,1.8),('hl_b_left',100,4000,None,4.0,1.0)):
        for v in ('side','cockpit'):
            a=level(g,kt,rpm,ct,0.0,al,cl,v); b=level(g,kt,rpm,ct,h,al,cl,v)
            print('  %-12s %3d kt %d rpm ct %-5s %-8s OASPL %+.2f dB  dBA %+.2f dB  (law %.2f; misfit %+.2f, %+.2f)  thrust ratio %.3f'%(g,kt,rpm,ct,v,b[0]-a[0],b[1]-a[1],20*math.log10(s),b[0]-a[0]-20*math.log10(s),b[1]-a[1]-20*math.log10(s),b[2]/a[2]),flush=True)
