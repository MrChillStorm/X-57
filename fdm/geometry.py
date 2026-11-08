#!/usr/bin/env python3
"""The X-57 Mod III/IV geometry, read from NASA's OpenVSP model into data/geometry.json and data/sections/*.dat.

    python3 geometry.py            # write them and print the summary

Stations in inches as the model has them: x aft, y right, z up. Only the right side is listed where the aircraft is
symmetric. The moment reference and the CFD's reference quantities are NASA's [ReadMe table 1, in the CFD results].
"""
import json, os, sys
import numpy as np
import vsp

HERE = os.path.dirname(os.path.abspath(__file__))
VSP = os.path.join(HERE, '..', 'refs', 'geometry', 'X-57.vsp3')
DATA = os.path.join(HERE, 'data')
FT = 12.0

# NASA's reference quantities for every CFD coefficient [ReadMe table 1; TM-20250001715 table 1]
REF = dict(S_ft2=66.667, b_ft=31.623, c_ft=2.13, mrs_in=(154.797, 0.0, 64.527), wing_le_in=(144.878, 0.0, 87.491))


def planform(c):
    """span, area and mean aerodynamic chord of a Wing component from its panels, both sides"""
    y = S = M = 0.0
    panels = []
    for p in vsp.wing_sections(c):
        b, cr, ct = p.get('Span', 0.0), p.get('Root_Chord', 0.0), p.get('Tip_Chord', 0.0)
        if b <= 1.01:                # OpenVSP's root section, a placeholder
            continue
        area = 0.5 * (cr + ct) * b
        panels.append(dict(y0=y, y1=y + b, c0=cr, c1=ct, sweep=p.get('Sweep', 0.0), twist=p.get('Twist', 0.0),
                           dihedral=p.get('Dihedral', 0.0), loc=p.get('Sweep_Location', 0.0)))
        S += area
        M += b * (cr * cr + cr * ct + ct * ct) / 3
        y += b
    return dict(span=2 * y, area=2 * S, mac=M / S, panels=panels)


def write_section(name, up, lo):
    """Selig order, trailing edge over the top to the trailing edge"""
    os.makedirs(os.path.join(DATA, 'sections'), exist_ok=True)
    with open(os.path.join(DATA, 'sections', name + '.dat'), 'w') as f:
        f.write(name + '\n')
        for x, z in np.r_[up[::-1], lo[1:]]:
            f.write('%.6f %.6f\n' % (x, z))


def blade_table(b):
    """the blade's stations on the chord curve's r/R, in the propeller's own terms. Twist is the blade angle where
    the model states it directly; where it is set by the angle at 3/4 radius, it is shifted to that angle"""
    r, cr = b['curves']['chord']
    rt, tw = b['curves']['twist']
    tw = np.interp(r, rt, tw)
    if b['use_beta34']:
        tw = tw - np.interp(0.75, r, tw) + b['beta34']
    out = dict(r_R=r, chord_R=cr, beta_deg=tw)
    for k in ('rake', 'skew', 'thick'):
        rr, v = b['curves'][k]
        out[k] = np.interp(r, rr, v)
    return {k: [round(float(x), 5) for x in v] for k, v in out.items()}


def build():
    m = vsp.Model(VSP)
    w = m.first('CleanWing')
    P = planform(w)
    hl = sorted(m.named('HL-Prop'), key=lambda c: c.loc[1])
    cruise = m.first('Cruise-Prop')
    bh, bc = vsp.blade(hl[0]), vsp.blade(cruise)

    # the wing's chord where each propeller stands
    def chord(y):
        for p in P['panels']:
            if p['y0'] <= y <= p['y1'] + 1e-6:
                return p['c0'] + (p['c1'] - p['c0']) * (y - p['y0']) / (p['y1'] - p['y0'])
        return None

    g = dict(
        source='NASA OpenVSP model X-57.vsp3 (NASA-3D-Resources)',
        units='inches, x aft, y right, z up',
        reference=REF,
        wing=dict(origin_in=[round(float(v), 3) for v in w.loc], span_in=round(P['span'], 3), area_in2=round(P['area'], 2),
                  mac_in=round(P['mac'], 3), aspect_ratio=round(P['span'] ** 2 / P['area'], 3), panels=P['panels'],
                  le_marker_in=[round(float(v), 3) for v in m.first('WingLE_Marker').loc]),
        high_lift=dict(diameter_in=bh['diameter_in'], blades=bh['blades'], beta34_deg=round(bh['beta34'], 3),
                       airfoil=bh['foils'][0][0], count_per_side=len(hl),
                       hubs_in=[[round(float(v), 3) for v in c.loc] for c in hl],
                       wing_chord_in=[round(chord(c.loc[1]), 3) for c in hl], blade=blade_table(bh)),
        cruise=dict(diameter_in=bc['diameter_in'], blades=bc['blades'], beta34_deg=round(bc['beta34'], 3),
                    hub_in=[round(float(v), 3) for v in cruise.loc], blade=blade_table(bc),
                    slices=[f[0] for f in bc['foils']],
                    slice_thickness=[round(vsp.thickness(f[1], f[2]), 4) for f in bc['foils']]),
        surfaces={c.name: dict(loc_in=[round(float(v), 3) for v in c.loc], rot_deg=[round(float(v), 3) for v in c.rot],
                               mirrored=c.mirrored)
                  for c in m.comps if c.type == 'Hinge'},
        components=sorted({c.name for c in m.comps}),
        component_count=len(m.comps),
    )
    # the fuselage, tail and gear pods: the P2006T's own, kept on every Mod [TM 2.1]
    hs = planform(m.first('HorzStab'))
    g['fuselage'] = vsp.body_sections(m.first('Fuselage'))
    g['gear_pod'] = dict(y_in=float(m.first('GearPod').loc[1]), sections=vsp.body_sections(m.first('GearPod')))
    g['stabilator'] = dict(le_in=[round(float(v), 3) for v in m.first('HorzStab').loc], half_span_in=hs['span'] / 2, chord_in=hs['panels'][0]['c0'],
                           hinge_in=g['surfaces']['HorzStab-Hinge']['loc_in'][0], thickness=0.11)
    g['fin'] = vsp.fin_panels(m.first('VertTail'))
    for k in ('Wing_Sec1', 'Flap'):
        s = [p['airfoil'] for p in vsp.wing_sections(m.first(k)) if p['airfoil']]
        n, up, lo = s[0]
        write_section('wing_root' if k == 'Wing_Sec1' else 'flap_root', up, lo)
        g.setdefault('sections', {})['wing_root' if k == 'Wing_Sec1' else 'flap_root'] = dict(
            name=n, thickness=round(vsp.thickness(up, lo), 4))
    n, up, lo = bh['foils'][0]
    write_section('hl_blade', up, lo)
    g['sections']['hl_blade'] = dict(name=n, thickness=round(vsp.thickness(up, lo), 4))
    for i, (n, up, lo) in enumerate(bc['foils']):
        write_section('cruise_blade_%02d' % (i + 1), up, lo)
    return g


if __name__ == '__main__':
    g = build()
    with open(os.path.join(DATA, 'geometry.json'), 'w') as f:
        json.dump(g, f, indent=1)
    w = g['wing']
    print('wing: S %.3f ft2 (NASA %.3f)  b %.3f ft (%.3f)  MAC %.3f ft (%.2f)  AR %.2f' % (
        w['area_in2'] / 144, REF['S_ft2'], w['span_in'] / FT, REF['b_ft'], w['mac_in'] / FT, REF['c_ft'], w['aspect_ratio']))
    print('panels:', [(round(p['y0'], 2), round(p['y1'], 2), round(p['c0'], 2), round(p['c1'], 2)) for p in w['panels']])
    print('origin', w['origin_in'], 'LE marker', w['le_marker_in'], 'NASA wing LE', REF['wing_le_in'])
    h = g['high_lift']
    print('high lift: D %.3f in, %d blades, beta34 %.2f, %s' % (h['diameter_in'], h['blades'], h['beta34_deg'], h['airfoil']))
    print(' hubs', h['hubs_in'])
    print(' spacing', np.round(np.diff([p[1] for p in h['hubs_in']]), 3), 'wing chord', h['wing_chord_in'])
    c = g['cruise']
    print('cruise: D %.1f in, %d blades, beta34 %.1f, hub %s' % (c['diameter_in'], c['blades'], c['beta34_deg'], c['hub_in']))
    for nm, b in (('HL', h['blade']), ('cruise', c['blade'])):
        i = np.linspace(0, len(b['r_R']) - 1, 7).astype(int)
        print(nm, 'r/R', [b['r_R'][k] for k in i]); print('  c/R', [b['chord_R'][k] for k in i]); print('  beta', [b['beta_deg'][k] for k in i])
    print('sections', g['sections'])
    print('hinges', {k: v['loc_in'] for k, v in g['surfaces'].items()})
