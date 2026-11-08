#!/usr/bin/env python3
"""NASA's high-lift propeller tables [ANLYS-CEPT-023 Rev A, appendices B and C, 8 October 2021] into data/hlp_tables.json.

    python3 hlp_data.py

Appendix B is the Airspeed mode's schedule, rpm against equivalent airspeed and pressure altitude, with the torque, shaft
power and thrust it gives; appendix C is the same at the Fixed mode's 4800 rpm. Each is given for four atmospheres; the
standard day's are kept, with each point's advance ratio and coefficients, in NASA's definitions.
"""
import json, math, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, '..', 'refs', '20240014295-ANLYS-CEPT-023-HLP-operating-conditions.pdf')
D_FT = 22.674 / 12
KT = 1.6878099
NM_FTLB = 0.737562


def tables():
    text = subprocess.run(['pdftotext', '-layout', PDF, '-'], capture_output=True, text=True, check=True).stdout
    out, cur, head = {}, None, None
    for ln in text.split('\n'):
        m = re.search(r'Table (\d+):\s*(.+)', ln)
        if m and 'Appendix' not in ln:
            cur, head = int(m.group(1)), None
            out.setdefault(cur, dict(title=m.group(2).strip(), rows={}))
            continue
        if cur is None:
            continue
        if re.match(r'\s*hp, ft', ln):
            head = [float(x) for x in re.findall(r'-?\d+\.?\d*', ln.split('ft', 1)[1])]
            continue
        p = re.findall(r'-?\d+\.?\d*', ln)
        if head and len(p) == len(head) + 1 and float(p[0]) < 20000:
            out[cur]['rows'].setdefault(float(p[0]), {}).update(dict(zip(head, map(float, p[1:]))))
    return out


def isa(h_ft):
    """the standard atmosphere's density, slug/ft3"""
    return 0.0023769 * (1 - 6.87559e-6 * h_ft) ** 4.2559


def points(rpm_of, torque, power, thrust):
    pts = []
    for hp, row in thrust['rows'].items():
        rho = isa(hp)
        for k, T in row.items():
            rpm = rpm_of(hp, k)
            Q = torque['rows'][hp][k] * NM_FTLB
            n = rpm / 60.0
            V = k * KT / math.sqrt(rho / 0.0023769)
            pts.append(dict(hp=hp, keas=k, rpm=rpm, J=V / (n * D_FT), Ct=T / (rho * n * n * D_FT ** 4), Cq=Q / (rho * n * n * D_FT ** 5),
                            T_lbf=T, Q_nm=torque['rows'][hp][k], P_kw=power['rows'][hp][k]))
    return pts


def main():
    t = tables()
    assert 'Speed, RPM' in t[1]['title'] and 'Torque' in t[2]['title'] and 'Standard' in t[2]['title'], t[2]['title']
    assert 'Standard' in t[14]['title'] and 'Torque' in t[14]['title'] and 'Thrust' in t[16]['title'], t[14]['title']
    hps = sorted(t[1]['rows'])
    ks = sorted(t[1]['rows'][hps[0]])
    air = points(lambda h, k: t[1]['rows'][h][k], t[2], t[3], t[4])
    fix = points(lambda h, k: 4800.0, t[14], t[15], t[16])
    out = dict(source='ANLYS-CEPT-023 Rev A appendices B and C, standard day', hp_ft=hps, keas=ks,
               rpm=[[t[1]['rows'][h][k] for k in ks] for h in hps], airspeed=air, fixed=fix)
    with open(os.path.join(HERE, 'data', 'hlp_tables.json'), 'w') as f:
        json.dump(out, f, separators=(',', ':'))
    return out


if __name__ == '__main__':
    o = main()
    for nm in ('airspeed', 'fixed'):
        P = o[nm]
        print(nm, len(P), 'points; J', round(min(p['J'] for p in P), 3), 'to', round(max(p['J'] for p in P), 3))
    allp = o['airspeed'] + o['fixed']
    J = np.array([p['J'] for p in allp])
    bins = np.array([0.1, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 2.0])
    print('  J       n    Ct mean (spread)     Cq mean (spread)')
    for a, b in zip(bins, bins[1:]):
        m = (J >= a) & (J < b)
        if m.any():
            c = np.array([p['Ct'] for p, q in zip(allp, m) if q])
            d = np.array([p['Cq'] for p, q in zip(allp, m) if q])
            print('  %.1f-%.1f %4d  %.4f (%.4f)   %.5f (%.5f)' % (a, b, m.sum(), c.mean(), c.max() - c.min(), d.mean(), d.max() - d.min()))
