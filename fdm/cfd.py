#!/usr/bin/env python3
"""NASA's X-57 CFD results (NTRS 20250004430) into data/cfd/*.json.

    python3 cfd.py

What the spreadsheets hold, and how to read them [ReadMe; TM-20250001715]:
  - CL and CD in stability axes; CY, Cl, Cm, Cn in body axes; all on S 66.667 ft2, b 31.623 ft, c 2.13 ft, about the
    moment reference at x 154.797, z 64.527 in.
  - Stabilator and aileron positive trailing edge down, rudder positive trailing edge left. Ailerons are single sided:
    only the right one moves. Rudder deflections are negative only, and sideslip mostly positive, by symmetry.
  - Mod-III power on has one propulsor, on the right. STAR-CCM+ and USM3D ran it as a half model, whose coefficients
    are on the full reference area, so they read half the aircraft's; LAVA ran the whole aircraft. Take power on minus
    power off within one solver. Each difference is then one motor's effect, and the solvers can be compared.
  - Mod-IV runs have the twelve high-lift propulsors as actuator discs and no cruise propulsors at all.
  - Ground plane heights are feet below the wing's leading edge, not wheel heights.
  - Failure cases are not in the release: the TM only plots them (figs. 46-53).

Each column is checked against its heading, so a shifted sheet fails here and not in the flight model.
"""
import json, os, sys
import xlsx

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'refs', 'cfd')
OUT = os.path.join(HERE, 'data', 'cfd')
SIX = ('CD', 'CL', 'CY', 'Cl', 'Cm', 'Cn')
THREE = ('CD', 'CL', 'Cm')


def C(name):
    """column letters to index"""
    return xlsx.col_index(name)


def num(x):
    return float('%.6g' % x) if isinstance(x, float) else None


def expect(g, row, names):
    """the sheet's headings on that row, from column letters to the text they must start with"""
    for col, text in names.items():
        got = g[row, C(col)]
        if not (isinstance(got, str) and got.replace(' ', '').lower().startswith(text.replace(' ', '').lower())):
            raise ValueError('row %d column %s: expected %r, found %r' % (row, col, text, got))


def block(g, r, col, names=SIX):
    """a run of coefficients from a column: only those present"""
    c0 = C(col)
    d = {n: num(g[r, c0 + i]) for i, n in enumerate(names)}
    return {k: v for k, v in d.items() if v is not None}


def fields(g, r, cols):
    """named numbers from a row; text where a number belongs stops the run, so it can't vanish as an empty value"""
    out = {}
    for k, c in cols.items():
        v = g[r, C(c)]
        if isinstance(v, str):
            raise ValueError('row %d column %s (%s): text %r' % (r + 1, c, k, v))
        out[k] = num(v)
    return out


def data_rows(g, first, col='A'):
    return [r for r in range(first, g.nrows) if isinstance(g[r, C(col)], float)]


def poweroff(book):
    g = book['Power Off Base Runs']
    expect(g, 7, dict(A='Alpha', B='Beta', C='Stabilator', D='Tab', E='Aileron', F='Rudder', H='CD', I='CL', J='CY',
                      K='Cl', L='Cm', M='Cn', N='CD', T='CD', Z='CD', AE='Cn'))
    expect(g, 6, dict(G='Flap', H='StarCCM', N='LAVA', T='USM3D', Z='Kestrel'))
    rows = []
    for r in data_rows(g, 8):
        d = fields(g, r, dict(alpha='A', beta='B', stab='C', tab='D', ail='E', rud='F', flap='G'))
        for s, c in (('star', 'H'), ('lava', 'N'), ('usm3d', 'T'), ('kestrel', 'Z')):
            b = block(g, r, c)
            if b:
                d[s] = b
        rows.append(d)
    conds = [dict(alt_ft=g[r, 0], mach=num(g[r, 1]), kcas=float(str(g[r, 2]).strip('*')), re_ft=g[r, 3], flap=g[r, 4])
             for r in (2, 3, 4)]
    return dict(conditions=conds, rows=rows)


def poweroff_ground(book):
    g = book['Power Off Ground Effect']
    expect(g, 8, dict(A='Hp', B='KCAS', E='Ground plane height', F='STAR-CCM+ (Power Off Ground',
                      I='STAR-CCM+(*Power off No Ground', L='STAR-CCM+ (Deltas', O='LAVA (Power Off Ground',
                      R='LAVA (*Power off No Ground', U='LAVA ( Deltas'))
    expect(g, 9, dict(C='Alpha', D='flap', F='CD', G='CL', H='Cm', U='CD', W='Cm'))
    rows = []
    for r in data_rows(g, 10):
        d = fields(g, r, dict(hp='A', kcas='B', alpha='C', flap='D', height_ft='E'))
        for s, a, b, c in (('star', 'F', 'I', 'L'), ('lava', 'O', 'R', 'U')):
            d[s] = dict(ground=block(g, r, a, THREE), free=block(g, r, b, THREE), delta=block(g, r, c, THREE))
        rows.append(d)
    return rows


def cruise(book):
    g = book['Cruise Power']
    expect(g, 2, dict(A='Hp', B='KCAS', C='Vinf', D='Rho', E='Re', J='Ground plane', K='RPM', L='Advance', M='Blade Pitch',
                      N='Ct', O='Cp', P='Cq', Q='Thrust', R='Power', S='Torque', T='STAR-CCM+', Z='LAVA', AF='USM3D'))
    expect(g, 3, dict(F='Alpha', G='Beta', H='Aileron', I='Flap', T='CD', Y='Cn', Z='CD', AF='CD', AK='Cn'))
    rows = []
    for r in data_rows(g, 4):
        d = fields(g, r, dict(hp='A', kcas='B', v_fts='C', rho='D', re_ft='E', alpha='F', beta='G', ail='H', flap='I'))
        if isinstance(g[r, C('K')], float):
            d['prop'] = fields(g, r, dict(rpm='K', J='L', pitch='M', Ct='N', Cp='O', Cq='P', T_lbf='Q', P_kW='R', Q_lbft='S'))
        else:
            d['prop'] = None              # 'Power Off (No prop)': the baseline at this condition
        for s, c in (('star', 'T'), ('lava', 'Z'), ('usm3d', 'AF')):
            b = block(g, r, c)
            if b:
                d[s] = b
        rows.append(d)
    return rows


def highlift_settings(book):
    g = book['Notes']
    expect(g, 19, dict(A='KCAS', B='Vinf', C='Rho', D='Re', E='RPM', F='Advance', G='Ct', H='Cp', I='Cq', J='Thrust',
                       K='Power', L='Torque'))
    cols = dict(kcas='A', v_fts='B', rho='C', re_ft='D', rpm='E', J='F', Ct='G', Cp='H', Cq='I', T_lbf='J', P_kW='K', Q_lbft='L')
    return [fields(g, r, cols) for r in range(20, 30) if isinstance(g[r, 0], float)]


def highlift(book, sheet):
    g = book[sheet]
    expect(g, 2, dict(A='KCAS', I='Advance', J='StarCCM + (Absolute Values) Power On', P='StarCCM + (Absolute Values) Power Off',
                      V='StarCCM HL Delta', AB='LAVA (Absolute Values) Power On', AH='LAVA (Absolute Values) Power off',
                      AN='LAVA Deltas', AT='Kestrel - time accurate (Absolute Value) Power On',
                      AZ='Kestrel - time accurate (Absolute Value) Power Off', BF='LaRC Kestrel - time accurate Deltas',
                      BL='USM3D SA QCR - time accurate (Absolute Value) Power On',
                      BR='USM3D SA QCR - time accurate (Absolute Value) Power Off', BX='USM3D SA QCR - time accurate Deltas'))
    expect(g, 3, dict(B='Alpha', C='Beta', D='Aileron', E='Stab', F='Rudder', G='Tab', H='Flap', J='CD', O='Cn', BX='CD'))
    rows = []
    for r in data_rows(g, 4):
        double = g[r, C('B')] == '4,6'        # see split_double()
        cols = dict(kcas='A', alpha='B', beta='C', ail='D', stab='E', rud='F', tab='G', flap='H', J='I')
        if double:
            del cols['alpha']
        d = fields(g, r, cols)
        d['alpha'] = 4.0 if double else d['alpha']
        for s, a, b, c in (('star', 'J', 'P', 'V'), ('lava', 'AB', 'AH', 'AN'), ('kestrel', 'AT', 'AZ', 'BF'),
                           ('usm3d', 'BL', 'BR', 'BX')):
            x = dict(on=block(g, r, a), off=block(g, r, b), delta=block(g, r, c))
            x = {k: v for k, v in x.items() if v}
            if x:
                d[s] = x
        rows.append(d)
        if double and d['kcas'] == 58:
            rows.append(split_double(d))
    return rows


def split_double(d):
    """Two rows of the flaps 10 sheet, at 58 and 75 KCAS, give alpha as the text '4,6'. On the curves either side, and
    against Mod-III's own alpha 4 values, every solver is at 4 deg except STAR-CCM+ power on at 58 KCAS, at 6 deg. So
    its delta there is 6 deg less 4 deg, and is dropped: the 6 deg value becomes a row of its own."""
    x = d['star']
    assert abs(x['on']['CL'] - 2.526) < 0.01 and abs(x['off']['CL'] - 1.406) < 0.01, 'not the row described'
    six = {k: v for k, v in d.items() if k not in ('star', 'lava', 'kestrel', 'usm3d')}
    six['alpha'] = 6.0
    six['star'] = dict(on=x.pop('on'))
    x.pop('delta')
    return six


def highlift_ground(book):
    g = book['Ground Effect Flaps 10']
    expect(g, 10, dict(H='Ground plane', I='RPM', J='LAVA (Absolute Values) Power On', P='LAVA (Absolute Values) Power off', V='LAVA (Deltas)'))
    expect(g, 11, dict(A='Alpha', B='Beta', C='Aileron', D='Stab', E='Rudder', F='Tab', G='Flap', J='CD', AA='Cn'))
    rows = []
    for r in data_rows(g, 12):
        d = fields(g, r, dict(alpha='A', beta='B', ail='C', stab='D', rud='E', tab='F', flap='G', height_ft='H', rpm='I'))
        d['lava'] = dict(on=block(g, r, 'J'), off=block(g, r, 'P'), delta=block(g, r, 'V'))
        rows.append(d)
    g = book['Ground Effect Flaps 30']
    expect(g, 10, dict(A='Hp', I='Ground plane', J='RPM', K='StarCCM + (Absolute Values) Power On',
                       Q='StarCCM + (Absolute Values) Power Off', W='StarCCM + (Deltas)',
                       AC='LAVA (Absolute Values) Power On', AI='LAVA (Absolute Values) Power off', AO='LAVA (Deltas)'))
    expect(g, 11, dict(B='Alpha', C='Beta', D='Aileron', E='Stab', F='Rudder', G='Tab', H='Flap', K='CD', AT='Cn'))
    for r in data_rows(g, 12, 'B'):
        d = fields(g, r, dict(hp='A', alpha='B', beta='C', ail='D', stab='E', rud='F', tab='G', flap='H', height_ft='I', rpm='J'))
        d['star'] = dict(on=block(g, r, 'K'), off=block(g, r, 'Q'), delta=block(g, r, 'W'))
        d['lava'] = dict(on=block(g, r, 'AC'), off=block(g, r, 'AI'), delta=block(g, r, 'AO'))
        rows.append(d)
    return rows


def rates(book):
    RATE = ('Cmq', 'Clp', 'Clr', 'Cnr', 'Cnp')
    base = dict(hp='A', kcas='B', v_fts='C', rho='D', re_ft='E', alpha='F', beta='G', ail='H', stab='I', rud='J', flap='K')
    out = {}

    def amp(g, r, col):
        a = g[r, C(col)]
        return a if isinstance(a, float) else str(a)        # one cell reads '5,3'

    g = book['Power Off']
    expect(g, 1, dict(A='Hp', L='Axis', M='Amplitude', N='k', O='Frequency', P='Cmq', U='Cmq', Z='Cmq', AD='Cnp'))
    expect(g, 0, dict(P='StarCCM+', U='LAVA', Z='Kestrel'))
    rows = []
    for r in data_rows(g, 3):
        d = fields(g, r, base)
        d.update(axis=g[r, C('L')], amplitude_deg=amp(g, r, 'M'), k=num(g[r, C('N')]), f_hz=num(g[r, C('O')]))
        for s, c in (('star', 'P'), ('lava', 'U'), ('kestrel', 'Z')):
            b = block(g, r, c, RATE)
            if b:
                d[s] = b
        rows.append(d)
    out['power_off'] = rows

    g = book['HL Power On']
    expect(g, 1, dict(L='RPM', M='J', N='Ct', O='Cp', P='Cq', Q='Axis', R='Amplitude', S='k', T='Frequency', U='Cmq', Z='Cmq'))
    expect(g, 0, dict(U='StarCCM+', Z='Kestrel'))
    rows = []
    for r in data_rows(g, 3):
        d = fields(g, r, base)
        d['prop'] = fields(g, r, dict(rpm='L', J='M', Ct='N', Cp='O', Cq='P'))
        d.update(axis=g[r, C('Q')], amplitude_deg=amp(g, r, 'R'), k=num(g[r, C('S')]), f_hz=num(g[r, C('T')]))
        for s, c in (('star', 'U'), ('kestrel', 'Z')):
            b = block(g, r, c, RATE)
            if b:
                d[s] = b
        rows.append(d)
    out['high_lift'] = rows

    g = book['Cruise Power']
    expect(g, 1, dict(L='Blade Pitch', M='RPM', N='J', O='Ct', P='Cp', Q='Cq', R='T', S='Power', T='Torque', U='Axis',
                      V='Amplitude', W='k', X='Frequency', Y='Cmq'))
    expect(g, 0, dict(Y='StarCCM+'))
    rows = []
    for r in data_rows(g, 3):
        d = fields(g, r, base)
        if isinstance(g[r, C('M')], float):
            d['prop'] = fields(g, r, dict(pitch='L', rpm='M', J='N', Ct='O', Cp='P', Cq='Q', T_lbf='R', P_kW='S', Q_Nm='T'))
        else:
            d['prop'] = None              # 'Power off'
        d.update(axis=g[r, C('U')], amplitude_deg=amp(g, r, 'V'), k=num(g[r, C('W')]), f_hz=num(g[r, C('X')]))
        b = block(g, r, 'Y', RATE)
        if b:
            d['star'] = b
        rows.append(d)
    out['cruise'] = rows
    return out


def dump(name, obj):
    """one record to a line: readable, and a change shows up as a line in a diff"""
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name + '.json'), 'w') as f:
        if isinstance(obj, list):
            f.write('[\n' + ',\n'.join(json.dumps(x, separators=(',', ':')) for x in obj) + '\n]\n')
        else:
            f.write('{\n' + ',\n'.join(
                '%s:[\n%s\n]' % (json.dumps(k), ',\n'.join(json.dumps(x, separators=(',', ':')) for x in v))
                for k, v in obj.items()) + '\n}\n')


def build():
    src = lambda n: xlsx.read(os.path.join(SRC, n))
    off = src('Mod-III Power Off CFD Results for Public Release.xlsx')
    on = src('Mod-III Power On CFD Results for Public Release.xlsx')
    iv = src('Mod-IV CFD Results for Public Release.xlsx')
    rd = src('X57 Rate Derivative CFD Results for Public Release.xlsx')
    return dict(poweroff=poweroff(off), poweroff_ground=poweroff_ground(off), cruise=cruise(on),
                highlift_settings=highlift_settings(iv), highlift=highlift(iv, 'Flaps 30') + highlift(iv, 'Flaps 10'),
                highlift_ground=highlift_ground(iv), rates=rates(rd))


if __name__ == '__main__':
    d = build()
    for k, v in d.items():
        dump(k, v)
        n = {a: len(b) for a, b in v.items()} if isinstance(v, dict) else len(v)
        print('%-20s %s' % (k, n))
