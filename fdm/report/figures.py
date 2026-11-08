#!/usr/bin/env python3
"""The report's figures, from the check logs in results/ and upset.json."""
import json, os, re
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, 'results')
F = os.path.join(HERE, 'fig')
plt.rcParams.update({'font.family': 'serif', 'font.size': 8, 'axes.grid': True, 'grid.linewidth': 0.3, 'axes.linewidth': 0.5, 'lines.linewidth': 1.0,
                     'lines.markersize': 3.5, 'legend.fontsize': 7, 'legend.frameon': False, 'xtick.major.width': 0.4, 'ytick.major.width': 0.4})
BLUE, RED, GREY, GREEN = '#1f4e8c', '#b03030', '#777777', '#2a7a3a'


def log(n):
    return open(os.path.join(R, n + '.log'), errors='replace').read()


def geometry():
    """the discretisation, drawn from the generator's own data, as the ASK 21's report does"""
    import sys
    from matplotlib.patches import Polygon
    os.environ['X57_BUILT'] = '1'
    sys.path.insert(0, os.path.join(HERE, '..'))
    import x57built as B
    import p2006t as P
    IN = 0.0254
    c = B.config(True)
    fus = P.fuselage()
    rows = c['rows']
    half = [r for r in rows if r['side'] > 0]
    nf, na = sum(r['flap'] for r in half), sum(r['aileron'] for r in half)
    st, fn, bodies, discs = c['stab']['strips'], c['fin']['strips'], {b['name']: b for b in c['bodies']}, c['discs']
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.0, 6.6), gridspec_kw=dict(height_ratios=[2.05, 1.0], hspace=0.30))
    for r in rows:
        y0, y1, s = r['y0'], r['y1'], r['side']
        xl = [B.le(y0 / IN) * IN, B.le(y1 / IN) * IN]
        ch = [B.chord(y0 / IN) * IN, B.chord(y1 / IN) * IN]
        col = '#f2dcd9' if r['aileron'] else '#d9e6f2' if r['flap'] else '#eeeeee'
        a1.add_patch(Polygon(np.c_[s * np.array([y0, y1, y1, y0]), [xl[0], xl[1], xl[1] + ch[1], xl[0] + ch[0]]], closed=True, fc=col, ec='0.35', lw=0.4))
        a1.plot(s * 0.5 * (y0 + y1), r['x34'], '.', color=BLUE, ms=2.2)
    for h in st:
        for s in (-1, 1):
            y0, y1, x0 = h['y'] - h['dy'] / 2, h['y'] + h['dy'] / 2, h['x4'] - 0.25 * h['c']
            a1.add_patch(Polygon(np.c_[s * np.array([y0, y1, y1, y0]), [x0, x0, x0 + h['c'], x0 + h['c']]], closed=True, fc='#e3f0d9', ec='0.35', lw=0.4))
            a1.plot(s * h['y'], h['x34'], '.', color=BLUE, ms=2.2)
    xs = np.array([q['x'] for q in bodies['f']['stations']])
    x_in = np.linspace(xs[0] - 0.2, xs[-1] + 0.55, 240) / IN
    w = np.array([fus['half_w'](x) for x in x_in])
    a1.add_patch(Polygon(np.c_[np.r_[w, -w[::-1]], np.r_[x_in, x_in[::-1]] * IN], closed=True, fc='#eeeeee', ec='0.25', lw=0.5, zorder=0))
    for d in discs:
        a1.plot([d['hub'][1] - d['D'] / 2, d['hub'][1] + d['D'] / 2], [d['hub'][0]] * 2, color='0.1', lw=1.1)
    for k, b in bodies.items():
        for s in ((0,) if k == 'f' else (-1, 1)):
            a1.plot([s * (b.get('y') or 0.0)] * len(b['stations']), [q['x'] for q in b['stations']], 's', color=RED, ms=1.8)
    a1.set_aspect('equal')
    a1.set_xlim(-5.8, 5.8)
    a1.set_ylim(9.6, 0.1)
    a1.set(xlabel='$y$, m', ylabel='$x$ aft, m')
    a1.grid(False)
    a1.set_title('Plan view: %d wing strips a side (%d flapped, blue; %d aileron, red), %d stabilator strips a side (green)\n'
                 'dots: 3/4 chord; bars: the %d discs; squares: body stations' % (len(half), nf, na, len(st), len(discs)), fontsize=7.5)
    z = np.array([fus['zc'](x) for x in x_in])
    hh = np.array([fus['half_h'](x) for x in x_in])
    a2.plot(x_in * IN, z + hh, color='0.2', lw=0.7)
    a2.plot(x_in * IN, z - hh, color='0.2', lw=0.7)
    for q in bodies['f']['stations']:
        a2.axvline(q['x'], color='0.75', lw=0.3)
        a2.plot(q['x'], q['z'], 's', color=RED, ms=2.2)
    for v in fn:
        z0, z1, x0 = v['z'] - v['dz'] / 2, v['z'] + v['dz'] / 2, v['x4'] - 0.25 * v['c']
        a2.add_patch(Polygon(np.c_[[x0, x0 + v['c'], x0 + v['c'], x0], [z0, z0, z1, z1]], closed=True, fc='#f6ecd2', ec='0.35', lw=0.4))
    x0 = st[0]['x4'] - 0.25 * st[0]['c']
    a2.plot([x0, x0 + st[0]['c']], [st[0]['z']] * 2, color='0.1', lw=1.6)
    a2.plot([B.le(0) * IN, (B.le(0) + B.chord(0)) * IN], [B.Z_WING * IN] * 2, color=BLUE, lw=1.6)
    for d in discs[:1] + discs[2:3]:
        a2.plot([d['hub'][0]] * 2, [d['hub'][2] - d['D'] / 2, d['hub'][2] + d['D'] / 2], color='0.1', lw=1.1)
    a2.set_aspect('equal')
    a2.set_xlim(-0.2, 10.4)
    a2.set_ylim(0.2, 3.9)
    a2.set(xlabel='$x$ aft, m', ylabel='$z$, m')
    a2.grid(False)
    a2.set_title('Side view: %d fuselage stations (squares), %d fin strips, stabilator and wing root (bars), a cruise and a high-lift disc' % (len(bodies['f']['stations']), len(fn)), fontsize=7.5)
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(F, 'geometry.pdf'))
    print('strips', len(half), nf, na, len(st), len(fn), len(bodies['f']['stations']), {k: len(b['stations']) for k, b in bodies.items()})


def modii():
    rows = re.findall(r'(\d+) KCAS: short ([\d.]+) \(([\d.]+)\), ([\d.]+) \(0\.59\); phugoid ([\d.]+) \(([\d.]+)\), ([\d.]+) \(([\d.]+)\); Dutch ([\d.]+) \(([\d.]+)\), ([\d.]+) \(([\d.]+)\); roll ([\d.]+) s \(([\d.]+)\)', log('p3'))
    a = np.array(rows, float)
    fig, ax = plt.subplots(1, 2, figsize=(3.4, 2.0))
    ax[0].plot(a[:, 0], a[:, 1], 'o-', color=BLUE, label='model')
    ax[0].plot(a[:, 0], a[:, 2], 's--', color=RED, mfc='none', label='Mod II simulator')
    ax[0].plot([110], [3.41], 'x', color=GREY, label='flight paper')
    ax[0].plot([110], [4.95], '+', color=GREY, label="paper's derivatives")
    ax[0].set(xlabel='KCAS', ylabel='short period, rad/s')
    ax[1].plot(a[:, 0], a[:, 3], 'o-', color=BLUE)
    ax[1].plot(a[:, 0], 0.59 + 0 * a[:, 0], 's--', color=RED, mfc='none')
    ax[1].plot([110], [0.40], 'x', color=GREY)
    ax[1].plot([110], [0.54], '+', color=GREY)
    ax[1].set(xlabel='KCAS', ylabel='damping ratio', ylim=(0.3, 0.7))
    fig.legend(*ax[0].get_legend_handles_labels(), loc='upper center', ncol=2, handlelength=1.6, labelspacing=0.2, columnspacing=1.0)
    fig.tight_layout(pad=0.3, rect=(0, 0, 1, 0.84))
    fig.savefig(os.path.join(F, 'modii.pdf'))


def modes():
    s = log('p1')
    sec = s[s.index('mod3: model'):s.index('mod4_fixed: model')]
    rows = re.findall(r'^\s+(\d+)\s+zeta phugoid ([\d.]+) \(([\d.]+)\)\s+short ([\d.]+) \(([\d.]+)\)\s+dutch ([\d.]+) \(([\d.]+)\)\s+roll ([\d.]+) s \(([\d.]+)\)', sec, re.M)
    a = np.array(rows, float)
    fig, ax = plt.subplots(1, 4, figsize=(7.0, 1.8))
    for i, (c, n) in enumerate(((1, 'phugoid damping'), (3, 'short period damping'), (5, 'Dutch roll damping'), (7, 'roll time constant, s'))):
        ax[i].plot(a[:, 0], a[:, c], 'o-', color=BLUE, label='model')
        ax[i].plot(a[:, 0], a[:, c + 1], 's--', color=RED, mfc='none', label='NASA simulator')
        ax[i].set(xlabel='KCAS', ylabel=n)
    fig.legend(*ax[0].get_legend_handles_labels(), loc='upper center', ncol=2, handlelength=1.6)
    fig.tight_layout(pad=0.3, rect=(0, 0, 1, 0.86))
    fig.savefig(os.path.join(F, 'modes.pdf'))


def dcl(n):
    m = re.search(r'dCL, model \(STAR-CCM\+\): (.*)', log(n)).group(1)
    return np.array(re.findall(r'(\d+): ([\d.]+) \(([\d.]+)\)', m), float)


def slipstream():
    a, b, c = dcl('p4'), dcl('ring-c'), dcl('ring-b')
    fig, ax = plt.subplots(figsize=(3.4, 2.0))
    ax.plot(a[:, 0], a[:, 2], 'ks-', mfc='none', label='CFD (STAR-CCM+)')
    ax.plot(a[:, 0], a[:, 1], 'o-', color=BLUE, label='ring wake, one constant')
    ax.plot(b[:, 0], b[:, 1], 'o--', color=GREEN, mfc='none', label='ring wake, no constant')
    ax.plot(c[:, 0], c[:, 1], '^:', color=RED, mfc='none', label='plain jet, no constant')
    ax.set(xlabel='angle of attack, deg', ylabel=r'propellers\' $\Delta C_L$')
    ax.legend(loc='upper left', labelspacing=0.2)
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(F, 'slipstream.pdf'))


NASA = dict(t=[0, .5, 1, 1.5, 2, 2.5, 3], p=[0, -5, -16, -29, -41, -46, -46], r=[0, -7, -13, -15, -13.5, -12, -14], phi=[0, -1.5, -7, -20, -38, -61, -88],
            theta=[8.3, 8.6, 8.3, 6.8, 2, -4.5, -11], beta=[0, 1.2, 4.5, 8, 8.8, 6.5, 3.9], sink=[0, .2, .5, 1, 4, 11, 24.5])


def upset():
    d = json.load(open(os.path.join(R, 'upset.json')))
    keys = (('p', 'roll rate, deg/s'), ('r', 'yaw rate, deg/s'), ('phi', 'bank, deg'), ('theta', 'pitch, deg'), ('beta', 'sideslip, deg'), ('sink', 'sink, ft/s'))
    fig, ax = plt.subplots(2, 3, figsize=(7.0, 2.95), sharex=True)
    for x, (k, n) in zip(ax.ravel(), keys):
        x.plot(d['built']['t'], d['built'][k], '-', color=BLUE, label='built-up model')
        x.plot(d['table']['t'], d['table'][k], '--', color=GREEN, label='table model')
        x.plot(NASA['t'], NASA[k], 's', color=RED, mfc='none', label='NASA simulator, read by eye')
        x.set(ylabel=n)
    for x in ax[1]:
        x.set_xlabel('time after failure, s')
    fig.legend(*ax[0, 0].get_legend_handles_labels(), loc='upper center', ncol=3, handlelength=1.8)
    fig.tight_layout(pad=0.3, rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(F, 'upset.pdf'))


def cruise():
    s = log('p3')
    pts = np.array(re.findall(r'\d+/\d+%: (\d+) \((\d+)\)', s), float)
    fig, ax = plt.subplots(figsize=(3.0, 2.0))
    ax.plot(pts[:, 1], pts[:, 0], 'o', color=BLUE, mfc='none')
    ax.plot([90, 150], [90, 150], 'k-', lw=0.5)
    ax.set(xlabel='flight manual, KTAS', ylabel='model, KTAS')
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(F, 'cruise.pdf'))


for g in (geometry, modii, modes, slipstream, upset, cruise):
    g()
print('figures done')
