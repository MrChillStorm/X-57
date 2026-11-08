#!/usr/bin/env python3
"""The exterior lights, as the Bo 105's (its fdm/hdrmodel.py): each a halo and, in the HDR pipeline, a spot light aimed out through its lens, in candela.

All four are on DC Converter Bus B in NASA's power drawings, ICD-CEPT-006 slide 44 and 20240009560 fig. 5: nav, taxi and landing 3 A, strobe 7.5 A. Both
collar out nav, taxi and landing, the second with the Mod IV additions, so only the strobe was to be live. Here all four work. Neither drawing gives a place.
Position lights and strobes on the outboard side of each tip nacelle, as the P2006T carries them at its wing tips, with the white position light on the tail
cone and a third strobe on top of the fin, where the P2006T has its beacon. The landing and taxi lights are in the left wing's leading edge, 2.75 and 2.89 m
out: NASA's photograph of Mod II shows them there on the stock wing, just outboard of the nacelle (20240010807 fig. 3), and the P2006T's flight manual walks
past them there, where the FGAddon model sets them. No source shows them on the Mod III wing, so the same distance out is carried over, which falls between
the fourth and fifth high-lift nacelles. Which lamp is which isn't known. Levels: 40 cd red and green, 20 white,
400 cd strobes, 200 000 cd landing, as the Bo 105's, and a wider 30 000 cd taxi. Their states come from Nasal/lights.nas: each needs its switch and its bus.
"""
import os, re, shutil

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
HDR_CD = 635.0           # the pipeline's unit
L = 'sim/model/x57/lighting/'
RED, GREEN, WHITE, WARM = (1.0, 0.06, 0.03), (0.15, 1.0, 0.45), (1.0, 1.0, 1.0), (1.0, 0.92, 0.80)
TIP_X, TIP_Y, TIP_Z = 3.91, 5.03, 2.226
# name, halo ('disc': a flat one facing along the beam), position (model frame, m), direction, colour, emission, cd on axis, spot exponent, cutoff (deg), range (m), the state that lights it, wing side
LIGHTS = [('nav-red', 'RedLight.ac', (TIP_X, -TIP_Y, TIP_Z), (0, -1, 0), RED, (2, 0, 0), 40, 3, 70, 20, L + 'nav', -1),
          ('nav-green', 'GreenLight.ac', (TIP_X, TIP_Y, TIP_Z), (0, 1, 0), GREEN, (0, 2, 0), 40, 3, 70, 20, L + 'nav', 1),
          ('nav-white', 'WhiteLight.ac', (8.95, 0.0, 1.76), (1, 0, 0), WHITE, (2, 2, 2), 20, 3, 70, 20, L + 'nav', 0),
          ('strobe-left', 'WhiteLight.ac', (TIP_X - 0.16, -TIP_Y, TIP_Z), (0, -1, 0), WHITE, (2, 2, 2), 400, 1, 80, 40, L + 'strobe-left/state', -1),
          ('strobe-right', 'WhiteLight.ac', (TIP_X - 0.16, TIP_Y, TIP_Z), (0, 1, 0), WHITE, (2, 2, 2), 400, 1, 80, 40, L + 'strobe-right/state', 1),
          ('strobe-tail', 'WhiteLight.ac', (8.95, 0.0, 3.41), (0, 0, 1), WHITE, (2, 2, 2), 400, 1, 80, 40, L + 'strobe-tail/state', 0),
          ('landing', 'disc', (3.776, -2.75, 2.237), (-1, 0, -0.105), WARM, (3, 3, 3), 200000, 20, 12, 400, L + 'landing', 0),
          ('taxi', 'disc', (3.781, -2.89, 2.237), (-1, 0, -0.21), WARM, (3, 3, 3), 30000, 6, 28, 120, L + 'taxi', 0)]
LAMPS = {l[0]: l[2] for l in LIGHTS if l[0] in ('landing', 'taxi')}      # on the wing's leading edge, and bending with it
HALO_SCALE = [(0, 0.15), (20, 0.35), (1500, 4.0)]      # m -> the halo's size (m)
# the positions above are the lamps, inside their lenses. Each halo sits just outside its lens, along its beam, or the lens hides
# the halo's bright middle and a lit lamp looks like a dark ball in a ring: how far the lens reaches that way (mkfg.py), and 5 mm
HALO_OUT = {'landing': 0.015, 'taxi': 0.015, 'nav-white': 0.014, 'strobe-tail': 0.023,
            'nav-red': 0.006, 'nav-green': 0.006, 'strobe-left': 0.006, 'strobe-right': 0.006}


# The halos' glow. FlightGear's HDR pipeline leaves out the emission of Effects/model-transparent (its shader passes none), so
# there they shone only by the light falling on them: at dusk, lit by the sunset, and at night not at all. They go through
# Effects/model-pbr-transparent, which takes an emissive texture: the halo's own, times a luminance in the pipeline's unit.
# A screen can't show a lamp's own luminance (a landing lamp's face is near 1e8 cd/m2, and at night the auto exposure makes
# some 20 cd/m2 white), so the halo stands in for the glare, and HDR's bloom spreads its brightest part. Its strength: the
# lamp's intensity spread over the halo at its largest (HALO_SCALE's last size), sum(alpha x luminance) over the quad x its
# size squared = cd on axis. So no halo carries more light than its lamp, from any distance; nearer, where it is smaller, it
# carries less. A billboard glows alike from every side, which suits the wide position lights and strobes. Landing and taxi
# are narrow beams: their halo is a disc facing along the beam, seen whole from ahead, edge-on from the side and not from
# behind. Being far brighter, it takes its own texture, a glare core about a lens wide at 20 m that falls smoothly to nothing
# at the quad's edge: the old halo's still has 1 % of its peak there, which showed the quad's square outline.
GLOW_REF_M = HALO_SCALE[-1][0]


def disc_glow(n=256):
    """the disc's texture: white, alpha x linear value = 1 / (1 + (r / 0.05)^2) x (1 - (r / 0.5)^2)^2 across the quad (r from its
    centre, the quad 1 wide), split evenly between alpha and colour, so 8 bits reach down to 1e-5 of the peak"""
    c = (np.arange(n) + 0.5) / n - 0.5
    r = np.hypot(*np.meshgrid(c, c))
    g = np.where(r < 0.5, 1.0 / (1.0 + (r / 0.05) ** 2) * np.clip(1.0 - (r / 0.5) ** 2, 0.0, 1.0) ** 2, 0.0)
    a = np.sqrt(g)
    srgb = np.where(a <= 0.0031308, 12.92 * a, 1.055 * a ** (1 / 2.4) - 0.055)
    return np.dstack([srgb, srgb, srgb, a])


def unit(d):
    n = sum(c * c for c in d) ** 0.5
    return tuple(c / n for c in d)


def halo(l):
    """where a light's model sits: its halo, just outside the lens"""
    return tuple(p + HALO_OUT[l[0]] * c for p, c in zip(l[2], unit(l[3])))


def halo_quad(ac):
    """the texture coordinates of a halo's first quad"""
    t = open(os.path.join(HERE, 'lights', ac)).read()
    return [tuple(map(float, m.groups())) for m in re.finditer(r'^\d+ (\S+) (\S+)$', t, re.M)][:4]


def glow_k(ac, rgb=(1.0, 1.0, 1.0)):
    """the mean over a halo's quad of alpha x the luminance of the texture's linear colour times rgb: its glow, per unit
    emissive factor and square metre"""
    if ac == 'disc':
        r = np.round(disc_glow() * 255.0) / 255.0          # as written
    else:
        im = np.asarray(Image.open(os.path.join(HERE, 'lights', 'lights.png')).convert('RGBA'), float) / 255.0
        uv = halo_quad(ac)
        H, W = im.shape[:2]
        u, v = [q[0] for q in uv], [q[1] for q in uv]
        r = im[int((1 - max(v)) * H):int((1 - min(v)) * H), int(min(u) * W):int(max(u) * W)]
    lin = np.where(r[..., :3] <= 0.04045, r[..., :3] / 12.92, ((r[..., :3] + 0.055) / 1.055) ** 2.4)
    return float((((lin * rgb) @ (0.2126, 0.7152, 0.0722)) * r[..., 3]).mean())


def glow(halo, rgb, cd):
    """the emissive factor, in the pipeline's unit: the lamp's colour, scaled to its intensity (the red and green textures
    are tinted too, which the colour's luminance through them takes in)"""
    s = float(np.interp(GLOW_REF_M, [e[0] for e in HALO_SCALE], [e[1] for e in HALO_SCALE]))
    lum = cd / (glow_k(halo, rgb) * s * s)
    return tuple(lum * q / HDR_CD for q in rgb)


def disc_ac(d):
    """a halo as one quad facing along the beam d (model frame: x aft, y right, z up), in AC3D's (x aft, y up, z left)"""
    n = np.array((d[0], d[2], -d[1]), float)
    n /= np.linalg.norm(n)
    a = np.cross(n, (0.0, 1.0, 0.0))
    a = a / np.linalg.norm(a) if np.linalg.norm(a) > 1e-6 else np.array((1.0, 0.0, 0.0))
    b = np.cross(n, a)                           # a, b, n right-handed: counter-clockwise seen from where n points, one-sided
    corners = [(-1, -1, 0, 0), (1, -1, 1, 0), (1, 1, 1, 1), (-1, 1, 0, 1)]
    vs = [0.5 * (i * a + j * b) for i, j, _, _ in corners]
    out = ['AC3Db', 'MATERIAL "DefaultWhite" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0.5 0.5 0.5  shi 64  trans 0', 'OBJECT world', 'kids 1',
           'OBJECT poly', 'name "Light"', 'texture "disc.png"', 'crease 45.000001', 'numvert 4']
    out += ['%.6f %.6f %.6f' % tuple(q) for q in vs]
    out += ['numsurf 1', 'SURF 0x10', 'mat 0', 'refs 4'] + ['%d %.6f %.6f' % (k, c[2], c[3]) for k, c in enumerate(corners)] + ['kids 0']
    return '\n'.join(out) + '\n'


def light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state):
    sel = lambda obj, extra='': ('  <animation>\n    <type>select</type>\n    <object-name>%s</object-name>\n    <condition>\n'
                                 '%s      <property>%s</property>\n    </condition>\n  </animation>\n' % (obj, extra, state))
    g = glow(halo, rgb, cd)
    d = unit(d)
    s = 0.04 - HALO_OUT[name]           # the spot light stays 4 cm out from the lens's centre
    path = name + '-disc.ac' if halo == 'disc' else halo
    tex = 'disc.png' if halo == 'disc' else 'lights.png'
    x = '<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s, written by fdm/x57lights.py -->\n\n<PropertyList>\n\n  <path>%s</path>\n  <nopreview/>\n\n' % (name, path)
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>Light</object-name>\n  </animation>\n\n'
    x += ('  <effect>      <!-- its glow: %g cd over the halo at %g m (x57lights.py) -->\n    <inherits-from>Effects/model-pbr-transparent</inherits-from>\n    <parameters>\n'
          '      <metallic-factor type="float">0</metallic-factor>\n      <roughness-factor type="float">1</roughness-factor>\n'
          '      <texture n="3">\n        <image>%s</image>\n        <type>2d</type>\n'
          '        <filter>linear-mipmap-linear</filter>\n        <wrap-s>clamp</wrap-s>\n        <wrap-t>clamp</wrap-t>\n      </texture>\n'
          '      <emissive-factor type="vec3d">%.4g %.4g %.4g</emissive-factor>\n    </parameters>\n    <object-name>Light</object-name>\n  </effect>\n\n'
          % ((cd, GLOW_REF_M, tex) + g))
    x += '  <animation>\n    <type>alpha-test</type>\n    <alpha-factor>0.01</alpha-factor>\n  </animation>\n\n'
    if halo != 'disc':
        x += '  <animation>\n    <type>billboard</type>\n    <object-name>Light</object-name>\n    <spherical type="bool">true</spherical>\n  </animation>\n\n'
    x += ('  <animation>\n    <type>material</type>\n    <object-name>Light</object-name>\n    <emission>\n      <red> %g </red>\n'
          '      <green> %g </green>\n      <blue> %g </blue>\n    </emission>\n  </animation>\n\n' % emis)
    x += ('  <animation>\n    <type>dist-scale</type>\n    <object-name>Light</object-name>\n    <interpolation>\n' +
          ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in HALO_SCALE) + '    </interpolation>\n  </animation>\n\n')
    x += sel('Light') + '\n'
    x += ('  <light>\n    <name>%s-light</name>\n    <type>spot</type>\n    <position>\n      <x-m> %g </x-m>\n      <y-m> %g </y-m>\n'
          '      <z-m> %g </z-m>\n    </position>\n    <direction>\n      <lookat-x-m> %g </lookat-x-m>\n      <lookat-y-m> %g </lookat-y-m>\n'
          '      <lookat-z-m> %g </lookat-z-m>\n    </direction>\n    <color>\n      <r> %g </r>\n      <g> %g </g>\n      <b> %g </b>\n'
          '    </color>\n    <intensity> %.5f </intensity>      <!-- %g cd -->\n    <spot-exponent> %g </spot-exponent>\n'
          '    <spot-cutoff> %g </spot-cutoff>\n    <range-m> %g </range-m>\n  </light>\n\n'
          % ((name,) + tuple(s * c for c in d) + tuple(s * c + c for c in d) + tuple(rgb) + (cd / HDR_CD, cd, exp, cut, rng)))
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>%s-light</object-name>\n  </animation>\n\n' % name
    x += sel('%s-light' % name, '      <property>/sim/rendering/hdr/hdr-enabled</property>\n')
    return x + '\n</PropertyList>\n'


def install(out):
    folder = os.path.join(out, 'Models', 'Lights')
    os.makedirs(folder, exist_ok=True)
    for f in ('RedLight.ac', 'GreenLight.ac', 'WhiteLight.ac', 'lights.png'):
        shutil.copy(os.path.join(HERE, 'lights', f), folder)
    Image.fromarray(np.round(disc_glow() * 255.0).astype(np.uint8), 'RGBA').save(os.path.join(folder, 'disc.png'))
    for name, halo, pos, d, rgb, emis, cd, exp, cut, rng, state, side in LIGHTS:
        if halo == 'disc':
            open(os.path.join(folder, name + '-disc.ac'), 'w').write(disc_ac(d))
        open(os.path.join(folder, name + '.xml'), 'w').write(light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state))


def models():
    """the lights in the aircraft's model file"""
    return ''.join('<model><name>light-%s</name><path>Aircraft/X-57/Models/Lights/%s.xml</path><offsets><x-m>%.3f</x-m><y-m>%.3f</y-m><z-m>%.3f</z-m></offsets></model>\n'
                   % ((l[0], l[0]) + halo(l)) for l in LIGHTS)
