"""What the pilot can click, in the package as FlightGear loads it: every object of a pick, knob or slider must exist in its
model, and any Nasal it calls must be loaded. Found the door handles calling a module the aircraft no longer had.

    python3 pickcheck.py
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'X-57')
fails = []


def resolve(path, base):
    if path.startswith('Aircraft/X-57/'):
        return os.path.join(PKG, path[len('Aircraft/X-57/'):])
    return os.path.normpath(os.path.join(base, path))


def walk(xml, seen):
    if xml in seen or not xml.endswith('.xml') or not os.path.exists(xml):
        return
    seen.add(xml)
    root = ET.parse(xml).getroot()
    for m in root.findall('model'):
        walk(resolve(m.findtext('path'), os.path.dirname(xml)), seen)
    p = root.findtext('path')
    if p:
        walk(resolve(p, os.path.dirname(xml)), seen)


def main():
    seen = set()
    walk(os.path.join(PKG, 'Models', 'x57.xml'), seen)
    loaded = set(re.findall(r'<(\w+)><file>', open(os.path.join(PKG, 'x57-mod4-set.xml')).read()))
    known = loaded | {'gui', 'props', 'aircraft', 'math', 'view', 'controls'}
    clicks = 0
    for xml in sorted(seen):
        root = ET.parse(xml).getroot()
        names = {m.findtext('name') for m in root.findall('model')} | {a.findtext('name') for a in root.findall('animation')}
        p = root.findtext('path')
        if p and p.endswith('.ac'):
            ac = resolve(p, os.path.dirname(xml))
            names |= set(re.findall(r'^name "(.*)"', open(ac, errors='replace').read(), re.M))
        for a in root.findall('animation'):
            if a.findtext('type') not in ('pick', 'knob', 'slider', 'touch'):
                continue
            clicks += 1
            where = os.path.relpath(xml, PKG)
            if p and p.endswith('.ac'):
                for o in a.findall('object-name'):
                    if o.text not in names:
                        fails.append('%s: no object %s' % (where, o.text))
            for b in a.iter('binding'):
                for module in re.findall(r'\b([a-z]\w*)\.\w+', b.findtext('script') or ''):
                    if module not in known:
                        fails.append('%s: Nasal module %s is not loaded' % (where, module))
    print('%d models, %d controls' % (len(seen), clicks))
    for f in sorted(set(fails)):
        print('  MISS', f)
    print('ALL FOUND' if not fails else '%d misses' % len(set(fails)))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
