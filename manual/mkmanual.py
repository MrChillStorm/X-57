#!/usr/bin/env python3
"""The manual's cockpit tables, straight from fdm/cockpit-guide.md, so the guide and the manual can't drift apart.

    python3 mkmanual.py     writes tab/tab_*.tex
"""
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
GUIDE = os.path.join(HERE, '..', 'fdm', 'cockpit-guide.md')
# heading: table's name, columns
TABLES = {'Quick start': ('keys', 'lX'), 'Start-up': ('startup', 'lp{58mm}X'), 'Takeoff: DiTTo': ('speeds', 'Xccc'),
          'Levers': ('levers', 'llX'), 'Left panel': ('left', 'p{50mm}X'), 'High-lift panel (Mod IV)': ('highlift', 'p{42mm}X'),
          'Annunciator': ('annunciator', 'p{34mm}p{34mm}lX'), 'Centre stack': ('centre', 'p{42mm}X'), 'D175 pages': ('d175', 'lX'),
          'Right panel': ('right', 'p{42mm}X'), 'Lower panels': ('lower', 'p{42mm}X'), 'Power system': ('power', 'p{34mm}p{38mm}X')}


def cell(s):
    s = s.strip().replace('\\', '').replace('&', r'\&').replace('%', r'\%').replace('_', r'\_').replace('#', r'\#')
    s = s.replace('m²', r'm\textsuperscript{2}').replace('°', r'\textdegree{}')
    s = re.sub(r'`([^`]+)`', r'\\ann{\1}', s)
    return re.sub(r'\*\*([^*]+)\*\*', r'\\textbf{\1}', s)


def main():
    os.makedirs(os.path.join(HERE, 'tab'), exist_ok=True)
    head, seen = None, set()
    lines = open(GUIDE, encoding='utf-8').read().split('\n')
    i = 0
    while i < len(lines):
        m = re.match(r'#+ (.+)', lines[i])
        if m:
            head = m.group(1).strip()
        if lines[i].startswith('|') and head in TABLES and head not in seen:
            rows = []
            while i < len(lines) and lines[i].startswith('|'):
                rows.append([cell(c) for c in lines[i].strip().strip('|').split('|')])
                i += 1
            name, cols = TABLES[head]
            body = [' & '.join(r'\textbf{%s}' % c if c else '' for c in rows[0]) + r' \\', r'\midrule']
            body += [('{}' if r[0].startswith('[') else '') + ' & '.join(r) + r' \\' for r in rows[2:]]
            spec = cols.replace('X', r'>{\raggedright\arraybackslash}X')
            spec = re.sub(r'p\{(\d+mm)\}', r'>{\\raggedright\\arraybackslash}p{\1}', spec)
            with open(os.path.join(HERE, 'tab', 'tab_%s.tex' % name), 'w', encoding='utf-8') as f:
                f.write('\\begin{tabularx}{\\textwidth}{@{}%s@{}}\n\\toprule\n%s\n\\bottomrule\n\\end{tabularx}\n' % (spec, '\n'.join(body)))
            seen.add(head)
            continue
        i += 1
    missing = set(TABLES) - seen
    if missing:
        raise SystemExit('not in the guide: %s' % ', '.join(sorted(missing)))
    print('%d tables from the guide' % len(seen))


if __name__ == '__main__':
    main()
