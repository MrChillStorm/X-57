#!/usr/bin/env python3
"""The report's numbers and its summary table, from the check logs in results/."""
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, 'results')
OUT = os.path.join(HERE, 'tab')
os.makedirs(OUT, exist_ok=True)


def log(n):
    return open(os.path.join(R, n + '.log'), errors='replace').read()


def count(n):
    s = log(n)
    ok = len(re.findall(r'^ok\b', s, re.M)) + len(re.findall(r'\sok\s*$', s, re.M))
    miss = len(re.findall(r'^MISS\b', s, re.M)) + len(re.findall(r'\sMISS \(limit[^)]*\)\s*$', s, re.M))
    return ok, miss


PHASES = (('p1', 'Modes and CFD increments, built-up model (limits written for the table model)', 'C, S'), ('p2', 'Propellers, motors, governor, battery', 'M, S'),
          ('p3', 'P2006T: flight manual, flight tests, tunnel, Mod II simulator', 'M, S'), ('p4', 'X-57 built-up aerodynamics against CFD', 'C'),
          ('p5', 'Ground effect, climbs, failures, gear', 'C, S, M'), ('p6', 'Wing bending', 'M, S'), ('p7', 'Cockpit power and propeller modes', 'D'))
with open(os.path.join(OUT, 'tab_checks.tex'), 'w') as f:
    f.write('\\begin{tabular}{@{}p{50mm}lrr@{}}\n\\toprule\nChecks & Against & Pass & Miss\\\\\n\\midrule\n')
    T = [0, 0]
    for n, name, against in PHASES:
        a, b = count(n)
        T[0] += a
        T[1] += b
        f.write('%s & %s & %d & %d\\\\\n' % (name, against, a, b))
    f.write('\\midrule\nAll & & %d & %d\\\\\n\\bottomrule\n\\end{tabular}\n' % tuple(T))
print('checks', T)
with open(os.path.join(OUT, 'numbers.tex'), 'w') as f:
    f.write('\\newcommand{\\Npass}{%d}\n\\newcommand{\\Nmiss}{%d}\n' % tuple(T))
    for n, _, _ in PHASES:
        f.write('\\newcommand{\\Miss%s}{%d}\n' % (n[1].translate(str.maketrans('1234567', 'ABCDEFG')), count(n)[1]))
