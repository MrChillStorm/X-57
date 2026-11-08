#!/bin/sh
# Refit the model, then run every check and write the FlightGear package, four processes at a time, a fair share of a queue slot:
#   qrun fdm/check_all.sh [log folder]
# Each process builds in its own folder. The longest go first. Logs go to the folder, default fdm/out/checks.
set -e
cd "$(dirname "$0")"
PY=${PYTHON:-/opt/homebrew/bin/python3}
L=${1:-out/checks}
mkdir -p "$L"
"$PY" x57fit.py > "$L/fit.log" 2>&1
echo fit done
export PY L
printf '%s\n' 'p2 1 phase2_checks.py' 'p1 1 phase1_checks.py' 'fly 1 fly.py' 'p5 1 phase5_checks.py' 'p6 1 phase6_checks.py' 'p7 1 phase7_checks.py' 'p3 0 phase3_checks.py' 'p4 1 phase4_checks.py' 'mkfg 0 mkfg.py' |
    xargs -P 4 -L 1 sh -c 'if [ "$1" = 1 ]; then export X57_BUILT=1; fi; X57_ROOT_TAG=-$0 "$PY" "$2" > "$L/$0.log" 2>&1; echo "$0 $?"'
X57_ROOT_TAG=-sound "$PY" soundcheck.py > "$L/sound.log" 2>&1
echo sound $?
if "$PY" pickcheck.py > "$L/picks.log" 2>&1; then echo picks 0; else echo picks 1; fi
if "$PY" readcheck.py > "$L/reads.log" 2>&1; then echo reads 0; else echo reads 1; fi
rm -rf root-*-p[1-6] root-*-fly root-*-mkfg root-*-sound
