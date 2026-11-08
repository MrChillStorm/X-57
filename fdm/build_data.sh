#!/bin/sh
# Rebuild fdm/data from NASA's files in refs/, then check it.
set -e
cd "$(dirname "$0")"
PY=${PYTHON:-/opt/homebrew/bin/python3}
"$PY" geometry.py > /dev/null
"$PY" cfd.py
"$PY" digitise_fq.py > /dev/null
"$PY" digitise_tunnel.py > /dev/null
"$PY" hlp_data.py > /dev/null
"$PY" digitise_hlp.py > /dev/null
"$PY" phase0_checks.py
"$PY" aerotables.py > /dev/null
"$PY" discs.py > /dev/null
rm -f data/x57_flex_fit.json
"$PY" x57flex.py
"$PY" x57fit.py
"$PY" mkfg.py
"$PY" mkwind.py
"$PY" mkgear.py
"$PY" x57sound.py
