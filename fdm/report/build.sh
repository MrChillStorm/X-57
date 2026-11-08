#!/bin/sh
# Rebuild the report from the check logs in results/ (copied from a run of check_all.sh; rerun mkresults.py for the failure's time histories)
# and copy the PDF to the repository root.
set -e
cd "$(dirname "$0")"
/opt/homebrew/bin/python3 mktables.py
/opt/homebrew/bin/python3 figures.py
tectonic -X compile x57_report.tex
cp x57_report.pdf ../../X-57-JSBSim-model-report.pdf
