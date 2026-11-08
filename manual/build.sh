#!/bin/sh
# Rebuild the flight manual (its cockpit tables come from fdm/cockpit-guide.md) and copy the PDF to the repository root, beside the model's report.
set -e
cd "$(dirname "$0")"
/opt/homebrew/bin/python3 mkmanual.py
tectonic -X compile x57_manual.tex
cp x57_manual.pdf ../X-57-flight-manual.pdf
