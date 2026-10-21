#!/usr/bin/env python3
"""
Check the CAD setup and print what the simulator measured from it.

  python3 sim/plant/cad_check.py              uses sim/dashboard/models/boat.json
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from alan_plant.cad import load  # noqa: E402

models = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "dashboard", "models")
overrides, report, hydro = load(models)
if not report:
    print(f"No boat.json in {os.path.abspath(models)} (see the README there)")
for line in report:
    print(line)
