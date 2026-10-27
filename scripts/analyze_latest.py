#!/usr/bin/env python3
"""Analyze the newest completed experiment without confusing setup logs for runs."""
import json
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
runs = sorted(root.glob('results/*/metadata.json'))
if not runs:
    raise SystemExit('No experiment run found')
latest = runs[-1]
if json.loads(latest.read_text())['status'] != 'completed':
    raise SystemExit(f'Latest experiment did not complete: {latest.parent}')
subprocess.run([sys.executable, str(root / 'scripts/analyze.py'), str(latest.parent)], check=True)
