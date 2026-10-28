#!/bin/bash
# Double-click to set up (first time) and open the active aero simulator.
cd "$(dirname "$0")" || exit 1
python3 aero.py "$@" || read -r -p "Press Enter to close..."
