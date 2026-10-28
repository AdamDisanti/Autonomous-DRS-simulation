#!/bin/bash
# Double-click to set up (first time) and open the active aero simulator.
cd "$(dirname "$0")" || exit 1

# Use the newest Python >= 3.11. Plain `python3` is often Apple's older 3.9 even
# after installing a newer one from python.org or Homebrew.
PY=""
for p in python3.14 python3.13 python3.12 python3.11 python3 \
         /Library/Frameworks/Python.framework/Versions/Current/bin/python3 /opt/homebrew/bin/python3 /usr/local/bin/python3; do
  if command -v "$p" >/dev/null 2>&1 && "$p" -c 'import sys; sys.exit(sys.version_info < (3, 11))' 2>/dev/null; then
    PY="$p"; break
  fi
done
if [ -z "$PY" ]; then
  echo "Python 3.11 or newer was not found."
  echo "Install the latest Python 3 from https://www.python.org/downloads/ and double-click this file again."
  read -r -p "Press Enter to close..."
  exit 1
fi

"$PY" aero.py "$@" || read -r -p "Press Enter to close..."
