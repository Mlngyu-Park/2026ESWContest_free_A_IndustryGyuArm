#!/bin/sh
cd "$(dirname "$0")"
fuser -k 8765/tcp 2>/dev/null
python3 serve.py
