#!/bin/bash
# 하강 높이 검증 (시뮬) — 하강 직후 상태를 만들어 3D 로 보여준다
cd "$(dirname "$0")"
exec conda run --no-capture-output -n gyu_arm python test_down.py "$@"
