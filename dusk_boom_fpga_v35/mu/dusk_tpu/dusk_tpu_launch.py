#!/usr/bin/env python3
"""dusk_tpu_launch.py — 뮤(라떼판다)에서 ARM STUDIO PRO 를 'FPGA TPU 화면' 과 함께 띄우는 실행기 (2026-11-05)

  원본(arm_studio_pro.py · arm_studio_web.py · nano_double/rachel2.py)은 한 줄도 고치지 않는다.
  여기서 불러온 뒤 함수 몇 개만 감싼다:
    ① 말로 만들기(_llm_parse): AI 해석 동안 FPGA 화면에 '말 → 블록' 연산. 결과(블록)는 최소 4초·보드 계산이
       끝날 때까지 붙잡았다가 '계산 완료' 와 동시에 넘긴다 → 그 뒤 블록이 지금처럼 하나씩 붙는다.
    ② 더블 레이첼(Rachel2.raw): 같은 문장을 보드 TPU 에도 보내 실제로 계산하게 한다.
    ③ 음성 인식(/api/stt, 미리보기 제외): 같은 규칙으로 4초 — Whisper(Spark) 또는 Moonshine(뮤).
    ④ 로봇 포트 자동 찾기에서 보드의 USB-UART(CP2102N)를 뺀다 (보드 셸에 로봇 명령이 가지 않게).
  실행: python -u dusk_tpu_launch.py arm_studio_pro.py   (start_pro_tpu.sh 가 부른다)
"""
import asyncio
import json
import os
import runpy
import socket
import sys

APP = os.environ.get("ARM_STUDIO_DIR", os.path.expanduser("~/2026_project/Industry_gyu_arm"))
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(APP)
sys.path.insert(0, APP)
sys.path.insert(1, HERE)

import tpu_bridge as TB  # noqa: E402
import arm_studio_web as core  # noqa: E402   (arm_studio_pro 가 import 할 때 이 모듈을 그대로 쓴다)

# ---- ④ 로봇 포트에서 FPGA 빼기 -------------------------------------------------------------------
_scan0 = core.scan_serial


def _scan_serial():
    f = _scan0()
    fpga = TB.fpga_ports()
    for k, v in list(f.items()):
        if v and (v in fpga or os.path.realpath(v) in fpga):
            f[k] = None
    return f


core.scan_serial = _scan_serial

# ---- ② 더블 레이첼 문장을 보드 TPU 로도 --------------------------------------------------------------
sys.path.insert(0, os.path.join(APP, "nano_double"))
try:
    import rachel2  # noqa: E402
    _raw0 = rachel2.Rachel2.raw

    def _raw(self, scene, text):
        s = TB.current()
        if s is not None and not s.prompt_sent:       # 첫 번째 = 본 답 (다수결 보충은 보내지 않는다)
            try:
                s.prompt(f"{scene}\n{self._fit(' '.join(text.split()))}\n>")
            except Exception as e:  # noqa: BLE001
                print("[TPU 화면] 문장 보내기 실패:", e)
        return _raw0(self, scene, text)

    rachel2.Rachel2.raw = _raw
except Exception as e:  # noqa: BLE001
    print("[TPU 화면] rachel2 감싸기 실패 (보드는 모양만 보여준다):", e)

# ---- ① 말로 만들기 ---------------------------------------------------------------------------------
_llm0 = core._llm_parse


def _llm_label():
    eng = core.LLM_CFG.get("engine", "auto")
    if eng == "rachel2" or (eng == "auto" and not core.LLM_CFG.get("url")):
        return "rachel", "더블 레이첼 · 25.6M · int8"
    if eng in ("rachel", "rachel_py"):
        return "llm", "RACHEL v9 · 10M"
    return "llm", f"{core.LLM_CFG.get('model') or 'Gemma'} · DGX Spark"


def _llm_parse(t):
    if core.LLM_CFG.get("engine", "auto") == "rule":
        return _llm0(t)                         # AI 안 씀 — 화면도 안 돈다
    kind, model = _llm_label()
    with TB.session(kind, "말 → 블록", model):
        return _llm0(t)                         # 나올 때 finish(): 4초·보드 계산 끝까지 기다림


core._llm_parse = _llm_parse
# 11-05 (사용자): 말로 만들기 = 더블 레이첼 (뮤가 계산), 음성 인식만 Spark Whisper (노트북을 거쳐 127.0.0.1:18090)
core.LLM_CFG["engine"] = os.environ.get("DUSK_LLM_ENGINE", "rachel2")

# ---- ③ 음성 인식 -----------------------------------------------------------------------------------


def _stt_model():
    main = sys.modules.get("__main__")
    pick = (getattr(main, "STT_PICK", ["auto"]) or ["auto"])[0]
    if pick != "moonshine":
        try:
            with socket.create_connection(("127.0.0.1", 18090), timeout=0.25):
                return "spark", "Whisper large-v3-turbo · DGX Spark"
        except OSError:
            pass
    return "moonshine", "Moonshine tiny · 한국어"


def _wrap(app):
    async def wrapped(scope, receive, send):
        if (scope.get("type") != "http" or scope.get("path") != "/api/stt"
                or b"preview" in (scope.get("query_string") or b"")):
            return await app(scope, receive, send)
        loop = asyncio.get_running_loop()
        eng, model = _stt_model()
        await loop.run_in_executor(None, TB._LOCK.acquire)
        try:
            sess = await loop.run_in_executor(None, TB.Session, "stt", "음성 인식", model)
            msgs = []

            async def cap(m):
                msgs.append(m)
            await app(scope, receive, cap)
            try:
                body = b"".join(m.get("body", b"") for m in msgs if m.get("type") == "http.response.body")
                j = json.loads(body)
                if j.get("engine") and j.get("engine") != eng:
                    sess.relabel("Moonshine tiny · 한국어" if j["engine"] == "moonshine"
                                 else "Whisper large-v3-turbo · DGX Spark")
                if j.get("text"):
                    sess.text(j["text"])
            except Exception:  # noqa: BLE001
                pass
            await loop.run_in_executor(None, sess.finish)
        finally:
            TB._LOCK.release()
        for m in msgs:
            await send(m)
    return wrapped


import uvicorn  # noqa: E402

_run0 = uvicorn.run


def _run(app, *a, **k):
    return _run0(_wrap(app), *a, **k)


uvicorn.run = _run

print("[TPU 화면] FPGA 보드:", TB._port() or "없음 (꽂으면 다음 추론부터 화면이 움직입니다)",
      f"· 최소 {TB.MIN_S:.0f}초")

# ---- 원본 실행 -------------------------------------------------------------------------------------
sys.argv = [os.path.join(APP, "arm_studio_pro.py")] + sys.argv[2:]
runpy.run_path(os.path.join(APP, "arm_studio_pro.py"), run_name="__main__")
