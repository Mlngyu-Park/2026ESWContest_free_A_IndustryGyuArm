#!/usr/bin/env python3
"""auto_mirror.py — 뮤: 모니터를 아무거나 꽂으면 자동으로 '복제(미러)' (2026-11-06)

  GNOME 에는 '어떤 모니터든 늘 복제' 설정이 없다 (모니터 조합마다 따로 기억함).
  → 화면 관리자(Mutter DisplayConfig)의 MonitorsChanged 신호를 받아, 화면이 2개 이상이면
    모두가 지원하는 가장 큰 공통 해상도로 하나의 화면(복제)으로 묶는다. 이미 복제면 아무것도 안 한다.
  적용은 '영구(persistent)' 로 — GNOME 이 그 조합을 기억해서 이 프로그램이 없어도 다음에 복제로 켜진다.
  로그인 때 ~/.config/autostart/auto-mirror.desktop 이 띄운다. 기록: /tmp/auto_mirror.log
  시험: python3 auto_mirror.py --dry   (지금 상태와 고를 해상도만 보여 주고 바꾸지 않음)
"""
import sys
import time
from gi.repository import Gio, GLib

DRY = "--dry" in sys.argv
LOG = "/tmp/auto_mirror.log"


def log(*a):
    line = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a)
    print(line, flush=True)
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def proxy():
    bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)
    return Gio.DBusProxy.new_sync(bus, 0, None, "org.gnome.Mutter.DisplayConfig",
                                  "/org/gnome/Mutter/DisplayConfig", "org.gnome.Mutter.DisplayConfig", None)


def pick_modes(mons):
    """모든 모니터가 가진 해상도 중 가장 큰 것. 모니터마다 그 해상도의 모드(지금 쓰는 것 > 주사율 높은 것)."""
    sizes = None
    for _spec, modes, _p in mons:
        s = {(m[1], m[2]) for m in modes}
        sizes = s if sizes is None else sizes & s
    if not sizes:
        return None, None
    w, h = max(sizes, key=lambda wh: (wh[0] * wh[1], wh[0]))
    out = []
    for spec, modes, _p in mons:
        cand = [m for m in modes if (m[1], m[2]) == (w, h)]
        cand.sort(key=lambda m: (bool(m[6].get("is-current")), bool(m[6].get("is-preferred")), m[3]), reverse=True)
        out.append((spec[0], cand[0][0], cand[0]))
    return (w, h), out


def apply_mirror(p):
    serial, mons, logical, props = p.call_sync("GetCurrentState", None, 0, -1, None).unpack()
    if len(mons) < 2:
        return
    conns = sorted(spec[0] for spec, _m, _p in mons)
    if len(logical) == 1 and sorted(m[0] for m in logical[0][5]) == conns:
        return                                            # 이미 복제
    size, chosen = pick_modes(mons)
    if not chosen:
        log("공통 해상도가 없어 복제 못 함:", conns)
        return
    scale = 1.0
    if not all(scale in c[2][5] for c in chosen):         # 1.0 을 못 쓰는 모드면 공통 배율 중 가장 작은 것
        common = set(chosen[0][2][5])
        for c in chosen[1:]:
            common &= set(c[2][5])
        if not common:
            log("공통 배율이 없어 복제 못 함:", conns)
            return
        scale = min(common)
    log(f"복제: {conns} → {size[0]}×{size[1]} (배율 {scale})" + ("  [시험 — 바꾸지 않음]" if DRY else ""))
    if DRY:
        return
    lm = [(0, 0, scale, 0, True, [(c[0], c[1], {}) for c in chosen])]
    try:
        p.call_sync("ApplyMonitorsConfig",
                    GLib.Variant("(uua(iiduba(ssa{sv}))a{sv})", (serial, 2, lm, {})), 0, -1, None)
    except GLib.Error as e:
        log("복제 적용 실패:", e.message)


def main():
    p = proxy()
    pending = {"id": 0}

    def later():
        pending["id"] = 0
        try:
            apply_mirror(p)
        except Exception as e:                            # noqa: BLE001
            log("오류:", e)
        return False

    def on_signal(_p, _sender, name, _params):
        if name == "MonitorsChanged" and not pending["id"]:
            pending["id"] = GLib.timeout_add(1500, later)   # 꽂은 직후 모드 목록이 자리 잡을 때까지 잠깐

    try:
        serial, mons, logical, props = p.call_sync("GetCurrentState", None, 0, -1, None).unpack()
        log("시작 — 화면:", [spec[0] for spec, _m, _p in mons])
        if DRY:
            size, chosen = pick_modes(mons)
            log("지금 고를 공통 해상도:", size, [(c[0], c[1]) for c in chosen or []])
        apply_mirror(p)
    except Exception as e:                                # noqa: BLE001
        log("오류:", e)
    if DRY:
        return
    p.connect("g-signal", on_signal)
    GLib.MainLoop().run()


if __name__ == "__main__":
    main()
