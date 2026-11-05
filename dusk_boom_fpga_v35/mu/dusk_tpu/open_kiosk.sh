#!/bin/bash
# open_kiosk.sh — ARM STUDIO 를 전체 화면(Firefox 키오스크)으로 연다 (뮤, 2026-11-05)
#   평소 Firefox 와 따로 도는 전용 프로필 (snap 이 쓸 수 있는 ~/snap/firefox/common 안).
#   끄기: Alt+F4.  마이크는 묻지 않고 허용 (말로 만들기 음성).  이미 떠 있으면 새로 안 띄운다.
URL="${1:-http://localhost:7900}"
PROF="$HOME/snap/firefox/common/armstudio-kiosk"
mkdir -p "$PROF"
cat > "$PROF/user.js" <<'PREFS'
user_pref("media.navigator.permission.disabled", true);
user_pref("permissions.default.microphone", 1);
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.aboutwelcome.enabled", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("startup.homepage_welcome_url", "");
user_pref("startup.homepage_welcome_url.additional", "");
user_pref("trailhead.firstrun.didSeeAboutWelcome", true);
user_pref("datareporting.policy.dataSubmissionPolicyBypassNotification", true);
user_pref("toolkit.telemetry.reportingpolicy.firstRun", false);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.tabs.warnOnClose", false);
user_pref("browser.translations.automaticallyPopup", false);
PREFS
if pgrep -f "lib/firefox/firefox .*armstudio-kiosk" >/dev/null; then   # 진짜 firefox 프로세스만 (경로 글자 우연 일치 방지)
    exit 0
fi
exec firefox --new-instance --profile "$PROF" --kiosk "$URL" >/dev/null 2>&1
