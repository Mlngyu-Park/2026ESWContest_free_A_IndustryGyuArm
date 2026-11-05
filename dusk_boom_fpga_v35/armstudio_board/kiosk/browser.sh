#!/bin/sh
# full-screen Firefox: first the local "starting" page, which switches to ARM STUDIO by itself
xset s off s noblank -dpms 2>/dev/null
matchbox-window-manager -use_titlebar no -use_cursor yes &
P=/var/lib/dusk/firefox
mkdir -p $P && cp -f /opt/dusk/kiosk/user.js $P/user.js
export MOZ_DISABLE_CONTENT_SANDBOX=1 MOZ_DISABLE_GMP_SANDBOX=1 MOZ_DISABLE_RDD_SANDBOX=1
export MOZ_DISABLE_SOCKET_PROCESS_SANDBOX=1 MOZ_CRASHREPORTER_DISABLE=1 LIBGL_ALWAYS_SOFTWARE=1
exec firefox-esr --kiosk --no-remote --profile $P file:///opt/dusk/kiosk/loading.html
