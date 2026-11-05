// Firefox settings for the DUSK_BOOM ARM STUDIO screen (copied into the profile at every start)
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("startup.homepage_welcome_url", "");
user_pref("startup.homepage_welcome_url.additional", "");
user_pref("browser.aboutwelcome.enabled", false);
user_pref("trailhead.firstrun.didSeeAboutWelcome", true);
user_pref("datareporting.policy.dataSubmissionEnabled", false);
user_pref("datareporting.healthreport.uploadEnabled", false);
user_pref("toolkit.telemetry.enabled", false);
user_pref("app.update.enabled", false);
user_pref("app.update.auto", false);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.sessionstore.max_tabs_undo", 0);
user_pref("browser.tabs.warnOnClose", false);
user_pref("browser.newtabpage.enabled", false);
user_pref("extensions.pocket.enabled", false);
user_pref("network.captive-portal-service.enabled", false);
user_pref("network.connectivity-service.enabled", false);
// 512 MB board: one content process, no site isolation, memory cache only
user_pref("dom.ipc.processCount", 1);
user_pref("dom.ipc.processPrelaunch.enabled", false);
user_pref("fission.autostart", false);
user_pref("browser.cache.disk.enable", false);
user_pref("browser.cache.memory.capacity", 16384);
user_pref("browser.sessionhistory.max_total_viewers", 0);
// no GPU: software WebRender + WebGL on Mesa llvmpipe
user_pref("gfx.webrender.software", true);
user_pref("webgl.force-enabled", true);
user_pref("webgl.disabled", false);
// the starting page (file://) polls http://127.0.0.1:7900
user_pref("security.fileuri.strict_origin_policy", false);
user_pref("intl.accept_languages", "ko-KR, ko, en-US, en");
user_pref("font.name.sans-serif.ko", "NanumGothic");
user_pref("font.name.serif.ko", "NanumMyeongjo");
user_pref("ui.key.menuAccessKeyFocuses", false);
