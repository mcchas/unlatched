#ifndef IOHC_USER_H
#define IOHC_USER_H

#include <board_config.h>

// WiFi credentials are handled via WiFiManager
#define WIFI_SSID ""
#define WIFI_PASSWD ""
inline const char *wifi_ssid = "";
inline const char *wifi_passwd = "";

/* The broker and the web UI's password used to be compiled in from here. They
 * are settings now -- NVS, editable from the Settings tab or the console -- so
 * the constants have been removed rather than left to be edited by someone
 * reasonably expecting them to take effect. A rig that is reflashed over the
 * air should not need a rebuild to be told where its broker is.
 *
 * See app_settings.h for what is stored, and mqtt_bridge.h for the topics. */

#if defined(ESP8266)
#define SERIALSPEED 460800
#elif defined(ESP32)
#define SERIALSPEED 115200  //921600
#endif

#endif
