#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <WiFi.h>
#include <WiFiManager.h>
#include <interact.h>

extern TimerHandle_t wifiReconnectTimer;
extern ConnState wifiStatus;

void initWifi();
void connectToWifi();
void checkWifiConnection();

#endif  //WIFI_HELPER_H