#include <wifi_helper.h>
#include <user_config.h>
#include <WiFiManager.h>

TaskHandle_t wifiTaskHandle = nullptr;

void wifiReconnectTask(void *param) {
    Serial.println("wifiReconnectTask started");
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        connectToWifi();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void startWifiReconnectTask() {
    xTaskCreate(wifiReconnectTask, "wifiTask", 4096, nullptr, 1, &wifiTaskHandle);
}

void wifiTimerCallback(TimerHandle_t xTimer) {
    if (wifiTaskHandle) {
        xTaskNotifyGive(wifiTaskHandle);  // Notifie la tâche
    }
}

TimerHandle_t wifiReconnectTimer;

ConnState wifiStatus = ConnState::Disconnected;

void initWifi() {
    if (wifiReconnectTimer) {
        xTimerDelete(wifiReconnectTimer, 0);
        wifiReconnectTimer = nullptr;
    }
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(30000), pdFALSE, nullptr,
                                      wifiTimerCallback  // Utilise le nouveau callback
    );
    if (!wifiReconnectTimer) {
        Serial.println("Failed to create WiFi reconnect timer");
    }
    startWifiReconnectTask();
    if (wifiTaskHandle) {
        xTaskNotifyGive(wifiTaskHandle);
    }
}

void connectToWifi() {
    Serial.println("Connecting to Wi-Fi via WiFiManager...");
    wifiStatus = ConnState::Connecting;

    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    wm.setConnectTimeout(22);
    wm.setConfigPortalTimeout(180);
    bool res = wm.autoConnect("velux-rig-setup");

    if (!res) {
        Serial.println("WiFiManager failed to connect");
        wifiStatus = ConnState::Disconnected;

        // Retry later
        if (wifiReconnectTimer) {
            xTimerStart(wifiReconnectTimer, 0);
        }
    } else {
        Serial.printf("Connected to WiFi. IP address: %s\n", WiFi.localIP().toString().c_str());
        wifiStatus = ConnState::Connected;
    }
}

void checkWifiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiStatus == ConnState::Connected) {
            Serial.println("WiFi connection lost");
            wifiStatus = ConnState::Disconnected;

            if (wifiReconnectTimer) {
                xTimerStart(wifiReconnectTimer, 0);
            }
        }
    }
}
