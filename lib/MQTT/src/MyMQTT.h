#ifndef MYMQTT_H
#define MYMQTT_H

#include <WiFi.h>
#include <PubSubClient.h>
#include "WiFiCredentials.h"
#include "MQTTCredentials.h"

class MyMQTT
{
public:
    MyMQTT();
    void setup();
    void loop();
    void publish(const char *topic, const char *payload);
    void subscribe(const char *topic);

private:
    const char *ssid = WIFI_SSID;
    const char *password = WIFI_PASSWORD;
    const char *mqtt_server = MQTT_SERVER;
    const char *mqtt_username = MQTT_USERNAME;
    const char *mqtt_password = MQTT_PASSWORD;
    WiFiClient espClient;
    PubSubClient client;
    void reconnect();
    static void callback(char *topic, byte *payload, unsigned int length);
};

// Declare the global instance of MyMQTT
extern MyMQTT mqttClient;

#endif // MYMQTT_H