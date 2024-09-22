#include "MyMQTT.h"

// Define the global instance of MyMQTT
MyMQTT mqttClient;

MyMQTT::MyMQTT() : client(espClient) {}

void MyMQTT::setup()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP()); // Print the local IP address
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback); // Register the callback function
}

void MyMQTT::loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
}

void MyMQTT::publish(const char *topic, const char *payload)
{
    client.publish(topic, payload);
}

void MyMQTT::subscribe(const char *topic)
{
    client.subscribe(topic);
}

void MyMQTT::reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect with username and password
        if (client.connect("ESP32Client", mqtt_username, mqtt_password))
        {
            Serial.println("connected");
            client.publish("outTopic", "hello world");
            // Resubscribe to topics
            subscribe("inTopic");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void MyMQTT::callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}