#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>
#include "MyMQTT.h"
#include "AccessPoint.h"

#define LED_PIN 2 // Define the LED

AccessPoint accessPoint;

// Task 1: Handle MQTT
void Task1(void *pvParameters)
{
  mqttClient.setup();
  // mqttClient.subscribe("v1/devices/me/telemetry"); // Subscribe to a topic
  while (1)
  {
    mqttClient.loop();

    // Create a JSON document
    StaticJsonDocument<200> doc;
    doc["test_key"] = "test_value";

    // Serialize JSON to string
    char buffer[256];
    serializeJson(doc, buffer);

    // Publish to a topic
    mqttClient.publish("v1/devices/me/telemetry", buffer);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

// Task 2: Blink LED
void Task2(void *pvParameters)
{
  pinMode(LED_PIN, OUTPUT); // Initialize the LED pin as an output
  while (1)
  {
    digitalWrite(LED_PIN, HIGH);          // Turn the LED on
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500 milliseconds
    digitalWrite(LED_PIN, LOW);           // Turn the LED off
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500 milliseconds
  }
}

// Task 3: Handle Access Point Client Requests
void Task3(void *pvParameters)
{
  while (1)
  {
    accessPoint.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10 milliseconds to yield to other tasks
  }
}

void setup()
{
  // Start the Serial communication at a baud rate of 9600
  Serial.begin(9600);

  // Thiết lập Access Point
  accessPoint.setupAP();

  // Create Task 1
  xTaskCreate(Task1, "Task 1", 10000, NULL, 1, NULL);

  // Create Task 2
  xTaskCreate(Task2, "Task 2", 1000, NULL, 1, NULL);

  // Create Task 3
  xTaskCreate(Task3, "Task 3", 10000, NULL, 1, NULL);
}

void loop()
{
  // Do nothing in the loop
}