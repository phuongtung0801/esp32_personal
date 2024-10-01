#include "AccessPoint.h"

AccessPoint::AccessPoint() : server(80) {}

void AccessPoint::setupAP()
{
    // Thiết lập Access Point
    WiFi.softAP("ESP32-AccessPoint", "12345678");

    // In địa chỉ IP của Access Point
    Serial.print("Access Point IP address: ");
    Serial.println(WiFi.softAPIP());

    // Khởi tạo SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Cấu hình các route cho máy chủ web
    server.on("/", HTTP_GET, std::bind(&AccessPoint::handleRoot, this));
    server.on("/save", HTTP_POST, std::bind(&AccessPoint::handleSave, this));
    server.onNotFound(std::bind(&AccessPoint::handleNotFound, this));

    // Bắt đầu máy chủ web
    server.begin();
}

void AccessPoint::handleClient()
{
    server.handleClient();
}

void AccessPoint::handleRoot()
{
    String html = "<html><body><h1>ESP32 Access Point</h1>";
    html += "<form action='/save' method='POST'>";
    html += "SSID: <input type='text' name='ssid'><br>";
    html += "Password: <input type='text' name='password'><br>";
    html += "<input type='submit' value='Save'>";
    html += "</form></body></html>";
    server.send(200, "text/html", html);
}

void AccessPoint::handleNotFound()
{
    server.send(404, "text/plain", "404: Not found");
}

void AccessPoint::handleSave()
{
    if (server.hasArg("ssid") && server.hasArg("password"))
    {
        String ssid = server.arg("ssid");
        String password = server.arg("password");

        // Lưu thông tin WiFi vào SPIFFS
        File file = SPIFFS.open("/wifi_config.txt", FILE_WRITE);
        if (!file)
        {
            Serial.println("Failed to open file for writing");
            server.send(500, "text/plain", "Failed to save WiFi credentials");
            return;
        }
        file.println(ssid);
        file.println(password);
        file.close();

        // Gửi phản hồi cho người dùng
        server.send(200, "text/plain", "WiFi credentials saved. Please restart the ESP32.");

        // In ra Serial để kiểm tra
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("Password: ");
        Serial.println(password);
    }
    else
    {
        server.send(400, "text/plain", "Invalid request");
    }
}