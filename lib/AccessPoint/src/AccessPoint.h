#ifndef ACCESS_POINT_H
#define ACCESS_POINT_H

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

class AccessPoint
{
public:
    AccessPoint();
    void setupAP();
    void handleClient();

private:
    WebServer server;
    void handleRoot();
    void handleNotFound();
    void handleSave();
};

#endif // ACCESS_POINT_H