// src/modules/OSCManager.cpp
#include "OSCManager.h"
// #include "../config/constants.h"
#include <functional>
using namespace std::placeholders; // _1, _2

void OSCManager::begin()
{
    // WiFi 設定
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20)
    {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nWiFi failed!");
    }

    udp.begin(OSC_IN_PORT);
    Serial.printf("OSC listening on port %d\n", OSC_IN_PORT);
}

void OSCManager::update()
{
    OSCMessage msg;
    int size = udp.parsePacket();
    if (size > 0)
    {
        while (size--)
            msg.fill(udp.read());
        if (!msg.hasError())
        {
            // 把成員函式 + this + idx 綁定成「只接受 OSCMessage&」的函式指標
            msg.route("/motor1Target",
                      std::bind(&OSCManager::handleMotorTarget, this, _1, 0));
            msg.route("/motor2Target",
                      std::bind(&OSCManager::handleMotorTarget, this, _1, 1));
            msg.route("/motor3Target",
                      std::bind(&OSCManager::handleMotorTarget, this, _1, 2));
            // 可加 /P /I /D
            msg.route("/pwm", std::bind(&OSCManager::handlePwm, this, std::placeholders::_1));
        }
    }
}

void OSCManager::handlePwm(OSCMessage &msg)
{
    if (msg.isInt(0))
    {
        pwmValue = constrain(msg.getInt(0), 0, 255);
    }
    else if (msg.isFloat(0))
    {
        // 有些軟體會送 0.0~1.0 的 float，保險起見也處理一下
        pwmValue = constrain((int)(msg.getFloat(0)), 0, 255);
    }
}

void OSCManager::handleMotorTarget(OSCMessage &msg, int idx)
{
    if (msg.isInt(0))
    {
        oscTargets[idx] = constrain(msg.getInt(0), 0, MAX_ENCODER);
        // Serial.printf("OSC Target M%d = %d\n", idx + 1, oscTargets[idx]);
    }
}

int OSCManager::getTarget(int motor) const
{
    return oscTargets[motor];
}

void OSCManager::sendStatus(const int pos[3])
{
    OSCMessage msg("/status");
    msg.add(pos[0]).add(pos[1]).add(pos[2]);
    udp.beginPacket(OSC_OUT_IP, OSC_OUT_PORT);
    msg.send(udp);
    udp.endPacket();
}