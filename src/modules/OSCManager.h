// src/modules/OSCManager.h
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include <functional> // std::bind, std::placeholders
#include <OSCMessage.h>
#include "../config/constants.h"

class OSCManager
{
public:
    void begin();
    void update();                     // 每 loop 呼叫
    int getTarget(int motor) const;    // 取得 OSC 目標
    void sendStatus(const int pos[3]); // 發送位置（可選）

private:
    WiFiUDP udp;
    int oscTargets[3] = {0, 0, 0}; // 最新目標

    void handleMotorTarget(OSCMessage &msg, int idx);

    void motor1Callback(OSCMessage &msg) { handleMotorTarget(msg, 0); }
    void motor2Callback(OSCMessage &msg) { handleMotorTarget(msg, 1); }
    void motor3Callback(OSCMessage &msg) { handleMotorTarget(msg, 2); }
};