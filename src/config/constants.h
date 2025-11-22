// config/constants.h
#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// WiFi & OSC
constexpr const char *WIFI_SSID = "Perform2";
constexpr const char *WIFI_PASS = "qqqqqqq2";
inline const IPAddress LOCAL_IP(192, 168, 1, 30);
inline const IPAddress GATEWAY(192, 168, 1, 1);
inline const IPAddress SUBNET(255, 255, 255, 0);
constexpr int OSC_IN_PORT = 9100;
constexpr int OSC_OUT_PORT = 9101;
inline const IPAddress OSC_OUT_IP(192, 168, 1, 200);

// // 馬達腳位
// struct MotorPinConfig
// {
//     int in1, in2, pwm, encA, encB, pwmChannel;
// };
// constexpr MotorPinConfig MOTORS[3] = {
//     {19, 18, 23, 32, 33, 0},
//     {17, 16, 5, 25, 26, 1},
//     {2, 15, 4, 27, 14, 2}};

// // 張力安全
constexpr int MAX_ENCODER = 17000;
// constexpr int LIMIT_HIGH = 12000;       // x=17000 時，其他至少 12000
// constexpr int LIMIT_MID = 5000;         // x=8500  時，其他至少 5000
// constexpr float TENSION_CURVE_K = 1.3f; // 曲線係數
// constexpr int MASTER_HYSTERESIS = 1200; // 主軸切換遲滯，避免頻繁跳軸（可調 800~2000）

// // 停止模式
// constexpr int STOP_CHANCE = 5;              // 每次檢查有 5% 機率觸發
// constexpr int CHECK_INTERVAL_MS = 3000;     // 每 3 秒檢查一次
// constexpr int MIN_STOP_INTERVAL_MS = 10000; // 觸發後至少 10 秒不重複
// constexpr int SLOW_DOWN_MS = 8000;
// constexpr int STOP_DURATION_MS = 1500;
// constexpr int SPEED_UP_MS = 8000;

// // 隨機模式
// constexpr int RANDOM_UPDATE_MIN_MS = 3000;  // 中心位置、速率變化最低更新秒數
// constexpr int RANDOM_UPDATE_MAX_MS = 12000; // 中心位置、速率變化最高更新秒數
// constexpr int RANDOM_WAVE_AMPLITUDE = 3000; // 中心位置上下振幅
// constexpr int RANDOM_LERP_T = 5;            // 0.05慢 ~ 0.15快

// // 電容
// constexpr int CAP_READ_INTERVAL_MS = 40;
// constexpr int CAP_STABLE_DELAY_MS = 50;         // 停止/運動狀態切換後，等待穩定的延遲時間
// constexpr int CAP_TOUCH_THRESHOLD_STATIC = 200; // 靜止時觸摸閾值
// constexpr int CAP_TOUCH_THRESHOLD_MOVING = 300; // 運動時觸摸閾值
// constexpr int CAP_MEDIAN_WINDOW = 5;            // 取中位數
// constexpr float CAP_EMA_ALPHA = 0.30f;          // EMA 反應速度：0.2 快、0.1 慢，想再快一點：0.3~0.4。

// // 其他
// constexpr int MOTION_TIMEOUT_MS = 40; // 停下後，距離上次運動過超過 40ms 才判斷為靜止
// constexpr int LED_PIN = 12;
// constexpr int LED_PWM_CHANNEL = 3;
// constexpr int BUTTON_PIN = 13;
// constexpr int CAP_I2C_SDA = 21;
// constexpr int CAP_I2C_SCL = 22;