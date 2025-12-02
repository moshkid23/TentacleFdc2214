// src/modules/MotionAutomator.cpp (實現停止模式邏輯)

#include "MotionAutomator.h"
#include <esp_system.h> // for esp_random()
#include <algorithm>    // for std::max, std::min (雖然這個邏輯用 if/else 也可以，但建議保留)

// ... (PI 定義保持不變)

void MotionAutomator::begin()
{
    randomSeed(esp_random());
}

/**
 * @brief 重設停止模式 (用於 OSC/按鈕切換回常規模式時)
 */
// 【✅ 實現 resetStopMode 函式】
void MotionAutomator::resetStopMode()
{
    stopState = NORMAL;
    speedFactor = 1.0f;
    stopTriggerTime = 0;
}

/**
 * @brief 實現原 updateRandomStopMode 邏輯
 */
void MotionAutomator::_updateRandomStopMode(unsigned long now)
{
    // === 1. 僅在 NORMAL 狀態且到達檢查時間才檢查 ===
    if (stopState == NORMAL && now - lastCheckTime >= CHECK_INTERVAL_MS)
    {
        lastCheckTime = now;
        // 檢查最小間隔
        if (now - lastTriggerTime >= MIN_STOP_INTERVAL_MS && random(100) < STOP_CHANCE)
        {
            stopState = SLOWING;
            stopTriggerTime = now;
            lastTriggerTime = now;
            // Serial.println("Random STOP triggered!"); // 可選：遷移 log
        }
    }

    // === 2. 狀態機邏輯 (直接複製原 mainModules.cpp 的邏輯) ===
    switch (stopState)
    {
    case NORMAL:
        speedFactor = 1.0f;
        break;

    case SLOWING:
    {
        float t = (now - stopTriggerTime) / (float)SLOW_DOWN_MS;
        if (t >= 1.0f)
        {
            speedFactor = 0.0f;
            stopState = STOPPED;
            stopTriggerTime = now;
        }
        else
        {
            speedFactor = 1.0f - t; // 線性減速
        }
    }
    break;

    case STOPPED:
        if (now - stopTriggerTime >= STOP_DURATION_MS)
        {
            stopState = RESUMING;
            stopTriggerTime = now;
        }
        speedFactor = 0.0f;
        break;

    case RESUMING:
    {
        float t = (now - stopTriggerTime) / (float)SPEED_UP_MS;
        if (t >= 1.0f)
        {
            speedFactor = 1.0f;
            stopState = NORMAL;
        }
        else
        {
            speedFactor = t;
        }
    }
    break;
    }

    // 強制限制速度因數在 [0.0, 1.0] 之間
    speedFactor = std::max(0.0f, std::min(speedFactor, 1.0f));
}

/**
 * @brief 檢查並更新所有狀態機和因數 (在 loop() 開頭呼叫)
 */
// 【✅ 實現 updateState 函式】
void MotionAutomator::updateState(bool isRandomMode, unsigned long now)
{
    // === 1. OSC 模式強制關閉停止效果 (這是 mainModules 原有的邏輯) ===
    if (!isRandomMode)
    {
        resetStopMode();
        return;
    }

    // === 2. 運行隨機停止狀態機 ===
    _updateRandomStopMode(now);
}