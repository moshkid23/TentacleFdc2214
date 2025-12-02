// src/modules/MotionAutomator.h (修正版)

#pragma once

#include <Arduino.h>
#include "../config/constants.h" // 確保引入 constants.h

// 【✅ 新增：停止模式的狀態機 ENUM】
enum StopModeState
{
    NORMAL,
    SLOWING,
    STOPPED,
    RESUMING
};

class MotionAutomator
{
public:
    void begin();

    /**
     * @brief 步驟 1: 更新隨機/停止狀態機和速度因數。
     * @param isRandomMode: 當前是否處於隨機模式。
     */
    // 【✅ 修改：加入 updateState 函式簽名】
    void updateState(bool isRandomMode, unsigned long now);

    // 暫時保持空函式簽名不變，未來會用到
    void calculateTargets(int planned[3], const int currentTarget[3], unsigned long now) {}

    // 【✅ 修改：新增 getSpeedFactor 和 resetStopMode 函式】
    /**
     * @brief 輸出速度因數給 PID 輸出使用 (停止模式)。
     */
    float getSpeedFactor() const { return speedFactor; }

    /**
     * @brief 重置停止模式 (用於 OSC/按鈕切換回常規模式時)。
     */
    void resetStopMode();

    // 暫時保持空 getter
    float getMorphFactor(int i) const { return 1.0f; }
    int getRandomSpeedLimit(int i) const { return 255; }
    bool isMorphing(int i) const { return false; }

private:
    // 【✅ 新增：停止模式的狀態變數 (從 mainModules.cpp 搬過來)】
    StopModeState stopState = NORMAL;
    unsigned long stopTriggerTime = 0;
    float speedFactor = 1.0f; // 停止模式輸出的最終速度倍率
    unsigned long lastCheckTime = 0;
    unsigned long lastTriggerTime = 0;

    // 【✅ 新增：私有功能函式】
    void _updateRandomStopMode(unsigned long now);
};