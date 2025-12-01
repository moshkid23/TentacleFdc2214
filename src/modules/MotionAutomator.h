// src/modules/MotionAutomator.h (修正版)

#pragma once

#include <Arduino.h>
#include "../config/constants.h"

// 停止模式的狀態機
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
    void updateState(bool isRandomMode, unsigned long now);

    /**
     * @brief 步驟 2: 計算和應用目標位置。
     * @param planned[3]: 經計算後寫入的目標位置 (OUTPUT)。
     * @param currentTarget[3]: 當前馬達的目標位置 (targetArray) (INPUT)。
     */
    void calculateTargets(int planned[3], const int currentTarget[3], unsigned long now);

    /**
     * @brief 輸出速度因數給 PID 輸出使用 (停止模式)。
     */
    float getSpeedFactor() const { return speedFactor; }

    /**
     * @brief 輸出速度漸變 (Morphing) 的倍率。
     */
    float getMorphFactor(int i) const { return morphFactor[i]; }

    /**
     * @brief 輸出隨機模式的原始速度限制。
     */
    int getRandomSpeedLimit(int i) const { return randomSpeedLimit[i]; }

    /**
     * @brief 檢查馬達是否處於速度漸變狀態 (Morphing)。
     */
    bool isMorphing(int i) const { return speedMorph[i]; }

    // --- 輔助函式 (用於外部邏輯) ---
    void resetStopMode();

private:
    // --- 狀態變數 (所有隨機和停止模式的持久數據) ---

    // 隨機停止模式狀態
    StopModeState stopState = NORMAL;
    unsigned long stopTriggerTime = 0;
    float speedFactor = 1.0f; // 停止模式輸出的最終速度倍率
    unsigned long lastCheckTime = 0;
    unsigned long lastTriggerTime = 0;

    // 隨機目標生成狀態 (每顆馬達獨立)
    unsigned long nextUpdate[3] = {0, 0, 0};
    int randomCenter[3] = {8000, 8000, 8000};
    int randomSpeedLimit[3] = {150, 150, 150};
    float sinePhase[3] = {0.0, 1.0, 2.0};
    float sineSpeed[3] = {0.0004, 0.0003, 0.0005};

    // 速度漸變 (Morphing) 狀態
    bool speedMorph[3] = {false, false, false};
    bool morphUp[3] = {false, false, false};   // true=慢→快, false=快→慢
    float morphFactor[3] = {1.0f, 1.0f, 1.0f}; // 當前速度倍率 (僅用於 randomSpeedLimit 內部)
    unsigned long morphStartTime[3] = {0, 0, 0};
    unsigned long morphEndTime[3] = {0, 0, 0};
    float morphPhaseOffset[3] = {0.0, 0.0, 0.0};

    // --- 私有功能函式 ---
    void _updateRandomStopMode(unsigned long now);
    void _updateSpeedMorph(int i, unsigned long now);
};