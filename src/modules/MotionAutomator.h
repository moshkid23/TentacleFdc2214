// src/modules/MotionAutomator.h
#pragma once
#include <Arduino.h>
#include "../config/constants.h" // 確保引入 constants.h

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
    // stop mode
    void updateState(bool isRandomMode, unsigned long now);
    void resetStopMode();
    float getSpeedFactor() const { return speedFactor; }

    // random mode
    void calculateTargets(int planned[3], const int currentTarget[3], unsigned long now);
    int getRandomSpeedLimit(int i) const { return randomSpeedLimit[i]; }

    // speed morph
    bool isMorphing(int i) const { return speedMorph[i]; }
    float getMorphFactor(int i) const { return morphFactor[i]; }
    void updateMorphFactor(unsigned long now);

private:
    // stop mode
    StopModeState stopState = NORMAL;
    unsigned long stopTriggerTime = 0;
    float speedFactor = 1.0f; // 停止模式輸出的最終速度倍率
    unsigned long lastCheckTime = 0;
    unsigned long lastTriggerTime = 0;
    void _updateRandomStopMode(unsigned long now);

    // random mode
    unsigned long nextUpdate[3] = {0, 0, 0};
    int randomCenter[3] = {8000, 8000, 8000};
    int randomSpeedLimit[3] = {150, 150, 150};
    float sinePhase[3] = {0.0, 1.0, 2.0};
    float sineSpeed[3] = {0.0004, 0.0003, 0.0005};

    // speed morph
    bool speedMorph[3] = {false, false, false}; // 確保宣告為成員變數
    bool morphUp[3] = {false, false, false};
    float morphFactor[3] = {1.0, 1.0, 1.0}; // 初始值必須在此宣告
    unsigned long morphStartTime[3] = {0, 0, 0};
    unsigned long morphDuration[3] = {2000, 2000, 2000}; // 預設值
    float morphPhaseOffset[3] = {0.0, 0.0, 0.0};
};