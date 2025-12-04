// src/modules/MotorMonitor.h
#pragma once
#include <Arduino.h>
#include <EncoderStepCounter.h> // 必須包含 Encoder 庫的頭文件

struct MotionStatus
{
    bool isAnyMotion;
    unsigned long lastChangeTime;
};

class MotorMonitor
{
public:
    // 構造函式：初始化 Monitor
    MotorMonitor();

    /**
     * @brief 讀取編碼器、更新累積位置和運動狀態。
     * * @param encoders 外部傳入的三個 Encoder 物件陣列。
     * @param now 當前的 millis() 時間。
     */
    void update(EncoderStepCounter encoders[], unsigned long now);

    // Getters
    MotionStatus getMotionStatus() const;

    int getPosi(int i) const { return posiArray[i]; }

private:
    // 馬達狀態變數
    int posiArray[3] = {0, 0, 0};           // 馬達當前累積位置 (取代原 mainModules 的 posiArray)
    bool motion[3] = {false, false, false}; // 單軸是否在運動
    bool anyMotion = false;                 // 全軸是否在運動

    // 運動檢測相關變數
    static portMUX_TYPE encoder_mux; // 用於鎖定中斷
    unsigned long lastMoveTime[3] = {0, 0, 0};
    bool lastAnyMotion = false;
    unsigned long lastAnyMotionChangeTime = 0; // 上次 anyMotion 狀態改變的時間

    // 運動檢測常數
    static const int STATIONARY_THRESHOLD_MS = 40; // 超過此時間判斷為靜止
};