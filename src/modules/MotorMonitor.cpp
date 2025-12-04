// src/modules/MotorMonitor.cpp
#include "MotorMonitor.h"
#include <algorithm> // 包含 std::all_of 或 labs

// 初始化 portMUX_TYPE
portMUX_TYPE MotorMonitor::encoder_mux = portMUX_INITIALIZER_UNLOCKED;

MotorMonitor::MotorMonitor()
{
    // 構造函式不需要特別做什麼，因為成員變數已經初始化
}

MotionStatus MotorMonitor::getMotionStatus() const
{
    MotionStatus status;
    status.isAnyMotion = anyMotion;
    status.lastChangeTime = lastAnyMotionChangeTime;
    return status;
}

void MotorMonitor::update(EncoderStepCounter encoders[], unsigned long now)
{
    bool newAnyMotion = false;

    for (int i = 0; i < 3; ++i)
    {
        int pos = 0;

        // 1. 讀取編碼器 (鎖定中斷)
        portENTER_CRITICAL(&encoder_mux);
        pos = encoders[i].getPosition();
        encoders[i].reset();
        portEXIT_CRITICAL(&encoder_mux);

        // 2. 更新累積位置
        if (pos != 0)
        {
            posiArray[i] += pos;
        }

        // 3. 單軸 motion 判斷
        if (pos != 0)
        {
            // 有動 → 更新「上次有動的時間」
            lastMoveTime[i] = now;
            motion[i] = true;
        }
        else
        {
            // 沒動 → 若距離上次動過超過 40ms 才判斷為靜止
            if (now - lastMoveTime[i] > STATIONARY_THRESHOLD_MS)
            {
                motion[i] = false;
            }
            else
            {
                motion[i] = true; // 暫時保持動的狀態
            }
        }

        // 4. 判斷全軸狀態
        if (motion[i] == true)
        {
            newAnyMotion = true; // 只要有一個軸動，全軸就動
        }
    }

    // 5. 判斷 anyMotion 並檢測改變 (取代原 mainModules 底部邏輯)
    if (newAnyMotion != lastAnyMotion)
    {
        lastAnyMotionChangeTime = now;
        lastAnyMotion = newAnyMotion;
        // Serial.print("Motion changed -> ");
        // Serial.println(newAnyMotion ? "True" : "False");
    }
    anyMotion = newAnyMotion;
}