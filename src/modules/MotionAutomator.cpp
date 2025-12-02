// src/modules/MotionAutomator.cpp

#include "MotionAutomator.h"
#include <algorithm>    // std::max, std::min
#include <cmath>        // powf, sin, PI
#include <esp_system.h> // 需要包含此標頭檔來獲取 TRNG

// PI 在一些環境可能未定義，這裡手動定義，以確保 <cmath> 支援
#ifndef PI
#define PI 3.14159265358979323846f
#endif

void MotionAutomator::begin()
{
    // 【✅ 修正方案：使用 ESP32 內建的硬體真隨機數生成器 (TRNG)】
    // esp_random() 讀取 TRNG 輸出的 32-bit 真隨機數
    randomSeed(esp_random());
}

/**
 * @brief 重設停止模式 (用於 OSC/按鈕切換回常規模式時)
 */
void MotionAutomator::resetStopMode()
{
    stopState = NORMAL;
    speedFactor = 1.0f;
    stopTriggerTime = 0;
}

/**
 * @brief 更新隨機停止模式的狀態機
 */
void MotionAutomator::_updateRandomStopMode(unsigned long now)
{
    // === 1. 僅在 NORMAL 狀態且到達檢查時間才檢查 ===
    if (stopState == NORMAL && now - lastCheckTime >= CHECK_INTERVAL_MS)
    {
        lastCheckTime = now;
        // 檢查最小間隔 (避免頻繁觸發)
        if (now - lastTriggerTime >= MIN_STOP_INTERVAL_MS && random(100) < STOP_CHANCE)
        {
            stopState = SLOWING;
            stopTriggerTime = now;
            lastTriggerTime = now;
            // Serial.println("Random STOP triggered!");
        }
    }

    // === 2. 狀態機邏輯 ===
    switch (stopState)
    {
    case NORMAL:
        speedFactor = 1.0f;
        break;

    case SLOWING:
    {
        // t 從 0.0 線性增加到 1.0
        float t = (now - stopTriggerTime) / (float)SLOW_DOWN_MS;
        if (t >= 1.0f)
        {
            speedFactor = 0.0f;
            stopState = STOPPED;
            stopTriggerTime = now;
        }
        else
        {
            // 速度因數從 1.0 線性減到 0.0
            speedFactor = 1.0f - t;
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
        // t 從 0.0 線性增加到 1.0
        float t = (now - stopTriggerTime) / (float)SPEED_UP_MS;
        if (t >= 1.0f)
        {
            speedFactor = 1.0f;
            stopState = NORMAL;
        }
        else
        {
            // 速度因數從 0.0 線性增到 1.0
            speedFactor = t;
        }
    }
    break;
    }

    // 強制限制速度因數在 [0.0, 1.0] 之間
    speedFactor = std::max(0.0f, std::min(speedFactor, 1.0f));
}

/**
 * @brief 更新單顆馬達的速度漸變 (Morphing) 狀態
 */
void MotionAutomator::_updateSpeedMorph(int i, unsigned long now)
{
    // 如果不在 Morphing 狀態，則維持全速
    if (!speedMorph[i])
    {
        morphFactor[i] = 1.0f;
        return;
    }

    // 參數初始化：只在 Morphing 開始時執行一次
    static unsigned long morphDuration[3] = {0, 0, 0};
    if (morphEndTime[i] == 0)
    {
        morphDuration[i] = random(1500, 5000);        // 持續 1.5s~5s
        morphPhaseOffset[i] = random(0, 30) / 100.0f; // 隨機的起始相位
    }

    unsigned long elapsed = now - morphStartTime[i];
    float progress = (float)elapsed / (float)morphDuration[i];

    if (progress >= 1.0f)
    {
        progress = 1.0f;
        speedMorph[i] = false; // 🔚 自動結束 morph
        morphEndTime[i] = now;
        // 結束後固定在目標極值 (慢速或全速)
        morphFactor[i] = morphUp[i] ? 1.0f : 0.3f;
        return;
    }

    // 🎚️ 使用 sin 曲線讓變化更自然 (Ease-in / Ease-out)
    float phase = progress + morphPhaseOffset[i];
    phase = std::min(phase, 1.0f);        // 確保 phase 不超過 1.0
    float eased = sin(phase * PI / 2.0f); // 0 -> 1

    if (morphUp[i])
    {
        // 慢 → 快：從 0.3 緩升至 1.0
        morphFactor[i] = 0.3f + 0.7f * eased;
    }
    else
    {
        // 快 → 慢：從 1.0 緩降至 0.3
        morphFactor[i] = 1.0f - 0.7f * eased;
    }
    // 確保速度限制在合理範圍內
    morphFactor[i] = std::max(0.3f, std::min(morphFactor[i], 1.0f));
}

/**
 * @brief 檢查並更新所有狀態機和因數 (在 loop() 開頭呼叫)
 */
void MotionAutomator::updateState(bool isRandomMode, unsigned long now)
{
    // 如果不是隨機模式 (例如 OSC 模式)，強制關閉所有自動邏輯
    if (!isRandomMode)
    {
        // OSC 模式強制關閉停止效果
        resetStopMode();
        for (int i = 0; i < 3; ++i)
        {
            morphFactor[i] = 1.0f;
            speedMorph[i] = false;
        }
        return;
    }

    // 運行隨機停止狀態機
    _updateRandomStopMode(now);

    // 運行速度漸變狀態機 (Morphing)
    for (int i = 0; i < 3; ++i)
    {
        _updateSpeedMorph(i, now);
    }
}

/**
 * @brief 核心功能：計算隨機模式下的目標位置，並寫入 planned 陣列
 * @param planned[3]: 輸出計算後的目標位置。
 * @param currentTarget[3]: 輸入當前馬達的目標位置 (targetArray)，用於 LERP 起點。
 */
void MotionAutomator::calculateTargets(int planned[3], const int currentTarget[3], unsigned long now)
{
    for (int i = 0; i < 3; ++i)
    {
        // 1. 更新隨機參數 (中心、速度、Morphing觸發)
        if (now > nextUpdate[i])
        {
            // 中心分區：大範圍 60%、小範圍 40%
            int rangePick = random(100);
            if (rangePick < 60)
                randomCenter[i] = random(3000, 12000);
            else
                randomCenter[i] = random(3000, 4000);

            // 速度層級：中速60%、慢速30%、快速10%
            int speedPick = random(100);
            if (speedPick < 60)
                randomSpeedLimit[i] = random(30, 80);
            else if (speedPick < 90)
                randomSpeedLimit[i] = random(10, 30);
            else
                randomSpeedLimit[i] = random(80, 150);

            // 速度變形 (Morphing) 觸發
            int morphPick = random(100);
            if (morphPick < 5) // 5% 機率啟動加速
            {
                speedMorph[i] = true;
                morphUp[i] = true;
                morphStartTime[i] = now;
                morphEndTime[i] = 0;
            }
            else if (morphPick < 10) // 5% 機率啟動減速
            {
                speedMorph[i] = true;
                morphUp[i] = false;
                morphStartTime[i] = now;
                morphEndTime[i] = 0;
            }
            else
            {
                // 如果 Morphing 已經結束，則重置 morphFactor
                if (morphEndTime[i] != 0 && now > morphEndTime[i])
                {
                    speedMorph[i] = false;
                    morphFactor[i] = 1.0f;
                    morphEndTime[i] = 0;
                }
            }

            sineSpeed[i] = random(20, 60) / 100000.0f;
            sinePhase[i] = random(0, 628) / 100.0f; // 0.0 到 6.28
            nextUpdate[i] = now + random(RANDOM_UPDATE_MIN_MS, RANDOM_UPDATE_MAX_MS);
        }

        // 2. 應用呼吸波計算目標
        // 計算 sine 波的偏移量
        float wave = sin((now * sineSpeed[i]) + sinePhase[i]);
        int waveOffset = (int)(wave * RANDOM_WAVE_AMPLITUDE);
        // 加上中心點並限制在編碼器範圍內
        int target = constrain(randomCenter[i] + waveOffset, 0, MAX_ENCODER);

        // 3. LERP 平滑目標
        // 從當前目標 (currentTarget) 平滑過渡到新計算的目標 (target)
        planned[i] = currentTarget[i] + (int)((target - currentTarget[i]) * RANDOM_LERP_T);
        planned[i] = constrain(planned[i], 0, MAX_ENCODER);
    }
}