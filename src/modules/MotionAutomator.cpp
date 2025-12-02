// src/modules/MotionAutomator.cpp (實現停止模式邏輯)
#include "MotionAutomator.h"
#include <esp_system.h> // for esp_random()
#include <algorithm>    // for std::max, std::min (雖然這個邏輯用 if/else 也可以，但建議保留)
#include <cmath>

void MotionAutomator::begin()
{
    randomSeed(esp_random());
}
// stop mode
void MotionAutomator::resetStopMode()
{
    stopState = NORMAL;
    speedFactor = 1.0f;
    stopTriggerTime = 0;
}
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

// random mode
void MotionAutomator::calculateTargets(int planned[3], const int currentTarget[3], unsigned long now)
{
    for (int i = 0; i < 3; ++i)
    {
        // 1. 更新隨機參數 (randomCenter, speedLimit, sine, Morphing 觸發)
        if (now > nextUpdate[i])
        {
            // 中心位置分區
            int rangePick = random(100);
            if (rangePick > 60)
                randomCenter[i] = random(6000, 12000);
            else
                randomCenter[i] = random(3000, 6000);

            // 速度層級
            int speedPick = random(100);
            if (speedPick < 60) // 60% 中速
                randomSpeedLimit[i] = random(30, 80);
            else if (speedPick < 90) // 30% 慢速
                randomSpeedLimit[i] = random(10, 30);
            else // 10% 快速
                randomSpeedLimit[i] = random(80, 150);

            // Morphing 觸發邏輯
            int morphPick = random(100);
            if (morphPick < 5)
            {
                speedMorph[i] = true;
                morphUp[i] = true;
                morphFactor[i] = 0.3f; // 起始值
            }
            else if (morphPick < 10)
            {
                speedMorph[i] = true;
                morphUp[i] = false;
                morphFactor[i] = 1.0f; // 起始值
            }
            else
            {
                speedMorph[i] = false;
            }

            // 更新正弦波參數
            sineSpeed[i] = random(20, 60) / 100000.0f;
            sinePhase[i] = random(0, 628) / 100.0f;
            nextUpdate[i] = now + random(RANDOM_UPDATE_MIN_MS, RANDOM_UPDATE_MAX_MS);
        }

        // 2. 呼吸波計算目標
        float wave = sin((now * sineSpeed[i]) + sinePhase[i]);
        int waveOffset = (int)(wave * RANDOM_WAVE_AMPLITUDE);
        int target = constrain(randomCenter[i] + waveOffset, 0, MAX_ENCODER);

        // 3. LERP 平滑目標
        float t = RANDOM_LERP_T / 100.0f;
        planned[i] = currentTarget[i] + (int)((target - currentTarget[i]) * t);

        // 最終限制
        planned[i] = constrain(planned[i], 0, MAX_ENCODER);
    }
}

// speed morph
void MotionAutomator::updateMorphFactor(unsigned long now)
{
    for (int i = 0; i < 3; ++i)
    {
        if (speedMorph[i])
        {
            // 檢查是否是 Morphing 剛觸發（在 calculateTargets 之後）
            if (morphStartTime[i] == 0)
            {
                // 由於 Morphing 狀態已在 calculateTargets 中設置為 true，現在開始計時
                morphStartTime[i] = now;
                morphDuration[i] = random(1500, 5000);        // 1.5s~5s
                morphPhaseOffset[i] = random(0, 30) / 100.0f; // 0.0~0.3
            }

            unsigned long elapsed = now - morphStartTime[i];
            float progress = (float)elapsed / (float)morphDuration[i];

            if (progress >= 1.0f)
            {
                progress = 1.0f;
                speedMorph[i] = false; // 🔚 自動結束 morph
                morphStartTime[i] = 0; // 重置計時器，等待下次觸發
                // Serial.printf("✅ M%d morph 結束\n", i);
            }

            // 使用 sin 曲線讓變化更自然（ease-in / ease-out）
            float phase = progress + morphPhaseOffset[i];
            if (phase > 1.0f)
                phase = 1.0f;                       // 避免超出
            float eased = sin(phase * M_PI / 2.0f); // M_PI = π (0~1 之間的 sin 曲線)
                                                    // 0 ~ π/2 = 0 ~ 1
                                                    // π/2 ~ π = 1 ~ 0
                                                    // π ~ 3π/2 = 0 ~ -1
                                                    // 3π/2 ~ 2π = -1 ~ 0

            if (morphUp[i])
            {
                // 慢 → 快：從 0.3 緩升至 1.0
                morphFactor[i] = 0.3f + 0.7f * eased;
            }
            else
            {
                // 快 → 慢：從 0.7 緩降至 0.2
                morphFactor[i] = 0.7f - 0.6f * eased;
            }
        }
        else
        {
            // 確保 Morphing 結束時，Factor 恢復正常值 1.0
            morphFactor[i] = 1.0f;
            morphStartTime[i] = 0; // 確保非 Morphing 狀態下計時器為 0
        }
    }
}