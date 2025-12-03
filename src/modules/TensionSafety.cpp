#include "TensionSafety.h"
#include <algorithm> // std::max, std::min
#include <cmath>     // powf
// #include <Arduino.h> // 如果需要 Serial.printf 或其他 Arduino 函式，請取消註釋

void TensionSafety::begin()
{
    // 目前無需初始化
}

/**
 * @brief 找出當前位置最高的馬達索引 (潛在的主軸)
 * @param pos 當前三顆馬達的位置陣列 (posiArray)
 * @return 位置最高的馬達索引 (0, 1, or 2)
 */
int TensionSafety::findMasterMotor(const int pos[3]) const
{
    int master = 0;
    int maxPos = pos[0];
    for (int i = 1; i < 3; ++i)
    {
        if (pos[i] > maxPos)
        {
            maxPos = pos[i];
            master = i;
        }
    }
    return master;
}

/**
 * @brief 根據主軸位置計算安全下限 (Safe Minimum)
 * @param masterPos 主軸的當前或錨定位置 (anchor)
 * @return 根據曲線計算出的安全下限
 */
int TensionSafety::calculateSafeMin(int masterPos) const
{
    if (masterPos < 100)
        return 0;

    // 將位置正規化到 [0.0, 1.0]
    float ratio = static_cast<float>(masterPos) / MAX_ENCODER;
    ratio = std::max(0.0f, std::min(ratio, 1.0f));

    // 計算主曲線 (y = ratio^K * LIMIT_HIGH)
    float y = powf(ratio, TENSION_CURVE_K) * LIMIT_HIGH;

    // 計算並套用修正項，使曲線通過 (0.5 * MAX_ENCODER, LIMIT_MID)
    float correction = LIMIT_MID - (powf(0.5f, TENSION_CURVE_K) * LIMIT_HIGH);
    y += correction * (1.0f - ratio);

    return std::max(static_cast<int>(y), 0);
}

/**
 * @brief 應用張力約束：找出主軸，計算安全下限，並套用至其他兩軸
 * @param planned[3] 經 OSC/Random 計算出的目標位置 (會被修改)
 * @param currentPos[3] 馬達的實際當前位置 (posiArray)
 */
void TensionSafety::apply(int planned[3], const int currentPos[3])
{
    // 1. 找出潛在的主軸 (Candidate Master)，即實際位置最高者
    int cand = findMasterMotor(currentPos);

    // 2. 應用主軸切換遲滯：
    // 只有當潛在主軸比現任主軸高出 HYSTERESIS 幅度時才換
    int master = lastMaster;
    if (cand != lastMaster && currentPos[cand] > currentPos[lastMaster] + HYSTERESIS)
    {
        master = cand;
        // 可選：Serial.printf("🧭 master pos-switch %d→%d\n", lastMaster, master);
    }
    // 更新現任主軸
    lastMaster = master;

    // 3. 計算錨定位置 (Anchor)：主軸的目標或實際位置，取兩者中較高的，避免主軸目標太低
    int anchor = std::max(planned[master], currentPos[master]);

    // 4. 計算安全下限
    int safeMin = calculateSafeMin(anchor);

    // 5. 套用安全下限：對非主軸的目標位置應用 safeMin
    for (int i = 0; i < 3; ++i)
    {
        // 僅對非主軸 (i != master) 的目標位置 (planned[i]) 進行限制
        if (i != master && planned[i] < safeMin)
        {
            planned[i] = safeMin;
        }
    }
}

// // ===== 全軸下跌限速（主軸稍嚴，其他也限一下）=====
// const int DROP_MASTER = 600; // 每輪主軸最多下降量
// const int DROP_OTHER = 500;  // 其他軸每輪最多下降量
// for (int j = 0; j < 3; ++j)
// {
//   int prevT = targetArray[j];
//   int dj = planned[j] - prevT;
//   int cap = (j == master) ? DROP_MASTER : DROP_OTHER;
//   if (dj < -cap)
//     planned[j] = prevT - cap;
// }
// ===== 最後寫回 targetArray =====