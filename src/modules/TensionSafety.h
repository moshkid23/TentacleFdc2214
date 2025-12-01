// src/modules/TensionSafety.h
#pragma once
#include "../config/constants.h"

class TensionSafety
{
public:
    void begin();                                        // 初始化（可留空）
    void apply(int planned[3], const int currentPos[3]); // 主要功能

private:
    int findMasterMotor(const int pos[3]) const;
    int calculateSafeMin(int masterPos) const;

    int lastMaster = 0;                     // 用於遲滯
    static constexpr int HYSTERESIS = 1200; // 主軸切換遲滯，避免頻繁跳軸（可調 800~2000）
};