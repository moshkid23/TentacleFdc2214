// 「pos-anchor 安全張力＋隨機停止 + 模組化」
// src/mainModules.cpp
#include <Arduino.h>
static char logBuffer[100] = {0}; // 用在 handleTouchDetection 函數中

// ledPWM 設置
const int ledPin = 12;       // LED 連接到 GPIO 12
const int ledPwmChannel = 3; // PWM 通道 0

// 按鈕設定
#include <ezButton.h>
ezButton button(13);
bool randomMode = false; // false = OSC 模式, true = 隨機模式

// 馬達管腳配置
const int motorPins[3][2] = {
    {19, 18}, // AIN1, AIN2 [Motor 1]
    {17, 16}, // BIN1, BIN2 [Motor 2]
    {2, 15}   // CIN1, CIN2 [Motor 3]
};
const int PWM_PINS[3] = {23, 5, 4};
const int PWM_CHANNELS[3] = {0, 1, 2};
// 馬達控制函數
void setMotor(int motor, int speed, bool forward)
{
  int in1 = motorPins[motor][0];
  int in2 = motorPins[motor][1];
  int pwmChannel = PWM_CHANNELS[motor];

  // 設定方向
  if (forward)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  // 設定速度
  ledcWrite(pwmChannel, speed);
}
// 編碼器設定
#include "EncoderStepCounter.h"
const int ENCODER_PINS[3][2] = {
    {32, 33}, // Motor 1
    {25, 26}, // Motor 2
    {27, 14}  // Motor 3
};
EncoderStepCounter encoders[3] = {
    {ENCODER_PINS[0][0], ENCODER_PINS[0][1], HALF_STEP},
    {ENCODER_PINS[1][0], ENCODER_PINS[1][1], HALF_STEP},
    {ENCODER_PINS[2][0], ENCODER_PINS[2][1], HALF_STEP}};
void IRAM_ATTR interrupt1() { encoders[0].tick(); }
void IRAM_ATTR interrupt2() { encoders[1].tick(); }
void IRAM_ATTR interrupt3() { encoders[2].tick(); }
// PID 參數
#include "modules/Pid.h"
Pid motors[3];
float kp = 1.0, ki = 0.0, kd = 0.0;
// 馬達狀態變數
int posiArray[3] = {0, 0, 0};   // 馬達當前位置
int targetArray[3] = {0, 0, 0}; // 目標位置
bool motion[3] = {0, 0, 0};     // 是否在運動
bool anyMotion = false;

#include "modules/OSCManager.h"
OSCManager osc;
#include "modules/TensionSafety.h"
TensionSafety tension;
#include "modules/MotionAutomator.h"
MotionAutomator motionAuto;

// 電容感測器設定
#include <Wire.h>
#include "FDC2214.h"
FDC2214 capsense(FDC2214_I2C_ADDR_0); // FDC2214_I2C_ADDR_0 對應 0x2A
unsigned long capa;
unsigned long previousCapaClose = 0; // 上次讀取的 capa 值
unsigned long proxThresh;

// 全域變數（放在 loop 外面）
static unsigned long lastPrintValue = 0; // 上次印出的 capa 值
static const int PRINT_THRESHOLD = 5;    // 變化 > 5 才印（可調）
static const char *modeStr = "";

// 觸摸檢測 + 智慧列印
static bool lastAnyMotion = false;
static unsigned long lastAnyMotionChangeTime = 0;
static unsigned long lastReadTimeClose = 0;
static unsigned long lastReadTimeAway = 0;

// ==== 觸摸中位數 Median(5) ====
const int MED_WIN = 5;
unsigned long medBuf[MED_WIN];
int medIdx = 0;
bool medFilled = false;
unsigned long median5(unsigned long x)
{
  medBuf[medIdx++] = x;
  if (medIdx >= MED_WIN)
  {
    medIdx = 0;
    medFilled = true;
  }
  int n = medFilled ? MED_WIN : medIdx;
  // 複製 + 插入排序
  unsigned long tmp[MED_WIN];
  for (int i = 0; i < n; i++)
    tmp[i] = medBuf[i];
  for (int i = 1; i < n; i++)
  {
    unsigned long k = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > k)
    {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = k;
  }
  return tmp[n / 2];
}
// ==== Exponential Moving Average (EMA) ====
// y = (1-α)*y + α*x
float alpha = 0.30f; // 反應速度：0.2 快、0.1 慢，想再快一點：0.3~0.4。
double ema_y = 0;    // 用 double/float 都可；也可用整數累積
unsigned long emaFilter(unsigned long x)
{
  if (ema_y == 0)
    ema_y = (double)x; // 初始化
  ema_y = (1.0 - alpha) * ema_y + alpha * (double)x;
  return (unsigned long)(ema_y + 0.5);
}
// ==== 可選：毛刺夾限（防連續怪值） ====
long lastRaw = 0;
unsigned long clampSpike(unsigned long x)
{
  if (lastRaw != 0 && labs((long)x - lastRaw) > 50000)
  {
    // 突波夾限：直接回上一筆或夾到門檻附近（視需求）
    return (unsigned long)lastRaw;
  }
  lastRaw = (long)x;
  return x;
}
// ==== 讀值：先毛刺夾限 -> median(5) -> EMA ====
unsigned long readFiltered()
{
  unsigned long raw = capsense.getReading28(2);
  // unsigned long clamped = clampSpike(raw);   // 可關掉，若不需要
  // unsigned long m = median5(clamped);
  unsigned long m = median5(raw);
  unsigned long y = emaFilter(m);
  // return y;
  return raw;
}
// === 觸摸智慧列印函式snprintf ===
void handleTouchDetection(unsigned long currentMillis, int approachThresh, int leaveThresh)
{
  // if (currentMillis - lastReadTimeClose < 40)
  if (currentMillis - lastReadTimeClose < 17)

    return;

  long diff = (long)capa - (long)previousCapaClose;
  if (abs(diff) < 10)
  {
    previousCapaClose = capa;
    lastReadTimeClose = currentMillis;
    return;
  }

  const char *event = "";
  if (diff <= -approachThresh)
  {
    proxThresh = capa;
    ledcWrite(ledPwmChannel, 255);
    event = "接近";
  }
  else if (capa - proxThresh >= leaveThresh)
  {
    ledcWrite(ledPwmChannel, 0);
    event = "離開";
  }

  // 智慧列印
  //  if (event[0] || abs((long)capa - (long)lastPrintValue) > PRINT_THRESHOLD)
  //  {
  //    snprintf(logBuffer, sizeof(logBuffer), "[%s] capa: %5lu | diff: %+5ld | %s → LED %s\n",
  //             modeStr, capa, diff,
  //             event[0] ? event : "    ",
  //             ledcRead(ledPwmChannel) ? "ON" : "OFF");
  //    Serial.println(logBuffer);
  //    lastPrintValue = capa;
  //  }

  previousCapaClose = capa;
  lastReadTimeClose = currentMillis;
}

void setup()
{
  Serial.begin(115200);
  osc.begin(); // WiFi + OSC 初始化
  motionAuto.begin();
  tension.begin();
  // 初始化電容感測器
  Wire.begin(21, 22); /* SDA=21, SCL=22 */
  // Wire.setClock(400000L);
  Wire.setClock(100000L);                            // Set I2C clock to 100kHz (slower for stability)
  bool capOk = capsense.begin(0x4, 0x6, 0x5, false); // 差分量測
  // bool capOk = capsense.begin(0x4, 0x6, 0x5, true); //單端量測
  if (capOk)
  {
    delay(50); // Wait for sensor stabilization
    // proxThresh = capsense.getReading28(2);
    proxThresh = readFiltered();
    Serial.print("Initial proxThresh: ");
    Serial.println(proxThresh);
  }
  else
    Serial.println("FDC2214 Sensor Fail");

  // 初始化馬達腳位+編碼器+中斷
  for (int i = 0; i < 3; ++i)
  {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    ledcSetup(PWM_CHANNELS[i], 16000, 8);
    ledcAttachPin(PWM_PINS[i], PWM_CHANNELS[i]); // 將 PWM 腳位與通道綁定
    encoders[i].begin();
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][0]), i == 0 ? interrupt1 : (i == 1 ? interrupt2 : interrupt3), CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][1]), i == 0 ? interrupt1 : (i == 1 ? interrupt2 : interrupt3), CHANGE);
  }

  // 初始化 LED PWM
  ledcSetup(ledPwmChannel, 5000, 8);    // channel, frequency (5kHz), resolution (8-bit)
  ledcAttachPin(ledPin, ledPwmChannel); // Attach pin to LEDC channel
  // 按鈕設定
  button.setDebounceTime(50);
}

void loop()
{
  osc.update(); // 接收 OSC
  // 按鈕更新
  button.loop();
  if (button.isPressed())
  {
    randomMode = true; //
    Serial.println("🎲 進入隨機模式");
  }
  else if (button.isReleased())
  {
    randomMode = false;
    Serial.printf("🌐 回到 OSC 模式，恢復目標 A=%d, B=%d, C=%d\n",
                  osc.getTarget(0), osc.getTarget(1), osc.getTarget(2));

    // 新增：重置停止模式（立即恢復正常速度）
    motionAuto.resetStopMode();
  }

  static unsigned long previousMillis = 0; // 上次讀取的時間
  unsigned long currentMillis = millis();  // 當前時間
  // 讀取電容值
  if (currentMillis - previousMillis >= 40)
  {
    // capaRaw = capsense.getReading28(2); // Read CH2
    capa = readFiltered();
    // Serial.println(capaRaw);        // Output single value
    previousMillis = currentMillis; // 更新上次讀取時間
  }

  // === 觸摸判斷主邏輯 ===
  if (anyMotion == false && (currentMillis - lastAnyMotionChangeTime) >= 50)
  {
    modeStr = "STATIC";
    handleTouchDetection(currentMillis, 200, 200); // 靜止：接近 -200，離開 +200
  }
  else if (anyMotion == true && (currentMillis - lastAnyMotionChangeTime) >= 50)
  {
    modeStr = "MOVING";
    handleTouchDetection(currentMillis, 300, 300); // 移動：接近 -300，離開 +300
  }

  // ==== ✨ 階段1：先計算三顆的「下一步候選值 planned[]」(不直接動 targetArray) ====
  int planned[3];

  unsigned long now = millis();
  motionAuto.updateState(randomMode, now);
  // 產生基準目標（random 或 OSC）
  if (randomMode)
  {
    // 【✅ 修正：只呼叫一次目標計算】
    motionAuto.calculateTargets(planned, targetArray, now);

    // ❌ 刪除：整個 for (int i = 0; i < 3; ++i) { ... } 區塊
    // 該區塊的邏輯 (updateMorphFactor, rawPower 調整) 應屬於 PID 輸出階段。
  }
  else // OSC 模式
  {
    for (int i = 0; i < 3; ++i)
    {
      planned[i] = osc.getTarget(i);
      planned[i] = constrain(planned[i], 0, 17000);
    }
  }

  // ⚠️ 注意：如果所有 Constrain 都已在 MotionAutomator 內或 OSC 迴圈內完成，這裡可以省略。
  // 由於 OSC 模式下的 Constrain 應該在 else 內部完成，這裡不再需要對所有馬達 Constrain。
  // 為了安全，我們保留 OSC 模式下的 Constrain 如下：
  // for (int i = 0; i < 3; ++i)
  //   planned[i] = constrain(planned[i], 0, 17000);
  tension.apply(planned, posiArray); // 張力安全調整
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
  for (int j = 0; j < 3; ++j)
    targetArray[j] = planned[j];

  // // （可選）摘要監控
  // static unsigned long lastDbg = 0;
  // if (millis() - lastDbg > 200)
  // {
  //   Serial.printf("[ten] master=%d anchor=%d safeMin=%d | T=(%d,%d,%d) | pos=(%d,%d,%d)\n",
  //                 master, anchor, safeMin, targetArray[0], targetArray[1], targetArray[2],
  //                 posNow[0], posNow[1], posNow[2]);
  //   lastDbg = millis();
  // }

  static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
  unsigned long loopNow = millis();
  // 【✅ 呼叫更新 Morphing 狀態和因子】
  motionAuto.updateMorphFactor(loopNow);

  for (int i = 0; i < 3; ++i)
  {
    int pos = 0;
    // 🏆 最佳且高效的方案：暫時鎖住中斷
    portENTER_CRITICAL(&encoder_mux);
    pos = encoders[i].getPosition();
    encoders[i].reset();
    portEXIT_CRITICAL(&encoder_mux);
    // 更新累積位置 (posiArray[i])
    if (pos != 0)
    {
      posiArray[i] += pos;
    }

    // // 更新編碼器位置舊
    // int pos = encoders[i].getPosition();
    // if (pos != 0)
    // {
    //   posiArray[i] += pos;
    //   encoders[i].reset();
    // }

    // ========= motion判斷 =========
    static unsigned long lastMoveTime[3] = {0, 0, 0};
    if (pos != 0)
    {
      // 有動 → 更新「上次有動的時間」
      lastMoveTime[i] = loopNow;
      motion[i] = true;
    }
    else
    {
      // 沒動 → 若距離上次動過超過 40ms 才判斷為靜止
      if (loopNow - lastMoveTime[i] > 40)
      {
        motion[i] = false;
      }
      else
      {
        motion[i] = true; // 暫時保持動的狀態
      }
    }

    // 更新PID馬達輸出
    Pid::DiPo statuses = motors[i].DirAndPwr(targetArray[i], posiArray[i], kp, ki, kd);
    int direction = statuses.D;
    int rawPower = statuses.P;

    if (randomMode)
    {
      // ⚠️ 這裡必須使用 MotionAutomator 提供的 speedMorph 狀態和 morphFactor
      if (motionAuto.isMorphing(i))
      {
        // ✨ 當使用加速度變化時，不受 randomSpeedLimit 限制
        // 【✅ 修改：使用 MotionAutomator 輸出的 morphFactor】
        rawPower = constrain(rawPower * motionAuto.getMorphFactor(i), 0, 200);
      }
      else
      {
        // 🧭 正常分層狀態：受限於 randomSpeedLimit
        // 【✅ 修改：使用 MotionAutomator 輸出的 randomSpeedLimit】
        rawPower = constrain(rawPower, 0, motionAuto.getRandomSpeedLimit(i));
      }
    }

    // === 停止模式：乘 speedFactor（自動用當下功率作為起/終點）===
    int finalPower = (int)(rawPower * motionAuto.getSpeedFactor());
    finalPower = constrain(finalPower, 0, 255);

    // 移動馬達
    setMotor(i, finalPower, direction > 0);
  }

  // 判斷 anyMotion 並檢測改變
  bool newAnyMotion = !(motion[0] == false && motion[1] == false && motion[2] == false);
  if (newAnyMotion != lastAnyMotion)
  {
    lastAnyMotionChangeTime = currentMillis;
    lastAnyMotion = newAnyMotion;
    // Serial.print("Motion changed -> ");
    // Serial.println(newAnyMotion ? "True" : "False");
  }
  anyMotion = newAnyMotion;
}