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
int targetArray[3] = {0, 0, 0}; // 目標位置

#include "modules/OSCManager.h"
OSCManager osc;
#include "modules/TensionSafety.h"
TensionSafety tension;
#include "modules/MotionAutomator.h"
MotionAutomator motionAuto;
#include "modules/MotorMonitor.h"
MotorMonitor motorMonitor;

// 電容感測器設定
#include <Wire.h>
#include "FDC2214.h"
FDC2214 capsense(FDC2214_I2C_ADDR_0); // FDC2214_I2C_ADDR_0 對應 0x2A
unsigned long capa;
unsigned long previousCapaClose = 0; // 上次讀取的 capa 值
unsigned long proxThresh;

// ==== 連續觸摸判定追蹤變數 ====
static int consecutiveCount = 0;
static unsigned long comparisonBaseCapa = 0; // 首次觸發時的基值
static bool baseCapaSet = false;             // 是否已設定基值
const int REQUIRED_CONSECUTIVE = 5;          // 所需的連續次數

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
  // return m;
  return y;
  //  return raw;
}

// === 觸摸智慧列印函式snprintf ===
void handleTouchDetection(unsigned long currentMillis, int approachThresh, int leaveThresh)
{
  // if (currentMillis - lastReadTimeClose < 17)
  if (currentMillis - lastReadTimeClose < 40)
    return;

  capa = capsense.getReading28(2); // Read CH2
  // 如果需要濾波，請改用: capa = readFiltered();

  long diff = (long)capa - (long)previousCapaClose;
  // 雜訊過濾
  if (abs(diff) < 10)
  {
    previousCapaClose = capa;
    lastReadTimeClose = currentMillis;
    return;
  }

  // 判斷是否「接近」的條件
  if ((long)capa - (long)comparisonBaseCapa <= -approachThresh)
  {
    // A. 第一次滿足條件：設定比較基值，並開始計數
    if (!baseCapaSet)
    {
      comparisonBaseCapa = capa; // ✨ 修正點 1：記錄當前低點作為連續判定的基準
      baseCapaSet = true;
      consecutiveCount = 1;
    }
    // B. 非第一次滿足條件：增加計數
    else
    {
      consecutiveCount++;
    }

    // C. 檢查是否達到連續次數 (假設 REQUIRED_CONSECUTIVE = 5)
    if (consecutiveCount >= REQUIRED_CONSECUTIVE)
    {
      // *** 判斷為「接近」的動作 ***
      proxThresh = capa; // 新的 proxThresh (用於「離開」判定)
      ledcWrite(ledPwmChannel, 255);
      const char *event = "接近";
      // 觸發智慧列印
      snprintf(logBuffer, sizeof(logBuffer), "[%s] capa: %5lu | diff: %+5ld | %s → LED %s (連續 %d 次)\n",
               modeStr, capa, (long)capa - (long)comparisonBaseCapa,
               event,
               ledcRead(ledPwmChannel) ? "ON" : "OFF",
               consecutiveCount);
      Serial.println(logBuffer);
      lastPrintValue = capa;

      // 重設判定狀態
      consecutiveCount = 0;
      baseCapaSet = false;
    }
  }
  // 3. 不滿足「接近」條件或「離開」
  else
  {
    // 判斷為「離開」的動作
    if (capa - proxThresh >= leaveThresh)
    {
      comparisonBaseCapa = capa; // 在這裡設定/更新新的基值
      ledcWrite(ledPwmChannel, 0);
      const char *event = "離開";
      // 觸發智慧列印
      snprintf(logBuffer, sizeof(logBuffer), "[%s] capa: %5lu | diff: %+5ld | %s → LED %s\n",
               modeStr, capa, (long)capa - (long)proxThresh,
               event,
               ledcRead(ledPwmChannel) ? "ON" : "OFF");
      Serial.println(logBuffer);
      lastPrintValue = capa;
    }

    // 歸零連續計數並重設基值旗標 (不滿足接近條件時)
    consecutiveCount = 0;
    baseCapaSet = false;
  }

  // --- 連續判定邏輯結束 ---

  // 更新 tracking 變數 (保持在函式尾部)
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
    // proxThresh = readFiltered();
    // Serial.print("Initial proxThresh: ");
    // Serial.println(proxThresh);

    comparisonBaseCapa = readFiltered();
    Serial.print("Initial comparisonBaseCapa: ");
    Serial.println(comparisonBaseCapa);
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
  osc.update();  // 接收 OSC
  button.loop(); // 按鈕更新
  if (button.isPressed())
  {
    randomMode = true; //
    Serial.println("🎲 進入隨機模式");
  }
  else if (button.isReleased())
  {
    randomMode = false;
    motionAuto.resetAllRandomEffects();
    Serial.printf("🌐 回到 OSC 模式，恢復目標 A=%d, B=%d, C=%d\n",
                  osc.getTarget(0), osc.getTarget(1), osc.getTarget(2));
  }

  static unsigned long previousMillis = 0; // 上次讀取的時間
  unsigned long currentMillis = millis();  // 當前時間
  // // 讀取電容值
  // if (currentMillis - previousMillis >= 40)
  // {
  //   capa = capsense.getReading28(2); // Read CH2
  //   // capa = readFiltered();
  //   Serial.println(capa);        // Output single value
  //   previousMillis = currentMillis; // 更新上次讀取時間
  // }

  // === 觸摸判斷主邏輯 ===
  MotionStatus status = motorMonitor.getMotionStatus();
  if (status.isAnyMotion == false && (currentMillis - status.lastChangeTime) >= 50)
  {
    modeStr = "STATIC";
    handleTouchDetection(currentMillis, 200, 200); // 靜止：接近 -200，離開 +200
  }
  else if (status.isAnyMotion == true && (currentMillis - status.lastChangeTime) >= 50)
  {
    modeStr = "MOVING";
    handleTouchDetection(currentMillis, 200, 200); // 移動：接近 -300，離開 +300
  }

  // ==== ✨ 階段1：先計算三顆的「下一步候選值 planned[]」(不直接動 targetArray) ====
  int planned[3];
  unsigned long now = millis();
  if (randomMode) // 自動模式
  {
    motionAuto.calculateTargets(planned, targetArray, now); // 計算下一步候選值
    motionAuto.updateMorphFactor(now);                      // 加速度變化更新
    motionAuto.updateRandomStopMode(now);                   // 隨機停止模式更新
  }
  else // OSC 模式
  {
    motionAuto.resetAllRandomEffects();
    for (int i = 0; i < 3; ++i)
    {
      planned[i] = osc.getTarget(i);
      planned[i] = constrain(planned[i], 0, 17000);
    }
  }

  motorMonitor.update(encoders, now); // 讀取 Encoder 和判斷 Motion

  // 獲取 Monitor 內最新的累積位置
  int currentPosiArray[3];
  for (int i = 0; i < 3; ++i)
  {
    currentPosiArray[i] = motorMonitor.getPosi(i);
  }

  tension.apply(planned, currentPosiArray); // 張力安全調整

  for (int j = 0; j < 3; ++j) // 計算後更新目標位置
    targetArray[j] = planned[j];

  // ==== ✨ 階段2：根據 targetArray 更新馬達輸出 ====
  for (int i = 0; i < 3; ++i)
  {
    Pid::DiPo statuses = motors[i].DirAndPwr(targetArray[i], currentPosiArray[i], kp, ki, kd);
    int direction = statuses.D;
    int rawPower = statuses.P;

    if (randomMode)
    {
      if (motionAuto.isMorphing(i))
      {
        // 加速度變化狀態
        rawPower = constrain(rawPower * motionAuto.getMorphFactor(i), 0, 200);
      }
      else
      {
        // 正常分層狀態
        rawPower = constrain(rawPower, 0, motionAuto.getRandomSpeedLimit(i));
      }
    }

    // === 停止模式：乘 speedFactor（自動用當下功率作為起/終點）===
    int finalPower = (int)(rawPower * motionAuto.getSpeedFactor());
    finalPower = constrain(finalPower, 0, 255);

    // 移動馬達
    setMotor(i, finalPower, direction > 0);
  }
}