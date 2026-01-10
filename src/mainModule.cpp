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
// unsigned long capa;
// unsigned long previousCapaClose = 0; // 上次讀取的 capa 值
// unsigned long proxThresh;

void setFdcIDrive(uint8_t channel, uint16_t driveLevel)
{
  uint8_t regAddr = 0x1E + channel; // CH2 是 0x20

  // driveLevel 範圍是 0 ~ 31
  // 我們要寫入暫存器的 Bit 15-11
  uint16_t value = (driveLevel << 11);

  Wire.beginTransmission(0x2A);
  Wire.write(regAddr);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

// ==========================================
// 變數宣告 (請放在 mainModules.cpp 最上方)
// ==========================================

// --- 感測器數值暫存 ---
static unsigned long capa = 0; // 當前 FDC2214 讀值
static long lastCapa = 0;      // 上一次的讀值，用來算 Trend

// --- 能量判定核心變數 ---
static long energy = 0;                     // 累積的靠近能量
static int activeCount = 0;                 // 連續偵測到靠近的次數
static bool handDetected = false;           // 目前是否判定為觸發狀態
static unsigned long lastReadTimeClose = 0; // 時間戳記
static const char *modeStr = "INIT";        // <--- 確保這行在這裡

// --- 調整參數 (可依實測手感微調) ---
const int SAMPLE_INTERVAL = 40;  // 取樣間隔 40ms
const int APPROACH_THRESH = 200; // 啟動門檻：負趨勢要超過此值才開始累加
// const int APPROACH_THRESH = 100; // 啟動門檻：負趨勢要超過此值才開始累加
const long ENERGY_GOAL = 1000; // 能量總量門檻：超過此值才亮燈
// const long ENERGY_GOAL = 500; // 能量總量門檻：超過此值才亮燈
const int LEAVE_THRESH = 200; // 離開門檻：正向趨勢(離開)超過此值則熄滅
// const int LEAVE_THRESH = 500;  // 離開門檻：正向趨勢(離開)超過此值則熄滅

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
float alpha = 0.40f; // 反應速度：0.2 快、0.1 慢，想再快一點：0.3~0.4。
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
}

// ==========================================
// 核心函式：handleEnergyDetection
// ==========================================

void handleEnergyDetection(unsigned long currentMillis)
{
  // 1. 定時觸發檢查 (確保每 40ms 執行一次)
  if (currentMillis - lastReadTimeClose < SAMPLE_INTERVAL)
    return;

  // 2. 讀取感測器
  // capa = capsense.getReading28(2);
  capa = capsense.getReading28(3);

  // 3. 初始運行處理：如果是開機第一筆，先記錄數值就離開
  if (lastCapa == 0)
  {
    lastCapa = (long)capa;
    lastReadTimeClose = currentMillis;
    return;
  }

  // 4. 計算趨勢 (Trend)
  // 趨勢 = 現在 - 過去。手靠近時，電容下降，Trend 會是負數。
  long pureTrend = (long)capa - lastCapa;

  // 更新 lastCapa 供下次使用
  lastCapa = (long)capa;

  // 5. 能量積分邏輯 (區別突波與快手的關鍵)
  if (pureTrend <= -APPROACH_THRESH)
  {
    // 【手正在靠近】
    energy += abs(pureTrend); // 累加能量 (取絕對值變正數加總)
    activeCount++;            // 增加連續次數
  }
  else
  {
    // 【手停止靠近或雜訊消失】
    // 只要有一筆資料沒達標，能量跟計數就清零，這能過濾掉單次突波
    energy = 0;
    activeCount = 0;
  }

  // 6. 狀態判定 (亮燈與熄滅)
  if (!handDetected)
  {
    // --- 進入判定 ---
    // 必須同時滿足：累積能量夠大 且 至少連續偵測到兩次 (這就是區別突波的保險)
    if (energy >= ENERGY_GOAL && activeCount >= 3)
    {
      handDetected = true;
      ledcWrite(ledPwmChannel, 255); // 點亮 LED
      Serial.println(">>>>> [TRIGGER] Hand Detected! <<<<<");
    }
  }
  else
  {
    // --- 離開判定 ---
    // 如果偵測到一個明顯的正向趨勢 (手抽離)，或是趨勢歸零一段時間 (由讀值判斷)
    if (pureTrend >= LEAVE_THRESH)
    {
      handDetected = false;
      ledcWrite(ledPwmChannel, 0); // 熄滅 LED
      Serial.println(">>>>> [RELEASE] Hand Moved Away <<<<<");
    }
  }

  // // 7. 格式化監看輸出 (對齊版)
  // // %[標記][寬度][型態] -> %-8ld 代表左對齊，佔 8 格寬度的長整數
  // Serial.printf("Trend:%-8ld \t", pureTrend);
  // Serial.printf("Energy:%-8ld \t", energy);
  // Serial.printf("Goal:%-6ld \t", ENERGY_GOAL);
  // Serial.printf("Count:%-4d \t", activeCount);
  // Serial.printf("LED:%-4s \n", handDetected ? "ON" : "OFF");

  // 使用 printf 的對齊格式：
  // %-10lu : 靠左對齊，佔 10 格 (Raw Data)
  // %+8ld  : 顯示正負號，並佔 8 格 (Trend)
  // %-8ld  : 靠左對齊，佔 8 格 (Energy)
  Serial.printf("Raw:%-10lu | ", capa);
  Serial.printf("Trend:%+8ld | ", pureTrend); // %+ 號會強制顯示 + 或 -，非常利於對齊
  Serial.printf("Energy:%-8ld | ", energy);
  Serial.printf("Count:%-4d | ", activeCount);
  Serial.printf("LED:%-3s\n", handDetected ? "ON" : "OFF");

  // 更新時間戳記
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
  Wire.setClock(100000L); // Set I2C clock to 100kHz (slower for stability)
  // bool capOk = capsense.begin(0x4, 0x6, 0x5, false); // chanMask, autoscanSeq, deglitchValue, 晶震
  // chanMask: 0x01 (CH0), 0x02 (CH1), 0x04 (CH2), 0x08 (CH3)
  // autoscanSeq:
  // deglitchValue: 0x1(1 MHz), 0x4(3.3 MHz), 0x5(10 MHz), 0x7(33 MHz)
  // 晶震: true(內部), false(外部)
  bool capOk = capsense.begin(0x8, 0x6, 0x5, false); //

  // setFdcIDrive(2, 20); //建議先從 10 到 15 之間嘗試，直到示波器底部變圓
  // setFdcIDrive(3, 10);

  if (capOk)
  {
    delay(50); // Wait for sensor stabilization

    // proxThresh = capsense.getReading28(2);
    // proxThresh = readFiltered();
    // Serial.print("Initial proxThresh: ");
    // Serial.println(proxThresh);
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
  if (!handDetected)
  {
    ledcWrite(ledPwmChannel, osc.getPwmValue());
  }
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
    modeStr = "SENSING_ENERGY";           // 更改模式名稱，方便在 Serial 監控
    handleEnergyDetection(currentMillis); // 調用新的能量判定函式
  }
  else if (status.isAnyMotion == true && (currentMillis - status.lastChangeTime) >= 50)
  {
    modeStr = "SENSING_ENERGY";           // 更改模式名稱，方便在 Serial 監控
    handleEnergyDetection(currentMillis); // 調用新的能量判定函式
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

  //tension.apply(planned, currentPosiArray); // 張力安全調整

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