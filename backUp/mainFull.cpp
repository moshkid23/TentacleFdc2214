// 「pos-anchor 安全張力＋隨機停止」
#include <Arduino.h>

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
static bool speedMorph[3] = {false, false, false};
static bool morphUp[3] = {false, false, false}; // true=慢→快, false=快→慢
static float morphFactor[3] = {1.0, 1.0, 1.0};  // 當前速度倍率

// 全域變數（放在檔案頂端）
enum StopModeState
{
  NORMAL,
  SLOWING,
  STOPPED,
  RESUMING
};
StopModeState stopState = NORMAL;
unsigned long stopTriggerTime = 0;
float speedFactor = 1.0f;

// === 可調參數（建議放在 constants.h）===
constexpr int STOP_CHANCE_PER_CHECK = 5; // 每次檢查有 5% 機率觸發
constexpr int CHECK_INTERVAL_MS = 3000;  // 每 3 秒檢查一次
constexpr int MIN_INTERVAL_MS = 10000;   // 觸發後至少 10 秒不重複
constexpr int SLOW_DOWN_MS = 8000;
constexpr int STOP_DURATION_MS = 1500;
constexpr int SPEED_UP_MS = 8000;

// 靜態變數：記錄上次檢查時間 & 上次觸發時間
static unsigned long lastCheckTime = 0;
static unsigned long lastTriggerTime = 0;

void updateRandomStopMode()
{
  unsigned long now = millis();

  // === 1. OSC 模式強制關閉停止效果 ===
  if (!randomMode)
  {
    stopState = NORMAL;
    speedFactor = 1.0f;
    return;
  }

  // === 2. 僅在 NORMAL 狀態且到達檢查時間才檢查 ===
  if (stopState == NORMAL && now - lastCheckTime >= CHECK_INTERVAL_MS)
  {
    lastCheckTime = now;

    // 檢查最小間隔
    if (now - lastTriggerTime >= MIN_INTERVAL_MS && random(100) < STOP_CHANCE_PER_CHECK)
    {
      stopState = SLOWING;
      stopTriggerTime = now;
      lastTriggerTime = now;
      Serial.println("Random STOP triggered!");
    }
  }

  // === 3. 狀態機（不變）===
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

  // 強制限制
  if (speedFactor > 1.0f)
    speedFactor = 1.0f;
  if (speedFactor < 0.0f)
    speedFactor = 0.0f;
}

// ---- 協調約束常數與函式（全域）----
constexpr int MAX_ENCODER = 17000;
constexpr int LIMIT_HIGH = 12000; // x=17000 時，其他至少 12000
constexpr int LIMIT_MID = 5000;   // x=8500  時，其他至少 5000
constexpr float CURVE_K = 1.3f;   // 曲線係數
inline int minOtherMotor(int x)
{
  float ratio = (float)x / (float)MAX_ENCODER;
  if (ratio < 0.0f)
    ratio = 0.0f;
  if (ratio > 1.0f)
    ratio = 1.0f;

  // 主曲線
  float y = powf(ratio, CURVE_K) * LIMIT_HIGH;

  // 修正 8500→5000 對應
  float correction = LIMIT_MID - (powf(0.5f, CURVE_K) * LIMIT_HIGH);
  y += correction * (1.0f - ratio);

  return (int)y;
}

// wiFi & OSC 庫
#include <WiFi.h>
#include <OSCBundle.h>
#include <OSCMessage.h> // OSC Send
#include <OSCData.h>
const char *ssid = "Perform2";
const char *pass = "qqqqqqq2";
WiFiUDP Udp;                             // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192, 168, 1, 200); // remote IP of your computer
const unsigned int localPort = 9100;     // local port to listen for UDP packets at the NodeMCU
const unsigned int outPort = 9101;       // remote port of the target device where the NodeMCU sends OSC to
// OSC 接收變數
int oscTarget[3] = {0, 0, 0}; // 記錄 OSC 模式的最後目標
void motor1Target(OSCMessage &msg, int addrOffset)
{
  oscTarget[0] = msg.getInt(0); // 保存接收到的資料到 ATarget
}
void motor2Target(OSCMessage &msg, int addrOffset)
{
  oscTarget[1] = msg.getInt(0); // 保存接收到的資料到 BTarget
}
void motor3Target(OSCMessage &msg, int addrOffset)
{
  oscTarget[2] = msg.getInt(0); // 保存接收到的資料到 CTarget
}
void P(OSCMessage &msg, int addrOffset)
{
  kp = msg.getInt(0); // 保存接收到的資料到 aP
}
void I(OSCMessage &msg, int addrOffset)
{
  ki = msg.getInt(0); // 保存接收到的資料到 aI
}
void D(OSCMessage &msg, int addrOffset)
{
  kd = msg.getInt(0); // 保存接收到的資料到 aD
}
void OSC()
{
  OSCMessage msgIN;
  int size;
  if ((size = Udp.parsePacket()) > 0)
  {
    while (size--)
      msgIN.fill(Udp.read());
    if (!msgIN.hasError())
    {
      msgIN.route("/motor1Target", motor1Target);
      msgIN.route("/motor2Target", motor2Target);
      msgIN.route("/motor3Target", motor3Target);
      msgIN.route("/P", P);
      msgIN.route("/I", I);
      msgIN.route("/D", D);
    }
  }
}

// 電容感測器設定
#include <Wire.h>
#include "FDC2214.h"
FDC2214 capsense(FDC2214_I2C_ADDR_0); // FDC2214_I2C_ADDR_0 對應 0x2A
unsigned long capa;
unsigned long previousCapaClose = 0; // 上次讀取的 capa 值
unsigned long proxThresh;
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
  return y;
}

void setup()
{
  Serial.begin(115200);
  // 設定靜態 IP
  WiFi.config(IPAddress(192, 168, 1, 30), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, pass);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20)
  {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected!");
  }
  else
  {
    Serial.println("WiFi connect failed!");
  }
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

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
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("wifi re-connecting");
    WiFi.reconnect();
  }
  // 處理 OSC 訊息
  OSC();

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
                  oscTarget[0], oscTarget[1], oscTarget[2]);

    // 新增：重置停止模式（立即恢復正常速度）
    stopState = NORMAL;
    speedFactor = 1.0f;
    stopTriggerTime = 0; // 可選：清零時間
  }

  // 讀取電容值
  static unsigned long previousMillis = 0; // 上次讀取的時間
  unsigned long currentMillis = millis();  // 當前時間
  if (currentMillis - previousMillis >= 40)
  {
    // capa = capsense.getReading28(2); // Read CH2
    capa = readFiltered();
    // Serial.println(capa);            // Output single value
    previousMillis = currentMillis; // 更新上次讀取時間
  }

  // 觸摸檢測
  static bool lastAnyMotion = false;
  static unsigned long lastAnyMotionChangeTime = 0; // 記錄 anyMotion 改變的時間
  static unsigned long lastReadTimeClose = 0;       // 上次讀取的時間
  static unsigned long lastReadTimeAway = 0;        // 上次讀取的時間
  // 靜止狀態檢測
  if (anyMotion == false && (currentMillis - lastAnyMotionChangeTime) >= 50)
  {
    if (currentMillis - lastReadTimeClose >= 40)
    {
      long diff = (long)capa - (long)previousCapaClose;
      // Serial.print(capa);
      // Serial.print(" diff: ");
      // Serial.print(diff);
      if (diff < -200)
      {
        proxThresh = capa;
        // Serial.print(" 接近oooo ");
        // Serial.print(proxThresh);
        ledcWrite(ledPwmChannel, 255);
      }
      previousCapaClose = capa;
      lastReadTimeClose = currentMillis;
      // Serial.println(" ");
    }
    if (currentMillis - lastReadTimeAway >= 80)
    {
      if (capa - proxThresh >= 200)
      {
        // Serial.println("離開xxxx ");
        ledcWrite(ledPwmChannel, 0);
      }
      lastReadTimeAway = currentMillis;
    }
  }
  // 移動狀態檢測
  else if (anyMotion == true && (currentMillis - lastAnyMotionChangeTime) >= 50)
  {
    if (currentMillis - lastReadTimeClose >= 40)
    {
      long diff = (long)capa - (long)previousCapaClose;
      // Serial.print(capa);
      // Serial.print(" diff: ");
      // Serial.print(diff);
      if (diff < -300)
      {
        proxThresh = capa;
        // Serial.print(" 移動接近proxThresh:");
        // Serial.print(proxThresh);
        ledcWrite(ledPwmChannel, 255);
      }
      else if (capa - proxThresh > 300)
      {
        // Serial.print(" 移動離開xxxx");
        ledcWrite(ledPwmChannel, 0);
      }
      // Serial.println("");
      previousCapaClose = capa;
      lastReadTimeClose = currentMillis;
    }
  }

  // ===== 隨機運動的持久狀態（所有階段都會用到）=====
  static unsigned long nextUpdate[3] = {0, 0, 0};   // 每顆更新時機
  static int randomCenter[3] = {8000, 8000, 8000};  // 隨機中心
  static int randomSpeedLimit[3] = {150, 150, 150}; // 速度上限（馬達輸出區會用到）
  static float sinePhase[3] = {0.0, 1.0, 2.0};
  static float sineSpeed[3] = {0.0004, 0.0003, 0.0005};
  // 速度變化變數
  static unsigned long morphStartTime[3] = {0, 0, 0}; // 記錄開始時間
  static unsigned long morphEndTime[3] = {0, 0, 0};   // 記錄結束時間
  static float morphPhaseOffset[3] = {0.0, 0.0, 0.0};

  // ==== ✨ 階段1：先計算三顆的「下一步候選值 planned[]」(不直接動 targetArray) ====
  int planned[3];
  {
    unsigned long now = millis();

    // 產生基準目標（random 或 OSC）
    for (int i = 0; i < 3; ++i)
    {
      if (randomMode)
      {
        // 每顆馬達每 3~7 秒更新自己的 random 中心與速度層級
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

          // 是否進入速度變形（保留你原本的變速機制即可）
          int morphPick = random(100);
          if (morphPick < 5)
          {
            speedMorph[i] = true;
            morphUp[i] = true;
            morphFactor[i] = 0.3f;
            morphStartTime[i] = millis();
            morphEndTime[i] = 0;
          }
          else if (morphPick < 10)
          {
            speedMorph[i] = true;
            morphUp[i] = false;
            morphFactor[i] = 1.0f;
            morphStartTime[i] = millis();
            morphEndTime[i] = 0;
          }
          else
          {
            speedMorph[i] = false;
          }

          sineSpeed[i] = random(20, 60) / 100000.0f;
          sinePhase[i] = random(0, 628) / 100.0f;
          nextUpdate[i] = now + random(3000, 12000);
        }

        // 呼吸波
        float wave = sin((millis() * sineSpeed[i]) + sinePhase[i]);
        int waveOffset = (int)(wave * 3000); // ±3000 振幅
        int target = constrain(randomCenter[i] + waveOffset, 0, 17000);

        // LERP（隨機模式可稍快一點）
        float t = 0.05f; // 0.05慢 ~ 0.15快
        planned[i] = targetArray[i] + (int)((target - targetArray[i]) * t);
      }
      else
      {
        // OSC 直控 → 直接指向 oscTarget（或也可加輕微 LERP）
        planned[i] = oscTarget[i];
      }
      planned[i] = constrain(planned[i], 0, 17000);
    }
  }

  updateRandomStopMode();

  // ===== 以「實際位置 pos」選主張力軸 + 遲滯 =====
  static int lastMaster = 0;
  const int MASTER_HYST = 1200; // 主軸切換遲滯，避免頻繁跳軸（可調 800~2000）
  int posNow[3] = {posiArray[0], posiArray[1], posiArray[2]};

  // 先找當前 pos 最大者
  int cand = 0;
  int gpos = posNow[0];
  for (int i = 1; i < 3; ++i)
    if (posNow[i] > gpos)
    {
      gpos = posNow[i];
      cand = i;
    }

  // 遲滯：只有當 cand 比現任主軸高出一定幅度才換
  int master = lastMaster;
  if (cand != lastMaster && posNow[cand] > posNow[lastMaster] + MASTER_HYST)
  {
    master = cand;
    // 可選：Serial.printf("🧭 master pos-switch %d→%d | pos=(%d,%d,%d)\n",
    //                     lastMaster, master, posNow[0],posNow[1],posNow[2]);
  }
  lastMaster = master;

  // ===== 用「實際張力錨」計 safeMin（避免被低 planned 拖低）=====
  int anchor = max(planned[master], posNow[master]); // 關鍵：用 max(planned, pos)
  int safeMin = (anchor >= 100) ? minOtherMotor(anchor) : 0;
  // int safeMin = minOtherMotor(anchor);

  // ===== 套用 safeMin 到其餘兩軸（一次性）=====
  for (int j = 0; j < 3; ++j)
  {
    if (j == master)
      continue;
    if (planned[j] < safeMin)
      planned[j] = safeMin;
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

  for (int i = 0; i < 3; ++i)
  {
    // 更新編碼器位置
    int pos = encoders[i].getPosition();
    if (pos != 0)
    {
      posiArray[i] += pos;
      encoders[i].reset();
    }

    // ========= motion判斷 =========
    static unsigned long lastMoveTime[3] = {0, 0, 0};
    unsigned long loopNow = millis();
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

    // ==== 速度漸變處理 (時間制 + sin 線性曲線 + 隨機長度 + 隨機相位) ====
    if (speedMorph[i])
    {
      static unsigned long morphDuration[3] = {2000, 2000, 2000}; // 預設
      if (morphEndTime[i] == 0)
      {
        morphDuration[i] = random(1500, 5000);        // 1.5s~5s
        morphPhaseOffset[i] = random(0, 30) / 100.0f; // 0.0~0.3
      }

      unsigned long elapsed = millis() - morphStartTime[i];
      float progress = (float)elapsed / (float)morphDuration[i];

      if (progress >= 1.0f)
      {
        progress = 1.0f;
        speedMorph[i] = false; // 🔚 自動結束 morph
        morphEndTime[i] = millis();
        // Serial.printf("✅ M%d morph 結束, 維持 %.1fs\n", i,
        //               (morphEndTime[i] - morphStartTime[i]) / 1000.0f);
      }

      // 🎚️ 使用 sin 曲線讓變化更自然（ease-in / ease-out）
      float phase = progress + morphPhaseOffset[i];
      if (phase > 1.0f)
        phase = 1.0f; // 避免超出
      float eased = sin(phase * PI / 2.0f);

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
      morphFactor[i] = 1.0f;
    }

    if (randomMode)
    {
      if (speedMorph[i])
      {
        // ✨ 當使用加速度變化時，不受 randomSpeedLimit 限制
        // （但仍限制在 255 以防爆 PWM）
        rawPower = constrain(rawPower * morphFactor[i], 0, 200);
      }
      else
      {
        // 🧭 正常分層狀態：受限於 randomSpeedLimit
        rawPower = constrain(rawPower, 0, randomSpeedLimit[i]);
      }
    }

    // === 停止模式：乘 speedFactor（自動用當下功率作為起/終點）===
    int finalPower = (int)(rawPower * speedFactor); // 這裡乘！
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
