/*
 * 光光星 PedalStar - 互動式兒童遊樂設施控制系統
 *
 * 功能：透過霍爾感測器偵測腳踏車輪子旋轉,計算能量累積,
 *       並以 32 顆單色 LED 燈提供即時視覺回饋（進度條效果）
 *
 * 硬體架構：
 *   - Arduino Uno (ATmega328P)
 *   - 霍爾感測器模組 (A3144) × 1
 *   - PCA9685 PWM 驅動模組 × 2 (I²C 控制)
 *   - 單色 LED × 32 (可調整數量)
 *
 * 開發環境：PlatformIO + Arduino Framework
 * 版本：v2.0
 * 日期：2025-10-15
 * 更新：改用 PCA9685 驅動 32 顆單色 LED,LED 數量可調整
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== 腳位定義 ====================
// 霍爾感測器模組（查證來源：A3144 模組，內建上拉電阻與去耦電容）
// 接線：VCC->5V, GND->GND, DO->D2
// 模組優點：內建電路，接線簡單，穩定性高
// 工作原理：磁鐵 S 極靠近時輸出 LOW，遠離時輸出 HIGH
#define HALL_SENSOR_PIN 2 // D2 = INT0 中斷腳位

// PCA9685 模組配置（查證來源：16 通道 PWM 驅動器，I²C 通訊）
// 接線：VCC->5V, GND->GND, SDA->A4, SCL->A5
// 模組優點：16 個 PWM 通道，透過 I²C 控制，節省 Arduino 腳位
// I²C 位址：0x40 (預設), 0x41 (焊接 A0 跳線)
// 每通道電流：25mA @ 5V (開漏輸出)
#define PCA9685_ADDR_1 0x40 // PCA9685 #1 位址（預設）
#define PCA9685_ADDR_2 0x41 // PCA9685 #2 位址（焊接 A0）
#define PCA9685_FREQ 1000   // PWM 頻率 (Hz)，1000Hz 足夠 LED 使用

// ⚙️ LED 數量配置（可調整）
// 注意：每個 PCA9685 有 16 個通道
// - 使用 1 個 PCA9685: 最多 16 顆
// - 使用 2 個 PCA9685: 最多 32 顆
#define NUM_LEDS 25 // ⚙️ LED 總數（可調整：1-32）

// ==================== 參數配置 ====================
#define TIME_WINDOW 1000    // 時間窗口：1秒（提高輸出頻率）
#define MAX_ROTATIONS 50    // 最大圈數⚙️ 調高此值可增加難度
#define DEBOUNCE_DELAY 10   // 去彈跳延遲：10ms（提高靈敏度,適應快速踩踏）⚙️
#define ENERGY_LEVELS 5     // 能量等級數量
#define PWM_MAX 255         // Arduino PWM 最大值（8 位元，0-255）
#define PRINT_ROTATION true // 是否即時輸出每次旋轉偵測

// 能量衰減參數
#define IDLE_TIMEOUT 3000   // 閒置超過 3 秒開始衰減
#define DECAY_RATE 4        // 衰減速率：每秒降低 4% (= 1顆LED) ⚙️ 調高此值能量消失更快
#define DECAY_INTERVAL 1000 // 衰減檢查間隔：1 秒

// ==================== 全域變數 ====================
// PCA9685 驅動物件
Adafruit_PWMServoDriver pca9685_1 = Adafruit_PWMServoDriver(PCA9685_ADDR_1);
Adafruit_PWMServoDriver pca9685_2 = Adafruit_PWMServoDriver(PCA9685_ADDR_2);

// PWM 亮度值（PCA9685 使用 12-bit，範圍 0-4095）
#define PWM_OFF 0
#define PWM_ON 4095 // 全亮

// 霍爾感測器相關變數（使用 volatile 因為在 ISR 中修改）
volatile unsigned int rotationCount = 0;
volatile unsigned long lastInterruptTime = 0;
volatile bool magnetDetected = false; // 追蹤磁鐵是否已靠近（偵測完整波形用）

// 能量計算相關變數
unsigned long previousMillis = 0;
unsigned int currentEnergy = 0;
int currentEnergyLevel = 0;

// 能量衰減相關變數
unsigned long lastRotationTime = 0; // 最後一次旋轉的時間
unsigned long lastDecayTime = 0;    // 最後一次衰減的時間

// ==================== 中斷服務函式 ====================
/*
 * 霍爾感測器中斷處理函式（改為 CHANGE 模式）
 * 偵測策略：完整波形偵測（LOW→HIGH 才計數）
 * 1. FALLING (HIGH→LOW): 磁鐵靠近，設定 magnetDetected = true
 * 2. RISING (LOW→HIGH): 磁鐵遠離，若 magnetDetected = true 則計數（完成一圈）
 * 功能：確保偵測完整的磁鐵經過週期，避免誤計數
 */
void hallSensorISR()
{
  unsigned long currentTime = millis();
  int sensorState = digitalRead(HALL_SENSOR_PIN);

  // 去彈跳處理：10ms 內的重複觸發視為雜訊（已針對快速踩踏優化）
  if (currentTime - lastInterruptTime > DEBOUNCE_DELAY)
  {
    if (sensorState == LOW)
    {
      // 磁鐵靠近（FALLING edge）
      magnetDetected = true;
      if (PRINT_ROTATION)
      {
        Serial.print("🧲 磁鐵靠近 (時間: ");
        Serial.print(currentTime / 1000.0, 2);
        Serial.println("秒)");
      }
    }
    else if (sensorState == HIGH && magnetDetected)
    {
      // 磁鐵遠離（RISING edge）且之前已偵測到靠近 → 完成一圈
      rotationCount++;
      magnetDetected = false;
      lastRotationTime = currentTime; // 更新最後旋轉時間（用於能量衰減）

      // 即時輸出每次旋轉偵測（可選）
      if (PRINT_ROTATION)
      {
        Serial.print("✓ 完成旋轉 #");
        Serial.print(rotationCount);
        Serial.print(" (時間: ");
        Serial.print(currentTime / 1000.0, 2);
        Serial.println("秒)");
      }
    }

    lastInterruptTime = currentTime;
  }
}

// ==================== LED 控制函式 ====================
/*
 * 設定單顆 LED 的開關狀態
 * @param ledNum LED 編號 (0-31 對應 LED1-LED32)
 * @param state LED 狀態 (true=點亮, false=熄滅)
 */
void setLED(int ledNum, bool state)
{
  if (ledNum < 0 || ledNum >= NUM_LEDS)
    return;

  // 判斷使用哪個 PCA9685 模組
  if (ledNum < 16)
  {
    // LED 0-15: PCA9685 #1
    pca9685_1.setPWM(ledNum, 0, state ? PWM_ON : PWM_OFF);
  }
  else
  {
    // LED 16-31: PCA9685 #2
    pca9685_2.setPWM(ledNum - 16, 0, state ? PWM_ON : PWM_OFF);
  }
}

/*
 * 設定所有 LED 為相同狀態
 * @param state LED 狀態 (true=點亮, false=熄滅)
 */
void setAllLEDs(bool state)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLED(i, state);
  }
}

/*
 * 關閉所有 LED
 */
void clearAllLEDs()
{
  setAllLEDs(false);
}

/*
 * 根據能量等級點亮對應數量的 LED（進度條效果）
 * @param energyLevel 能量等級 (0-100 百分比)
 */
void updateLEDsByEnergy(int energyLevel)
{
  // 限制能量等級範圍
  energyLevel = constrain(energyLevel, 0, 100);

  // 將能量等級映射到 LED 數量 (0-NUM_LEDS)
  int numLEDsToLight = map(energyLevel, 0, 100, 0, NUM_LEDS);
  numLEDsToLight = constrain(numLEDsToLight, 0, NUM_LEDS);

  // 點亮對應數量的 LED（進度條效果）
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLED(i, i < numLEDsToLight);
  }

  // 序列埠除錯輸出
  Serial.print("能量: ");
  Serial.print(energyLevel);
  Serial.print("% -> 點亮 ");
  Serial.print(numLEDsToLight);
  Serial.print("/");
  Serial.print(NUM_LEDS);
  Serial.println(" 顆 LED");
}

/*
 * 計算能量值
 * 根據時間窗口內的旋轉圈數計算能量百分比
 * @return 能量等級 (0-100)
 */
int calculateEnergy()
{
  // 將旋轉圈數映射到能量百分比 (0-100)
  int energy = map(rotationCount, 0, MAX_ROTATIONS, 0, 100);
  energy = constrain(energy, 0, 100);

  return energy;
}

/*
 * LED 啟動測試動畫
 * 依序點亮每顆 LED，測試硬體連接是否正常
 */
void ledStartupTest()
{
  Serial.println("開始 LED 測試...");
  Serial.print("LED 數量: ");
  Serial.println(NUM_LEDS);

  // 測試全亮
  Serial.println("測試全亮...");
  setAllLEDs(true);
  delay(500);

  // 測試全暗
  Serial.println("測試全暗...");
  setAllLEDs(false);
  delay(500);

  // 逐顆點亮測試（流水燈效果）
  Serial.println("逐顆點亮測試（流水燈）...");
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLED(i, true);
    delay(50); // 加快速度以適應更多 LED
  }
  delay(500);

  // 逐顆熄滅測試
  Serial.println("逐顆熄滅測試...");
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLED(i, false);
    delay(50);
  }
  delay(300);

  // 清除所有 LED
  clearAllLEDs();
  Serial.println("LED 測試完成！\n");
}

// ==================== 初始化函式 ====================
void setup()
{
  // 初始化序列埠（除錯用）
  Serial.begin(9600);
  Serial.println("========================================");
  Serial.println("   光光星 PedalStar 系統啟動");
  Serial.println("========================================");
  Serial.println();

  // 顯示系統資訊
  Serial.println("【系統資訊】");
  Serial.println("硬體配置：");
  Serial.println("  - Arduino Uno (ATmega328P)");
  Serial.println("  - 霍爾感測器模組 (A3144)");
  Serial.println("  - PCA9685 PWM 驅動模組 × 2");
  Serial.print("  - 單色 LED × ");
  Serial.println(NUM_LEDS);
  Serial.println();

  Serial.println("腳位配置：");
  Serial.print("  - 霍爾感測器: D");
  Serial.print(HALL_SENSOR_PIN);
  Serial.println(" (INT0)");
  Serial.println("  - I²C 通訊: A4(SDA), A5(SCL)");
  Serial.print("  - PCA9685 #1: 0x");
  Serial.print(PCA9685_ADDR_1, HEX);
  Serial.println(" (LED 0-15)");
  Serial.print("  - PCA9685 #2: 0x");
  Serial.print(PCA9685_ADDR_2, HEX);
  Serial.println(" (LED 16-31)");
  Serial.println();

  Serial.println("參數設定：");
  Serial.print("  - 時間窗口: ");
  Serial.print(TIME_WINDOW / 1000);
  Serial.println(" 秒");
  Serial.print("  - 最大圈數: ");
  Serial.println(MAX_ROTATIONS);
  Serial.print("  - 去彈跳延遲: ");
  Serial.print(DEBOUNCE_DELAY);
  Serial.println(" ms（快速踩踏優化）");
  Serial.print("  - LED 數量: ");
  Serial.print(NUM_LEDS);
  Serial.println(" (可調整)");
  Serial.print("  - 能量階差: ");
  Serial.print(100.0 / NUM_LEDS, 2);
  Serial.println("% 每顆");
  Serial.println();

  // 初始化 I²C 通訊
  Wire.begin();
  Serial.println("✓ I²C 通訊初始化完成");

  // 初始化 PCA9685 模組
  pca9685_1.begin();
  pca9685_1.setPWMFreq(PCA9685_FREQ);
  Serial.print("✓ PCA9685 #1 初始化完成 (位址: 0x");
  Serial.print(PCA9685_ADDR_1, HEX);
  Serial.println(")");

  // 檢查是否需要初始化第二個模組
  if (NUM_LEDS > 16)
  {
    pca9685_2.begin();
    pca9685_2.setPWMFreq(PCA9685_FREQ);
    Serial.print("✓ PCA9685 #2 初始化完成 (位址: 0x");
    Serial.print(PCA9685_ADDR_2, HEX);
    Serial.println(")");
  }
  else
  {
    Serial.println("  (PCA9685 #2 未使用)");
  }

  // 設定霍爾感測器腳位為輸入
  // 注意：模組已內建上拉電阻，使用 INPUT 即可
  pinMode(HALL_SENSOR_PIN, INPUT);
  Serial.println("✓ 霍爾感測器初始化完成");

  // 啟用外部中斷 INT0（D2），偵測所有電位變化（CHANGE）
  // 模組特性：磁鐵 S 極靠近時輸出 LOW，遠離時輸出 HIGH
  // 策略：偵測完整波形（LOW→HIGH）才算完成一圈
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN),
                  hallSensorISR, CHANGE);
  Serial.println("✓ 中斷服務已啟用 (CHANGE mode - 完整波形偵測)");

  // 關閉所有 LED（初始狀態）
  clearAllLEDs();
  Serial.println("✓ 所有 LED 已關閉");
  Serial.println();

  // 執行 LED 啟動測試
  ledStartupTest();

  // 系統就緒
  Serial.println("========================================");
  Serial.println("   系統就緒！開始偵測輪子旋轉...");
  Serial.println("========================================");
  Serial.println();
  Serial.println("【即時資料】");
  Serial.println("時間(秒) | 圈數 | 能量(%) | LED數量 | 狀態");
  Serial.println("---------|------|---------|---------|--------");

  // 初始化計時器
  previousMillis = millis();
  lastRotationTime = millis();
  lastDecayTime = millis();
}

// ==================== 主迴圈 ====================
void loop()
{
  unsigned long currentMillis = millis();

  // 檢查是否到達時間窗口
  if (currentMillis - previousMillis >= TIME_WINDOW)
  {
    // 計算能量值（根據本時間窗口的旋轉次數）
    int newEnergy = calculateEnergy();

    // 累加新能量到當前能量（不超過 100%）
    if (rotationCount > 0)
    {
      currentEnergy = constrain(currentEnergy + newEnergy, 0, 100);
    }

    // 更新 LED 顯示
    updateLEDsByEnergy(currentEnergy);

    // 序列埠輸出統計資料
    unsigned long elapsedSeconds = currentMillis / 1000;
    Serial.print(elapsedSeconds);
    Serial.print("      | ");
    Serial.print(rotationCount);
    Serial.print("    | ");
    Serial.print(currentEnergy);
    Serial.print("%      | ");
    int numLEDs = map(currentEnergy, 0, 100, 0, NUM_LEDS);
    Serial.print(numLEDs);
    Serial.print("        | ");

    // 顯示狀態
    if (rotationCount > 0)
    {
      Serial.println("踩踏中");
    }
    else
    {
      unsigned long idleTime = currentMillis - lastRotationTime;
      if (idleTime < IDLE_TIMEOUT)
      {
        Serial.println("等待中");
      }
      else
      {
        Serial.println("衰減中");
      }
    }

    // 重置計數器與計時器
    rotationCount = 0;
    previousMillis = currentMillis;
  }

  // 能量衰減機制：閒置超過 3 秒後，每秒降低 5%
  unsigned long idleTime = currentMillis - lastRotationTime;

  if (idleTime >= IDLE_TIMEOUT && currentEnergy > 0)
  {
    // 檢查是否到達衰減間隔（每秒執行一次）
    if (currentMillis - lastDecayTime >= DECAY_INTERVAL)
    {
      // 降低能量 5%
      int decayAmount = DECAY_RATE;
      currentEnergy = (currentEnergy > decayAmount) ? (currentEnergy - decayAmount) : 0;

      // 更新 LED 顯示
      updateLEDsByEnergy(currentEnergy);

      // 輸出衰減資訊
      Serial.print("⚠️  能量衰減: ");
      Serial.print(currentEnergy);
      Serial.print("% (閒置時間: ");
      Serial.print(idleTime / 1000.0, 1);
      Serial.println("秒)");

      lastDecayTime = currentMillis;
    }
  }
  else
  {
    // 重置衰減計時器（當有活動時）
    lastDecayTime = currentMillis;
  }

  // 主迴圈不使用 delay()，保持非阻塞運行
}
