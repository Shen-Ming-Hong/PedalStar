/*
 * 光光星 PedalStar - 互動式兒童遊樂設施控制系統
 *
 * 功能：透過霍爾感測器偵測腳踏車輪子旋轉，計算能量累積，
 *       並以 5 顆 RGB LED 燈提供即時視覺回饋
 *
 * 硬體架構：
 *   - Arduino Uno (ATmega328P)
 *   - 霍爾感測器模組 (A3144) × 1
 *   - PCA9685 PWM 驅動模組 × 1
 *   - RGB LED 模組（共陰極）× 5
 *
 * 開發環境：PlatformIO + Arduino Framework
 * 版本：v1.0
 * 日期：2025-10-07
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

// PCA9685 PWM 驅動模組（查證來源：16 通道 I2C PWM 驅動板）
// 接線：VCC->5V, GND->GND, SDA->A4, SCL->A5
// I2C 位址：0x40（預設）
// Arduino Uno I2C 腳位：A4 (SDA), A5 (SCL) - 硬體固定，不可更改
#define PCA9685_ADDRESS 0x40 // I2C 位址
#define PCA9685_FREQ 1000    // PWM 頻率 1000Hz

// RGB LED 通道配置（PCA9685 的 16 個通道：Ch0-Ch15）
// 每顆 RGB LED 使用 3 個通道（R, G, B）
#define LED1_R 0 // LED #1 紅色 → PCA9685 通道 0
#define LED1_G 1 // LED #1 綠色 → PCA9685 通道 1
#define LED1_B 2 // LED #1 藍色 → PCA9685 通道 2

#define LED2_R 3 // LED #2 紅色 → PCA9685 通道 3
#define LED2_G 4 // LED #2 綠色 → PCA9685 通道 4
#define LED2_B 5 // LED #2 藍色 → PCA9685 通道 5

#define LED3_R 6 // LED #3 紅色 → PCA9685 通道 6
#define LED3_G 7 // LED #3 綠色 → PCA9685 通道 7
#define LED3_B 8 // LED #3 藍色 → PCA9685 通道 8

#define LED4_R 9  // LED #4 紅色 → PCA9685 通道 9
#define LED4_G 10 // LED #4 綠色 → PCA9685 通道 10
#define LED4_B 11 // LED #4 藍色 → PCA9685 通道 11

#define LED5_R 12 // LED #5 紅色 → PCA9685 通道 12
#define LED5_G 13 // LED #5 綠色 → PCA9685 通道 13
#define LED5_B 14 // LED #5 藍色 → PCA9685 通道 14

#define NUM_LEDS 5 // LED 總數

// ==================== 參數配置 ====================
#define TIME_WINDOW 1000    // 時間窗口：1秒（提高輸出頻率）
#define MAX_ROTATIONS 10    // 最大圈數（用於映射，1秒內約10圈）
#define DEBOUNCE_DELAY 50   // 去彈跳延遲：50ms
#define ENERGY_LEVELS 5     // 能量等級數量
#define PWM_MAX 4095        // PCA9685 PWM 最大值（12 位元）
#define PRINT_ROTATION true // 是否即時輸出每次旋轉偵測

// 能量衰減參數
#define IDLE_TIMEOUT 3000   // 閒置超過 3 秒開始衰減
#define DECAY_RATE 5        // 衰減速率：每秒降低 5%
#define DECAY_INTERVAL 1000 // 衰減檢查間隔：1 秒

// ==================== 全域變數 ====================
// PCA9685 PWM 驅動物件
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

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

  // 去彈跳處理：50ms 內的重複觸發視為雜訊
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
 * 設定單顆 LED 的 RGB 顏色
 * @param ledNum LED 編號 (0-4 對應 LED1-LED5)
 * @param r 紅色亮度 (0-4095)
 * @param g 綠色亮度 (0-4095)
 * @param b 藍色亮度 (0-4095)
 */
void setLEDColor(int ledNum, uint16_t r, uint16_t g, uint16_t b)
{
  if (ledNum < 0 || ledNum >= NUM_LEDS)
    return;

  int baseChannel = ledNum * 3;      // 計算起始通道（每顆 LED 佔 3 通道）
  pwm.setPWM(baseChannel + 0, 0, r); // 紅色通道
  pwm.setPWM(baseChannel + 1, 0, g); // 綠色通道
  pwm.setPWM(baseChannel + 2, 0, b); // 藍色通道
}

/*
 * 設定所有 LED 為相同顏色
 * @param r 紅色亮度 (0-4095)
 * @param g 綠色亮度 (0-4095)
 * @param b 藍色亮度 (0-4095)
 */
void setAllLEDs(uint16_t r, uint16_t g, uint16_t b)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLEDColor(i, r, g, b);
  }
}

/*
 * 關閉所有 LED
 */
void clearAllLEDs()
{
  for (int i = 0; i < 15; i++)
  { // 通道 0-14
    pwm.setPWM(i, 0, 0);
  }
}

/*
 * 根據能量等級點亮對應數量的 LED
 * @param energyLevel 能量等級 (0-100 百分比)
 */
void updateLEDsByEnergy(int energyLevel)
{
  // 限制能量等級範圍
  energyLevel = constrain(energyLevel, 0, 100);

  // 將能量等級映射到 LED 數量 (0-5)
  int numLEDsToLight = map(energyLevel, 0, 100, 0, NUM_LEDS);
  numLEDsToLight = constrain(numLEDsToLight, 0, NUM_LEDS);

  // 根據能量等級選擇顏色
  uint16_t r, g, b;
  if (energyLevel < 20)
  {
    // 低能量：紅色
    r = PWM_MAX;
    g = 0;
    b = 0;
  }
  else if (energyLevel < 40)
  {
    // 低中能量：黃色
    r = PWM_MAX;
    g = PWM_MAX / 2;
    b = 0;
  }
  else if (energyLevel < 60)
  {
    // 中能量：綠色
    r = 0;
    g = PWM_MAX;
    b = 0;
  }
  else if (energyLevel < 80)
  {
    // 中高能量：青色
    r = 0;
    g = PWM_MAX;
    b = PWM_MAX / 2;
  }
  else
  {
    // 高能量：藍色
    r = 0;
    g = 0;
    b = PWM_MAX;
  }

  // 點亮對應數量的 LED
  for (int i = 0; i < NUM_LEDS; i++)
  {
    if (i < numLEDsToLight)
    {
      setLEDColor(i, r, g, b); // 點亮
    }
    else
    {
      setLEDColor(i, 0, 0, 0); // 熄滅
    }
  }

  // 序列埠除錯輸出
  Serial.print("能量: ");
  Serial.print(energyLevel);
  Serial.print("% -> 點亮 ");
  Serial.print(numLEDsToLight);
  Serial.print(" 顆 LED (顏色: ");
  if (energyLevel < 20)
    Serial.print("紅色");
  else if (energyLevel < 40)
    Serial.print("黃色");
  else if (energyLevel < 60)
    Serial.print("綠色");
  else if (energyLevel < 80)
    Serial.print("青色");
  else
    Serial.print("藍色");
  Serial.println(")");
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

  // 測試紅色
  Serial.println("測試紅色...");
  setAllLEDs(PWM_MAX, 0, 0);
  delay(500);

  // 測試綠色
  Serial.println("測試綠色...");
  setAllLEDs(0, PWM_MAX, 0);
  delay(500);

  // 測試藍色
  Serial.println("測試藍色...");
  setAllLEDs(0, 0, PWM_MAX);
  delay(500);

  // 逐顆點亮測試
  Serial.println("逐顆點亮測試...");
  clearAllLEDs();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLEDColor(i, PWM_MAX, PWM_MAX, PWM_MAX); // 白色
    delay(200);
  }
  delay(500);

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
  Serial.println("  - PCA9685 PWM 驅動模組");
  Serial.println("  - RGB LED 模組 × 5");
  Serial.println();

  Serial.println("腳位配置：");
  Serial.print("  - 霍爾感測器: D");
  Serial.print(HALL_SENSOR_PIN);
  Serial.println(" (INT0)");
  Serial.println("  - PCA9685 I2C: A4 (SDA), A5 (SCL)");
  Serial.print("  - I2C 位址: 0x");
  Serial.println(PCA9685_ADDRESS, HEX);
  Serial.println();

  Serial.println("參數設定：");
  Serial.print("  - 時間窗口: ");
  Serial.print(TIME_WINDOW / 1000);
  Serial.println(" 秒");
  Serial.print("  - 最大圈數: ");
  Serial.println(MAX_ROTATIONS);
  Serial.print("  - 去彈跳延遲: ");
  Serial.print(DEBOUNCE_DELAY);
  Serial.println(" ms");
  Serial.print("  - LED 數量: ");
  Serial.println(NUM_LEDS);
  Serial.println();

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

  // 初始化 PCA9685 模組
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // 內部振盪器頻率（通常為 27MHz）
  pwm.setPWMFreq(PCA9685_FREQ);         // 設定 PWM 頻率（1000Hz）

  Serial.println("✓ PCA9685 初始化完成");
  Serial.print("  PWM 頻率: ");
  Serial.print(PCA9685_FREQ);
  Serial.println(" Hz");

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
