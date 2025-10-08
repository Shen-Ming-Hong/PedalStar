/*
 * 光光星 PedalStar - 互動式兒童遊樂設施控制系統
 *
 * 功能：透過霍爾感測器偵測腳踏車輪子旋轉,計算能量累積,
 *       並以 5 顆 RG LED 燈提供即時視覺回饋（模擬星星顏色）
 *
 * 硬體架構：
 *   - Arduino Uno (ATmega328P)
 *   - 霍爾感測器模組 (A3144) × 1
 *   - RG LED 模組（共陰極，使用紅、綠雙色）× 5
 *
 * 開發環境：PlatformIO + Arduino Framework
 * 版本：v1.2
 * 日期：2025-10-08
 * 更新：優化難度參數（MAX_ROTATIONS: 10→20, DECAY_RATE: 5%→8%）
 */

#include <Arduino.h>

// ==================== 腳位定義 ====================
// 霍爾感測器模組（查證來源：A3144 模組，內建上拉電阻與去耦電容）
// 接線：VCC->5V, GND->GND, DO->D2
// 模組優點：內建電路，接線簡單，穩定性高
// 工作原理：磁鐵 S 極靠近時輸出 LOW，遠離時輸出 HIGH
#define HALL_SENSOR_PIN 2 // D2 = INT0 中斷腳位

// RG LED 腳位配置（查證來源：Arduino Uno PWM 腳位）
// Arduino Uno PWM 腳位：D3, D5, D6, D9, D10, D11（共 6 個）
// 每顆 RG LED 使用 2 個腳位（R, G），共需 10 個腳位
// 注意：D2 被霍爾感測器佔用（中斷腳位）
//
// LED 連接方式：共陰極（需要高電位點亮）
//
// 【優化接線策略 - 相鄰腳位配對】
// - 每顆 LED 的 R 和 G 使用相鄰或接近的腳位，減少跳線
// - 綠色通道優先使用 PWM 腳位（支援漸變效果）
// - 紅色通道使用數位腳位（僅需開/關控制）
//
// 腳位分配（按照 Arduino Uno 板子上的物理順序）：
#define LED1_G 3 // LED #1 綠色 → D3（PWM 腳位）
#define LED1_R 4 // LED #1 紅色 → D4（數位腳位）★ 相鄰

#define LED2_G 5 // LED #2 綠色 → D5（PWM 腳位）
#define LED2_R 7 // LED #2 紅色 → D7（數位腳位，跳過 D6）

#define LED3_G 6 // LED #3 綠色 → D6（PWM 腳位）
#define LED3_R 8 // LED #3 紅色 → D8（數位腳位）

#define LED4_G 9  // LED #4 綠色 → D9（PWM 腳位）
#define LED4_R 12 // LED #4 紅色 → D12（數位腳位）

#define LED5_G 10 // LED #5 綠色 → D10（PWM 腳位）
#define LED5_R 11 // LED #5 紅色 → D11（PWM 腳位，可當數位腳位用）★ 相鄰

#define NUM_LEDS 5 // LED 總數

// ==================== 參數配置 ====================
#define TIME_WINDOW 1000    // 時間窗口：1秒（提高輸出頻率）
#define MAX_ROTATIONS 20    // 最大圈數⚙️ 調高此值可增加難度
#define DEBOUNCE_DELAY 50   // 去彈跳延遲：50ms
#define ENERGY_LEVELS 5     // 能量等級數量
#define PWM_MAX 255         // Arduino PWM 最大值（8 位元，0-255）
#define PRINT_ROTATION true // 是否即時輸出每次旋轉偵測

// 能量衰減參數
#define IDLE_TIMEOUT 3000   // 閒置超過 3 秒開始衰減
#define DECAY_RATE 8        // 衰減速率：每秒降低 8% ⚙️ 調高此值能量消失更快
#define DECAY_INTERVAL 1000 // 衰減檢查間隔：1 秒

// ==================== 全域變數 ====================
// LED 腳位陣列（用於批次操作）
const int LED_R_PINS[NUM_LEDS] = {LED1_R, LED2_R, LED3_R, LED4_R, LED5_R};
const int LED_G_PINS[NUM_LEDS] = {LED1_G, LED2_G, LED3_G, LED4_G, LED5_G};

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
 * 設定單顆 LED 的 RG 顏色
 * @param ledNum LED 編號 (0-4 對應 LED1-LED5)
 * @param r 紅色亮度 (0-255)
 * @param g 綠色亮度 (0-255)
 */
void setLEDColor(int ledNum, uint8_t r, uint8_t g)
{
  if (ledNum < 0 || ledNum >= NUM_LEDS)
    return;

  // 紅色通道（數位腳位，僅能全亮或全暗）
  digitalWrite(LED_R_PINS[ledNum], r > 127 ? HIGH : LOW);

  // 綠色通道（PWM 腳位，支援漸變）
  analogWrite(LED_G_PINS[ledNum], g);
}

/*
 * 設定所有 LED 為相同顏色
 * @param r 紅色亮度 (0-255)
 * @param g 綠色亮度 (0-255)
 */
void setAllLEDs(uint8_t r, uint8_t g)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLEDColor(i, r, g);
  }
}

/*
 * 關閉所有 LED
 */
void clearAllLEDs()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    digitalWrite(LED_R_PINS[i], LOW);
    analogWrite(LED_G_PINS[i], 0);
  }
}

/*
 * 根據能量等級點亮對應數量的 LED（星星顏色漸變：紅→黃→綠）
 * @param energyLevel 能量等級 (0-100 百分比)
 */
void updateLEDsByEnergy(int energyLevel)
{
  // 限制能量等級範圍
  energyLevel = constrain(energyLevel, 0, 100);

  // 將能量等級映射到 LED 數量 (0-5)
  int numLEDsToLight = map(energyLevel, 0, 100, 0, NUM_LEDS);
  numLEDsToLight = constrain(numLEDsToLight, 0, NUM_LEDS);

  // 根據能量等級選擇星星顏色（紅→橙→黃→黃綠→綠）
  uint8_t r, g;
  const char *colorName;

  if (energyLevel < 20)
  {
    // 低能量：紅色（冷星）
    r = PWM_MAX;
    g = 0;
    colorName = "紅色";
  }
  else if (energyLevel < 40)
  {
    // 低中能量：橙色
    r = PWM_MAX;
    g = PWM_MAX / 3;
    colorName = "橙色";
  }
  else if (energyLevel < 60)
  {
    // 中能量：黃色（太陽色）
    r = PWM_MAX;
    g = PWM_MAX * 2 / 3;
    colorName = "黃色";
  }
  else if (energyLevel < 80)
  {
    // 中高能量：黃綠色
    r = PWM_MAX / 2;
    g = PWM_MAX;
    colorName = "黃綠色";
  }
  else
  {
    // 高能量：綠色（熱星）
    r = 0;
    g = PWM_MAX;
    colorName = "綠色";
  }

  // 點亮對應數量的 LED
  for (int i = 0; i < NUM_LEDS; i++)
  {
    if (i < numLEDsToLight)
    {
      setLEDColor(i, r, g); // 點亮
    }
    else
    {
      setLEDColor(i, 0, 0); // 熄滅
    }
  }

  // 序列埠除錯輸出
  Serial.print("能量: ");
  Serial.print(energyLevel);
  Serial.print("% -> 點亮 ");
  Serial.print(numLEDsToLight);
  Serial.print(" 顆 LED (顏色: ");
  Serial.print(colorName);
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
  setAllLEDs(PWM_MAX, 0);
  delay(500);

  // 測試黃色（星星色）
  Serial.println("測試黃色（星星色）...");
  setAllLEDs(PWM_MAX, PWM_MAX * 2 / 3);
  delay(500);

  // 測試綠色
  Serial.println("測試綠色...");
  setAllLEDs(0, PWM_MAX);
  delay(500);

  // 逐顆點亮測試（星星閃爍效果）
  Serial.println("逐顆點亮測試（星星閃爍）...");
  clearAllLEDs();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setLEDColor(i, PWM_MAX, PWM_MAX * 2 / 3); // 黃色（星星色）
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
  Serial.println("  - RG LED 模組（紅、綠雙色）× 5");
  Serial.println();

  Serial.println("腳位配置：");
  Serial.print("  - 霍爾感測器: D");
  Serial.print(HALL_SENSOR_PIN);
  Serial.println(" (INT0)");
  Serial.println("  - LED 接線（優化為相鄰腳位）：");
  Serial.println("    LED1: G=D3(PWM), R=D4 ★相鄰");
  Serial.println("    LED2: G=D5(PWM), R=D7");
  Serial.println("    LED3: G=D6(PWM), R=D8");
  Serial.println("    LED4: G=D9(PWM), R=D12");
  Serial.println("    LED5: G=D10(PWM), R=D11 ★相鄰");
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

  // 初始化 LED 腳位
  for (int i = 0; i < NUM_LEDS; i++)
  {
    pinMode(LED_R_PINS[i], OUTPUT);
    pinMode(LED_G_PINS[i], OUTPUT);
  }
  Serial.println("✓ LED 腳位初始化完成");
  Serial.println("  配對方式: (G綠色-PWM, R紅色)");
  Serial.println("  LED1: D3-D4, LED2: D5-D7, LED3: D6-D8, LED4: D9-D12, LED5: D10-D11");

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
