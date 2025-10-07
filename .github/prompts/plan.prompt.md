---
mode: agent
---

# 光光星 PedalStar 開發規格書

## 專案概述

### 專案名稱

光光星 PedalStar - 互動式兒童遊樂設施控制系統

### 專案目標

開發一個基於 Arduino Uno 的互動式控制系統，透過霍爾感測器偵測腳踏車輪子旋轉，計算能量累積，並以 RGB LED 燈提供即時視覺回饋，為兒童創造有趣且具教育意義的遊樂體驗。

### 專案範圍

-   硬體整合：霍爾感測器、RGB LED 燈、Arduino Uno
-   軟體開發：感測器資料處理、能量計算邏輯、燈光控制系統
-   使用者體驗：即時視覺回饋、能量等級顯示

---

## 功能需求

### 核心功能

#### 1. 輪子旋轉偵測系統

**需求描述：** 使用霍爾感測器精確偵測腳踏車輪子旋轉次數

**技術規格：**

-   **霍爾感測器模組**（查證來源：A3144 模組化版本）
-   工作電壓：3.3V - 5V（模組內建穩壓）
-   輸出類型：數位訊號（HIGH/LOW），**模組內建上拉電阻**
-   觸發模式：磁鐵 S 極靠近時輸出 LOW，遠離時輸出 HIGH
-   磁鐵規格：釹磁鐵（Neodymium），直徑 10mm 以上
-   每圈觸發次數：1 次（單磁鐵配置）
-   模組優點：內建上拉電阻、去耦電容、指示燈，接線簡單
-   接腳：VCC（+5V）、GND（接地）、DO（數位輸出）

**實作要求：**

-   使用中斷模式偵測（`attachInterrupt()`）
-   腳位使用：**D2（INT0）優先**，避免與 PWM Timer 衝突
-   中斷觸發模式：`FALLING`（磁鐵靠近時觸發）
-   腳位模式：`INPUT`（模組已內建上拉電阻，無需 INPUT_PULLUP）
-   實作去彈跳處理，避免雜訊誤觸發（建議 50ms）
-   中斷服務函式（ISR）保持簡短，只做計數操作

**驗收標準：**

-   能穩定偵測每次輪子經過
-   誤偵測率 < 1%
-   回應時間 < 10ms

#### 2. 能量累積計算系統

**需求描述：** 根據旋轉圈數計算並累積能量值

**技術規格：**

-   計算週期：5 秒時間窗口（可調整）
-   能量計算公式：能量值 = 時間窗口內旋轉圈數
-   能量等級：分為 5-10 個等級（可配置）
-   能量映射：將圈數線性映射到燈光亮度（0-255）或燈數

**實作要求：**

-   使用 `millis()` 實作非阻塞計時
-   使用 `map()` 函數進行數值映射
-   能量值使用 `unsigned int` 或 `unsigned long` 儲存
-   實作能量衰減機制（選配）

**驗收標準：**

-   計算準確度 100%
-   時間窗口誤差 < 100ms
-   能量值更新即時且流暢

#### 3. RGB LED 燈光控制系統

**需求描述：** 根據能量等級動態控制 5 顆 LED 燈的顏色和亮度

**技術規格：**

-   **PCA9685 PWM 驅動模組**（查證來源：16 通道 I2C PWM 驅動板）
-   通道數：16 通道（使用 15 通道控制 5 顆 RGB LED）
-   通訊介面：I2C（SDA、SCL）
-   I2C 位址：0x40（預設，可透過焊接點調整）
-   PWM 解析度：12 位元（0-4095）
-   PWM 頻率：40Hz - 1600Hz（可設定，預設約 1000Hz）
-   工作電壓：2.3V - 5.5V
-   Arduino 腳位：A4 (SDA)、A5 (SCL)
-   **RGB LED 模組 × 5**（查證來源：共陰極 RGB LED 模組）
-   LED 類型：共陰極（模組上通常標示 "-" 或 "GND"）
-   工作電壓：5V（模組內建限流電阻 220Ω）
-   控制方式：PCA9685 PWM 調光
-   通道分配：
    -   LED1: 通道 0 (R), 1 (G), 2 (B)
    -   LED2: 通道 3 (R), 4 (G), 5 (B)
    -   LED3: 通道 6 (R), 7 (G), 8 (B)
    -   LED4: 通道 9 (R), 10 (G), 11 (B)
    -   LED5: 通道 12 (R), 13 (G), 14 (B)
-   亮度範圍：0-4095（12 位元 PWM）
-   模組優點：內建限流電阻，可直接連接 PCA9685，無需額外電路
-   接腳：R（紅）、G（綠）、B（藍）、GND（共陰極）
-   顏色模式：
    -   低能量：1-2 顆 LED 亮紅色/黃色
    -   中能量：3-4 顆 LED 亮綠色/青色
    -   高能量：5 顆 LED 全亮藍色/紫色/白色

**實作要求：**

-   使用 Adafruit_PWMServoDriver 函式庫控制 PCA9685
-   使用 `pwm.setPWM(channel, 0, value)` 設定 PWM 輸出（value: 0-4095）
-   實作漸變效果（fade in/out）
-   使用狀態機管理不同能量等級的燈光模式
-   支援自訂顏色配置（使用 `#define` 或陣列）
-   實作能量等級對應 LED 數量控制（例如：能量 20% = 1 顆 LED，100% = 5 顆全亮）

**驗收標準：**

-   5 顆 LED 可獨立控制顏色和亮度
-   燈光變化流暢，無閃爍
-   顏色對應能量等級準確
-   漸變效果自然（建議 10-50ms 步進）
-   能量等級與點亮 LED 數量對應正確

---

## 技術規格

### 硬體架構

#### 主控制器

-   **型號：** Arduino Uno (ATmega328P)
-   **工作電壓：** 5V
-   **數位 I/O 腳位：** 14 個（其中 6 個支援 PWM）
-   **類比輸入腳位：** 6 個
-   **Flash 記憶體：** 32 KB（其中 0.5 KB 用於 bootloader）
-   **SRAM：** 2 KB
-   **EEPROM：** 1 KB
-   **時脈速度：** 16 MHz

#### 感測器配置

| 元件             | 型號/規格              | 連接腳位                        | 說明                       |
| ---------------- | ---------------------- | ------------------------------- | -------------------------- |
| 霍爾感測器模組   | A3144 模組（內建電路） | VCC→5V, GND→GND, DO→D2          | 偵測輪子旋轉 (INT0)        |
| 磁鐵             | 釹磁鐵 Ø10mm 以上      | -                               | 固定於輪輻                 |
| PCA9685 PWM 模組 | 16 通道 I2C PWM 驅動   | VCC→5V, GND→GND, SDA→A4, SCL→A5 | I2C PWM 驅動（位址 0x40）  |
| RGB LED 模組 #1  | 共陰極模組（內建電阻） | R→Ch0, G→Ch1, B→Ch2, GND→GND    | LED1（PCA9685 通道 0-2）   |
| RGB LED 模組 #2  | 共陰極模組（內建電阻） | R→Ch3, G→Ch4, B→Ch5, GND→GND    | LED2（PCA9685 通道 3-5）   |
| RGB LED 模組 #3  | 共陰極模組（內建電阻） | R→Ch6, G→Ch7, B→Ch8, GND→GND    | LED3（PCA9685 通道 6-8）   |
| RGB LED 模組 #4  | 共陰極模組（內建電阻） | R→Ch9, G→Ch10, B→Ch11, GND→GND  | LED4（PCA9685 通道 9-11）  |
| RGB LED 模組 #5  | 共陰極模組（內建電阻） | R→Ch12, G→Ch13, B→Ch14, GND→GND | LED5（PCA9685 通道 12-14） |

**模組化優勢：**

-   霍爾感測器模組：內建 10kΩ 上拉電阻、0.1μF 去耦電容、指示燈
-   PCA9685 模組：16 通道硬體 PWM，I2C 介面，僅佔用 2 個 Arduino 腳位
-   RGB LED 模組：內建 220Ω 限流電阻（每色），可直接連接 PCA9685
-   腳位節省：使用 I2C 控制 15 個 PWM 通道，Arduino 主控僅使用 4 個腳位（D2 + A4 + A5 + GND）
-   擴展性強：PCA9685 支援串接，最多可控制 992 個通道（62 片 × 16 通道）

#### 接線圖

```
【Arduino Uno 主控】
  5V  → 霍爾感測器 VCC + PCA9685 VCC
  GND → 霍爾感測器 GND + PCA9685 GND + 所有 RGB LED GND（共地）
  D2  → 霍爾感測器 DO (INT0 中斷腳位)
  A4  → PCA9685 SDA (I2C 數據線)
  A5  → PCA9685 SCL (I2C 時鐘線)

【霍爾感測器模組】（三線接口）
  VCC (或 +)   → Arduino 5V
  GND (或 -)   → Arduino GND
  DO (或 OUT)  → Arduino D2 (INT0)

  備註：模組已內建上拉電阻與去耦電容，直接連接即可
        部分模組有指示燈，磁鐵靠近時會亮起
        磁鐵 S 極靠近時，DO 輸出 LOW；遠離時輸出 HIGH

【PCA9685 PWM 驅動模組】
  VCC → Arduino 5V
  GND → Arduino GND（與所有 LED 共地）
  SDA → Arduino A4 (I2C 數據線)
  SCL → Arduino A5 (I2C 時鐘線)
  V+  → 可選外部電源（若 LED 電流大，建議外接 5V 電源）

  備註：I2C 位址預設為 0x40
        模組右上方焊接點可調整位址（支援多片串接）
        16 個 PWM 通道（Ch0-Ch15），本專案使用 Ch0-Ch14

【RGB LED 模組 × 5】（每顆四線接口）
  LED #1: R → PCA9685 Ch0,  G → Ch1,  B → Ch2,  GND → 共地
  LED #2: R → PCA9685 Ch3,  G → Ch4,  B → Ch5,  GND → 共地
  LED #3: R → PCA9685 Ch6,  G → Ch7,  B → Ch8,  GND → 共地
  LED #4: R → PCA9685 Ch9,  G → Ch10, B → Ch11, GND → 共地
  LED #5: R → PCA9685 Ch12, G → Ch13, B → Ch14, GND → 共地

  備註：模組已內建限流電阻（通常 220Ω），直接連接即可
        共陰極模組需高電位點亮（PWM 數值越大越亮）
        若為共陽極模組，需反轉邏輯（4095-亮度值）
        所有 LED 的 GND 必須接在一起，並連接到 Arduino GND

【磁鐵安裝】
  釹磁鐵固定於腳踏車輪輻，每轉一圈經過霍爾感測器一次
  建議使用熱熔膠或強力膠固定
  確保磁鐵與感測器距離 < 15mm

【接線總結】
  Arduino 使用腳位：D2 (霍爾感測器) + A4 (SDA) + A5 (SCL) + 5V + GND
  PCA9685 使用通道：Ch0-Ch14（15 個通道控制 5 顆 RGB LED）
  總線數：霍爾感測器 3 線 + PCA9685 I2C 4 線 + 每顆 LED 4 線 × 5 = 27 線
```

### 軟體架構

#### 開發環境

-   **IDE：** Visual Studio Code + PlatformIO Extension
-   **平台：** PlatformIO (atmelavr)
-   **框架：** Arduino
-   **開發板：** Arduino Uno
-   **編譯器：** AVR-GCC

#### 程式模組結構

```
src/main.cpp
├─ 全域變數定義
│  ├─ 腳位定義 (#define)
│  ├─ 能量參數配置
│  └─ 燈光效果參數
├─ 中斷服務函式 (ISR)
│  └─ hallSensorISR()
├─ 初始化函式
│  └─ setup()
├─ 主迴圈
│  └─ loop()
└─ 功能函式
   ├─ calculateEnergy()
   ├─ updateLED()
   ├─ fadeLED()
   └─ serialDebug()
```

#### 關鍵演算法

**非阻塞計時（Time Window）：**

```cpp
unsigned long previousMillis = 0;
const unsigned long interval = 5000; // 5秒時間窗口

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // 計算能量並更新 LED
    calculateEnergy();
    updateLED();
  }
}
```

**能量映射公式：**

```cpp
// 將旋轉圈數映射到 LED 亮度
int brightness = map(rotationCount, 0, MAX_ROTATIONS, 0, 255);
brightness = constrain(brightness, 0, 255);
```

**去彈跳處理：**

```cpp
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 50; // 50ms 去彈跳

void hallSensorISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > debounceDelay) {
    rotationCount++;
    lastInterruptTime = currentTime;
  }
}
```

---

## 實作約束

### 硬體限制

1. **記憶體限制：** Arduino Uno 僅有 2KB SRAM，需謹慎使用動態記憶體
2. **腳位限制：** 僅 D2 (INT0), D3 (INT1) 支援外部中斷
3. **I2C 腳位固定：** Arduino Uno 的 I2C 腳位為 **A4 (SDA)** 和 **A5 (SCL)**，不可更改
4. **I2C 位址衝突：** 若使用多個 I2C 裝置，需確認位址不衝突（PCA9685 預設 0x40）
5. **腳位選擇建議：**
    - 霍爾感測器使用 **D2 (INT0)**，避免佔用其他功能腳位
    - PCA9685 使用 **A4 (SDA)** 和 **A5 (SCL)**
    - 保留 D3 (INT1) 供未來擴充（第二組感測器或其他中斷）
    - 保留其他數位腳位供擴充功能（按鈕、蜂鳴器等）
6. **電流限制：**
    - 單一 I/O 腳位最大電流 40mA（推薦 20mA）
    - PCA9685 每通道最大電流 25mA
    - 使用 RGB LED 模組時，模組內建限流電阻，PCA9685 負載較輕
    - 若 LED 總電流 > 500mA，建議 PCA9685 的 V+ 接外部電源
7. **PCA9685 限制：**
    - 所有 16 通道共用相同的 PWM 頻率（預設約 1000Hz）
    - PWM 頻率範圍：40Hz - 1600Hz
    - 不支援個別通道獨立頻率設定
8. **模組化優勢：**
    - 霍爾感測器模組：內建必要電路，簡化接線
    - PCA9685 模組：硬體 PWM，不佔用 CPU 資源，無 Timer 衝突問題
    - RGB LED 模組：內建限流電阻，保護 PCA9685 輸出

### 軟體限制

1. **禁用 `delay()`：** 避免阻塞主迴圈，必須使用 `millis()` 實作非阻塞計時
2. **ISR 規則：** 中斷服務函式必須簡短，避免複雜運算和 I/O 操作
3. **變數宣告：** ISR 中修改的變數必須使用 `volatile` 關鍵字
4. **浮點運算：** 盡量避免浮點數運算以節省記憶體和提升效能

### 開發流程限制

1. **硬體查證：** 實作任何硬體控制前，必須先使用網路搜尋工具查證模組規格
2. **腳位驗證：** 確認 Arduino Uno 腳位功能（數位/類比/PWM/中斷）
3. **函式庫選擇：** 優先使用 GitHub URL 引用函式庫（需先查證官方 repository）
4. **測試方法：** 使用序列埠監控（`Serial.print()`）進行除錯

### PlatformIO 函式庫安裝

**在 `platformio.ini` 中加入以下依賴：**

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
lib_deps =
    # PCA9685 PWM 驅動模組（必要）
    https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git

    # 未來擴充功能（可選）
    # https://github.com/FastLED/FastLED.git
    # https://github.com/adafruit/Adafruit_NeoPixel.git
```

**手動安裝方式（若使用 Arduino IDE）：**

1. 開啟 Arduino IDE
2. 選擇「工具」→「管理程式庫」
3. 搜尋「Adafruit PWM Servo Driver」
4. 點擊「安裝」

**驗證安裝：**

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 若編譯無誤，表示函式庫安裝成功
```

### 專案特定規範

1. **腳位定義：** 使用 `#define` 在程式開頭集中定義，並包含查證來源註解
2. **能量參數：** 時間窗口、能量等級等參數應設計為可調整的常數
3. **燈光效果：** 實作至少 3 種能量等級對應的燈光模式
4. **除錯輸出：** 開發階段保留 Serial 輸出，上線版本可透過條件編譯移除

---

## 開發階段與里程碑

### Phase 1: 硬體查證與基礎設定（估計 1-2 天）

**任務清單：**

-   [ ] 查證霍爾感測器型號與接線方式
-   [ ] 查證 RGB LED 類型（共陰極/共陽極）
-   [ ] 確認 Arduino Uno 腳位分配
-   [ ] 建立腳位定義與硬體參數配置
-   [ ] 完成硬體接線並驗證

**驗收標準：**

-   所有硬體元件規格已查證並記錄於程式註解
-   腳位定義完整且符合硬體功能
-   硬體接線正確，通電無異常

### Phase 2: 感測器偵測實作（估計 2-3 天）

**任務清單：**

-   [ ] 實作霍爾感測器中斷處理
-   [ ] 實作去彈跳邏輯
-   [ ] 實作旋轉計數功能
-   [ ] 序列埠輸出除錯資訊
-   [ ] 實測並驗證偵測準確度

**驗收標準：**

-   中斷觸發穩定，無漏偵測
-   去彈跳有效，誤偵測率 < 1%
-   序列埠可正確顯示旋轉次數

### Phase 3: 能量計算系統（估計 2-3 天）

**任務清單：**

-   [ ] 實作非阻塞計時系統（`millis()`）
-   [ ] 實作時間窗口計數邏輯
-   [ ] 實作能量值計算與映射
-   [ ] 實作能量等級劃分
-   [ ] 序列埠輸出能量值與等級

**驗收標準：**

-   時間窗口準確，誤差 < 100ms
-   能量計算正確，映射合理
-   能量等級劃分清晰

### Phase 4: LED 燈光控制（估計 3-4 天）

**任務清單：**

-   [ ] 實作基礎 PWM 控制
-   [ ] 實作顏色配置（能量等級對應）
-   [ ] 實作漸變效果（fade in/out）
-   [ ] 實作狀態機管理燈光模式
-   [ ] 整合能量系統與燈光系統

**驗收標準：**

-   LED 顏色控制準確
-   漸變效果流暢自然
-   能量等級與燈光對應正確

### Phase 5: 整合測試與優化（估計 2-3 天）

**任務清單：**

-   [ ] 完整功能測試（感測 → 計算 → 燈光）
-   [ ] 效能優化（記憶體使用、回應速度）
-   [ ] 穩定性測試（長時間運行）
-   [ ] 使用者體驗優化
-   [ ] 程式碼整理與註解完善

**驗收標準：**

-   系統穩定運行 > 1 小時無異常
-   記憶體使用 < 70% (< 1.4KB)
-   視覺回饋流暢且吸引人

---

## 成功標準

### 功能驗收標準

#### 必須達成（Must Have）

1. ✅ 霍爾感測器能穩定偵測輪子旋轉，誤差率 < 1%
2. ✅ 能量計算準確，時間窗口誤差 < 100ms
3. ✅ LED 燈能根據能量等級顯示不同顏色/亮度
4. ✅ 系統持續運行 1 小時以上無當機或異常
5. ✅ 兒童踩踏時能看到即時的燈光回饋

#### 應該達成（Should Have）

1. 🎯 燈光漸變效果流暢自然
2. 🎯 能量等級劃分合理（5-10 級）
3. 🎯 序列埠輸出完整除錯資訊
4. 🎯 程式碼模組化且易於維護
5. 🎯 記憶體使用效率高（< 70%）

#### 可以達成（Nice to Have）

1. ⭐ 支援能量衰減機制
2. ⭐ 多種燈光效果模式切換
3. ⭐ 可透過序列埠調整參數
4. ⭐ 能量達到最高等級時的特殊效果
5. ⭐ 低電量警示功能

### 效能指標

-   **感測器回應時間：** < 10ms
-   **能量計算延遲：** < 50ms
-   **LED 更新頻率：** > 20 Hz（避免人眼察覺閃爍）
-   **記憶體使用率：** < 70% SRAM (< 1.4KB)
-   **Flash 使用率：** < 80% (< 25.6KB)

### 使用者體驗標準

1. **即時性：** 踩踏到燈光回饋延遲 < 200ms
2. **流暢度：** 燈光變化無明顯跳動或閃爍
3. **吸引力：** 燈光效果能吸引兒童持續遊玩
4. **清晰度：** 能量等級變化明顯可辨識
5. **安全性：** 無過亮燈光或閃爍效果導致不適

---

## 測試計畫

### 單元測試

#### 1. 霍爾感測器測試

```cpp
// 測試項目：
// - 中斷觸發正常
// - 計數準確度
// - 去彈跳有效性

void testHallSensor() {
  Serial.println("=== Hall Sensor Test ===");
  Serial.println("請旋轉輪子 10 圈...");
  delay(10000);
  Serial.print("偵測到圈數: ");
  Serial.println(rotationCount);
  // 預期結果：10 ± 1
}
```

#### 2. 能量計算測試

```cpp
// 測試項目：
// - 時間窗口準確度
// - 能量映射正確性
// - 能量等級劃分

void testEnergyCalculation() {
  Serial.println("=== Energy Calculation Test ===");
  // 模擬不同圈數，驗證能量計算
  for (int i = 0; i <= 50; i += 5) {
    rotationCount = i;
    int energy = calculateEnergy();
    Serial.print("圈數: ");
    Serial.print(i);
    Serial.print(" -> 能量: ");
    Serial.println(energy);
  }
}
```

#### 3. LED 控制測試

```cpp
// 測試項目：
// - PWM 輸出正常
// - 顏色顯示正確
// - 漸變效果流暢

void testLEDControl() {
  Serial.println("=== LED Control Test ===");
  // 測試各種顏色
  setColor(255, 0, 0);   // 紅
  delay(1000);
  setColor(0, 255, 0);   // 綠
  delay(1000);
  setColor(0, 0, 255);   // 藍
  delay(1000);
}
```

### 整合測試

#### 場景 1：低速踩踏

-   **測試步驟：** 緩慢旋轉輪子，每 5 秒約 5 圈
-   **預期結果：** LED 顯示低能量等級（紅色/黃色）

#### 場景 2：中速踩踏

-   **測試步驟：** 正常速度旋轉，每 5 秒約 15 圈
-   **預期結果：** LED 顯示中能量等級（綠色/青色）

#### 場景 3：高速踩踏

-   **測試步驟：** 快速旋轉，每 5 秒約 30 圈
-   **預期結果：** LED 顯示高能量等級（藍色/紫色/白色）

#### 場景 4：停止踩踏

-   **測試步驟：** 停止旋轉輪子
-   **預期結果：** 能量停止累積，LED 保持當前狀態或逐漸熄滅（若有衰減機制）

### 壓力測試

-   **長時間運行：** 持續運行 2 小時，監控記憶體洩漏和系統穩定性
-   **快速切換：** 快速改變旋轉速度，測試系統回應能力
-   **極限測試：** 測試最大偵測頻率（例如每秒 10 圈以上）

---

## 風險管理

### 潛在風險與應對策略

#### 技術風險

| 風險項目         | 風險等級 | 影響       | 應對策略                                                   |
| ---------------- | -------- | ---------- | ---------------------------------------------------------- |
| 霍爾感測器誤觸發 | 高       | 計數不準確 | 實作去彈跳邏輯、使用中斷模式、增加遮蔽設計                 |
| 記憶體不足       | 中       | 系統當機   | 避免動態記憶體分配、優化變數使用、移除不必要的 Serial 輸出 |
| PWM 頻率干擾     | 低       | LED 閃爍   | 調整 PWM 頻率、使用去耦電容                                |
| 磁鐵鬆脫         | 中       | 感測失效   | 使用強力磁鐵、加固定膠                                     |

#### 硬體風險

| 風險項目 | 風險等級 | 影響     | 應對策略                                 |
| -------- | -------- | -------- | ---------------------------------------- |
| 接線錯誤 | 高       | 元件損壞 | 實作前查證接線圖、使用限流電阻、多次檢查 |
| 電源不穩 | 中       | 系統重啟 | 使用穩壓電源、加濾波電容                 |
| LED 過熱 | 低       | 壽命縮短 | 使用限流電阻、避免長時間最大亮度         |

#### 專案風險

| 風險項目     | 風險等級 | 影響         | 應對策略                         |
| ------------ | -------- | ------------ | -------------------------------- |
| 開發時程延遲 | 中       | 無法如期完成 | 分階段開發、優先實作核心功能     |
| 需求變更     | 低       | 重新開發     | 模組化設計、預留擴充彈性         |
| 測試環境不足 | 中       | 無法驗證功能 | 使用序列埠模擬、建立簡易測試裝置 |

---

## 維護與擴充計畫

### 維護計畫

#### 例行維護

1. **每週檢查：** 檢查接線是否鬆動、磁鐵是否固定
2. **每月檢查：** 檢查 LED 是否正常、感測器是否靈敏
3. **每季檢查：** 清潔感測器、檢查電源供應

#### 程式碼維護

1. **版本控制：** 使用 Git 管理程式碼版本
2. **註解更新：** 修改程式時同步更新註解
3. **參數調整：** 根據使用回饋調整能量參數和燈光效果

### 未來擴充功能

#### Phase 2 功能（優先度：高）

1. **音效回饋系統**

    - 硬體：無源蜂鳴器（需查證接線與頻率）
    - 功能：不同能量等級播放不同音效
    - 腳位：D8（非 PWM 腳位，使用 `tone()` 函數）

2. **最高分數記錄**
    - 硬體：無需額外硬體
    - 功能：使用 EEPROM 儲存歷史最高圈數
    - 函式：`EEPROM.read()`, `EEPROM.write()`

#### Phase 3 功能（優先度：中）

3. **多人競賽模式**

    - 硬體：多組感測器、按鈕（模式切換）
    - 功能：同時偵測多台腳踏車，比較能量值
    - 挑戰：腳位數量限制、記憶體管理

4. **進階燈光效果**
    - 硬體：WS2812B LED 燈條（需查證 FastLED 函式庫）
    - 功能：彩虹效果、追逐效果、呼吸燈效果
    - 函式庫：FastLED（使用 GitHub URL 引用）

#### Phase 4 功能（優先度：低）

5. **顯示器整合**

    - 硬體：OLED 顯示器（I2C 介面）
    - 功能：顯示即時圈數、能量值、排行榜
    - 函式庫：Adafruit_SSD1306

6. **無線通訊**
    - 硬體：nRF24L01 無線模組
    - 功能：多台裝置資料同步、遠端監控
    - 挑戰：記憶體限制、通訊協定設計

### 擴充實作指引

**實作前準備：**

1. 使用網路搜尋工具查證新硬體規格
2. 確認 Arduino Uno 腳位是否足夠
3. 評估記憶體使用情況（SRAM 和 Flash）
4. 查證相關函式庫的官方 GitHub repository
5. 建立測試計畫

**實作流程：**

1. 更新硬體接線圖
2. 更新腳位定義與參數配置
3. 實作新功能模組
4. 單元測試
5. 整合測試
6. 文件更新

---

## 參考資源

### 官方文件

-   [Arduino 官方文件](https://www.arduino.cc/reference/en/)
-   [PlatformIO 文件](https://docs.platformio.org/)
-   [ATmega328P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)

### 硬體查證資源

-   Arduino Uno 腳位圖
-   霍爾感測器規格書（待查證）
-   RGB LED 規格書（待查證）

### 相關函式庫

-   Arduino 內建函式（無需額外安裝）
-   **Adafruit_PWMServoDriver**（必要，控制 PCA9685）: `https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git`
-   Wire（I2C 通訊，Arduino 內建）
-   FastLED（未來擴充使用，WS2812B 燈條）: `https://github.com/FastLED/FastLED.git`
-   Adafruit_NeoPixel（未來擴充使用，WS2812B 燈條）: `https://github.com/adafruit/Adafruit_NeoPixel.git`

### 開發工具

-   Visual Studio Code
-   PlatformIO Extension
-   Serial Monitor
-   Git 版本控制

---

## 附錄

### A. 程式碼範本

#### 腳位定義範本

```cpp
// ===== 腳位定義 =====
// 霍爾感測器模組（查證來源：A3144 模組，內建上拉電阻與去耦電容）
// 接線：VCC->5V, GND->GND, DO->D2
// 模組優點：內建電路，接線簡單，穩定性高
#define HALL_SENSOR_PIN 2  // D2 = INT0 中斷腳位

// PCA9685 PWM 驅動模組（查證來源：16 通道 I2C PWM 驅動板）
// 接線：VCC->5V, GND->GND, SDA->A4, SCL->A5
// I2C 位址：0x40（預設）
// Arduino Uno I2C 腳位：A4 (SDA), A5 (SCL) - 硬體固定，不可更改
#define PCA9685_ADDRESS 0x40   // I2C 位址
#define PCA9685_FREQ 1000      // PWM 頻率 1000Hz

// RGB LED 通道配置（PCA9685 的 16 個通道：Ch0-Ch15）
// 每顆 RGB LED 使用 3 個通道（R, G, B）
#define LED1_R 0    // LED #1 紅色 → PCA9685 通道 0
#define LED1_G 1    // LED #1 綠色 → PCA9685 通道 1
#define LED1_B 2    // LED #1 藍色 → PCA9685 通道 2

#define LED2_R 3    // LED #2 紅色 → PCA9685 通道 3
#define LED2_G 4    // LED #2 綠色 → PCA9685 通道 4
#define LED2_B 5    // LED #2 藍色 → PCA9685 通道 5

#define LED3_R 6    // LED #3 紅色 → PCA9685 通道 6
#define LED3_G 7    // LED #3 綠色 → PCA9685 通道 7
#define LED3_B 8    // LED #3 藍色 → PCA9685 通道 8

#define LED4_R 9    // LED #4 紅色 → PCA9685 通道 9
#define LED4_G 10   // LED #4 綠色 → PCA9685 通道 10
#define LED4_B 11   // LED #4 藍色 → PCA9685 通道 11

#define LED5_R 12   // LED #5 紅色 → PCA9685 通道 12
#define LED5_G 13   // LED #5 綠色 → PCA9685 通道 13
#define LED5_B 14   // LED #5 藍色 → PCA9685 通道 14

#define NUM_LEDS 5  // LED 總數

// ===== 參數配置 =====
#define TIME_WINDOW 5000        // 時間窗口：5秒
#define MAX_ROTATIONS 50        // 最大圈數（用於映射）
#define DEBOUNCE_DELAY 50       // 去彈跳延遲：50ms
#define ENERGY_LEVELS 5         // 能量等級數量
#define PWM_MAX 4095            // PCA9685 PWM 最大值（12 位元）
```

#### 中斷處理範本

```cpp
// ===== 全域變數 =====
volatile unsigned int rotationCount = 0;
volatile unsigned long lastInterruptTime = 0;

// ===== 中斷服務函式 =====
// A3144 開極輸出：磁鐵靠近時輸出 LOW，遠離時輸出 HIGH（透過上拉電阻）
void hallSensorISR() {
  unsigned long currentTime = millis();
  // 去彈跳處理：50ms 內的重複觸發視為雜訊
  if (currentTime - lastInterruptTime > DEBOUNCE_DELAY) {
    rotationCount++;
    lastInterruptTime = currentTime;
  }
}

// ===== 初始化 =====
void setup() {
  // 初始化序列埠（除錯用）
  Serial.begin(9600);
  Serial.println("光光星系統啟動...");

  // 設定霍爾感測器腳位為輸入（模組已內建上拉電阻，使用 INPUT 即可）
  // 注意：不需要 INPUT_PULLUP，因為模組已經有上拉電阻了
  pinMode(HALL_SENSOR_PIN, INPUT);

  // 啟用外部中斷 INT0（D2），磁鐵靠近時觸發（FALLING）
  // 模組特性：磁鐵 S 極靠近時輸出 LOW，遠離時輸出 HIGH
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN),
                  hallSensorISR, FALLING);

  // 初始化 PCA9685 模組
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // 內部振盪器頻率（通常為 27MHz）
  pwm.setPWMFreq(PCA9685_FREQ);          // 設定 PWM 頻率（1000Hz）

  Serial.println("PCA9685 初始化完成");
  Serial.print("PWM 頻率: ");
  Serial.print(PCA9685_FREQ);
  Serial.println(" Hz");

  // 關閉所有 LED（初始狀態）
  for (int i = 0; i < 15; i++) {
    pwm.setPWM(i, 0, 0);  // 通道 0-14 全部設為 0（熄滅）
  }

  Serial.println("系統就緒！");
}
```

#### LED 控制函式範本

```cpp
// ===== 全域變數（PCA9685 物件）=====
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// ===== LED 控制函式 =====
// 設定單顆 LED 的 RGB 顏色
// ledNum: LED 編號 (0-4 對應 LED1-LED5)
// r, g, b: 顏色值 (0-4095)
void setLEDColor(int ledNum, uint16_t r, uint16_t g, uint16_t b) {
  int baseChannel = ledNum * 3;  // 計算起始通道（每顆 LED 佔 3 通道）
  pwm.setPWM(baseChannel + 0, 0, r);  // 紅色通道
  pwm.setPWM(baseChannel + 1, 0, g);  // 綠色通道
  pwm.setPWM(baseChannel + 2, 0, b);  // 藍色通道
}

// 設定所有 LED 為相同顏色
void setAllLEDs(uint16_t r, uint16_t g, uint16_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    setLEDColor(i, r, g, b);
  }
}

// 關閉所有 LED
void clearAllLEDs() {
  for (int i = 0; i < 15; i++) {  // 通道 0-14
    pwm.setPWM(i, 0, 0);
  }
}

// 根據能量等級點亮對應數量的 LED
// energyLevel: 0-100 (百分比)
void updateLEDsByEnergy(int energyLevel) {
  // 將能量等級映射到 LED 數量 (0-5)
  int numLEDsToLight = map(energyLevel, 0, 100, 0, NUM_LEDS);
  numLEDsToLight = constrain(numLEDsToLight, 0, NUM_LEDS);

  // 根據能量等級選擇顏色
  uint16_t r, g, b;
  if (energyLevel < 30) {
    // 低能量：紅色
    r = 4095; g = 0; b = 0;
  } else if (energyLevel < 60) {
    // 中能量：黃色
    r = 4095; g = 2048; b = 0;
  } else if (energyLevel < 80) {
    // 中高能量：綠色
    r = 0; g = 4095; b = 0;
  } else {
    // 高能量：藍色
    r = 0; g = 0; b = 4095;
  }

  // 點亮對應數量的 LED
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLEDsToLight) {
      setLEDColor(i, r, g, b);  // 點亮
    } else {
      setLEDColor(i, 0, 0, 0);  // 熄滅
    }
  }

  Serial.print("能量: ");
  Serial.print(energyLevel);
  Serial.print("% -> 點亮 ");
  Serial.print(numLEDsToLight);
  Serial.println(" 顆 LED");
}

// LED 漸變效果範例
// 從當前亮度漸變到目標亮度
void fadeLED(int ledNum, uint16_t fromR, uint16_t fromG, uint16_t fromB,
             uint16_t toR, uint16_t toG, uint16_t toB, int steps, int delayMs) {
  for (int i = 0; i <= steps; i++) {
    uint16_t r = map(i, 0, steps, fromR, toR);
    uint16_t g = map(i, 0, steps, fromG, toG);
    uint16_t b = map(i, 0, steps, fromB, toB);
    setLEDColor(ledNum, r, g, b);
    delay(delayMs);
  }
}
```

### B. 除錯檢查清單

#### 硬體檢查

-   [ ] 電源電壓正確（5V）
-   [ ] 所有接線牢固，確認共地（GND）連接正確
-   [ ] **霍爾感測器模組接線**：VCC->5V, GND->GND, DO->D2
-   [ ] **磁鐵固定於輪子**，確認極性（S 極靠近感測器時模組指示燈亮）
-   [ ] **PCA9685 模組接線**：VCC->5V, GND->GND, SDA->A4, SCL->A5
-   [ ] **I2C 位址確認**：PCA9685 預設 0x40（可用 I2C 掃描程式檢查）
-   [ ] **5 顆 RGB LED 模組接線**：每顆 R/G/B 接對應 PCA9685 通道，GND 共同接地
-   [ ] **LED 為共陰極**（確認 GND 腳接地，高電位點亮）
-   [ ] 確認霍爾感測器使用 D2 (INT0) 中斷腳位

#### 軟體檢查

-   [ ] 腳位定義正確（D2, A4, A5, PCA9685 通道）
-   [ ] 中斷模式設定正確（FALLING）
-   [ ] 去彈跳延遲適當（50ms）
-   [ ] **PWM 輸出範圍正確**（0-4095，12 位元）
-   [ ] **Adafruit_PWMServoDriver 函式庫已安裝**
-   [ ] **I2C 位址與硬體一致**（0x40）
-   [ ] 序列埠齑率一致（9600）

#### 功能檢查

-   [ ] 霍爾感測器能偵測旋轉
-   [ ] 計數準確無誤
-   [ ] 能量計算正確
-   [ ] **PCA9685 I2C 通訊正常**（可用序列埠確認）
-   [ ] **5 顆 LED 可獨立控制顏色**
-   [ ] **LED 數量與能量等級對應**（低能量 1-2 顆，高能量 5 顆全亮）
-   [ ] 燈光漸變效果流暢
-   [ ] 系統穩定運行 > 1 小時

### C. 常見問題與解決方案

**Q1: 霍爾感測器無反應？**

-   **檢查模組接線**：確認 VCC->5V, GND->GND, DO->D2 接線正確
-   確認磁鐵極性：**S 極靠近感測器**時模組指示燈應該會亮
-   確認磁鐵距離：磁鐵與感測器距離 < 15mm
-   測試感測器輸出：無磁鐵時應為 HIGH（5V），有磁鐵時為 LOW（0V）
-   檢查中斷腳位：確認使用 D2 (INT0)

**Q2: PCA9685 無法控制 LED？**

-   **檢查 I2C 連接**：確認 SDA->A4, SCL->A5, VCC->5V, GND->GND
-   **掃描 I2C 位址**：使用 I2C Scanner 程式確認 PCA9685 位址為 0x40
-   **檢查函式庫安裝**：確認 Adafruit_PWMServoDriver 已正確安裝
-   **測試 PWM 輸出**：使用 `pwm.setPWM(0, 0, 4095)` 測試通道 0 全亮
-   檢查電源：若 LED 總電流 > 500mA，PCA9685 V+ 應外接電源

**Q3: LED 不亮或顏色錯誤？**

-   **確認共陰極/共陽極類型**：共陰極需高電位點亮（PWM 值 0-4095）
-   **檢查模組接線**：R/G/B 接對應 PCA9685 通道，GND 共同接地
-   **檢查通道分配**：LED1 使用 Ch0-2，LED2 使用 Ch3-5，以此類推
-   測試單一顏色：單獨測試紅/綠/藍通道
-   若 LED 不亮，檢查模組是否有損壞

**Q6: 為什麼要使用 PCA9685 而不是 Arduino 內建 PWM？**

-   Arduino Uno 只有 6 個 PWM 腳位，無法控制 5 顆 RGB LED（15 通道）
-   PCA9685 提供 16 通道硬體 PWM，不佔用 CPU 資源
-   I2C 介面僅使用 2 個腳位 (A4, A5)，節省腳位
-   12 位元解析度 (0-4095) 比 Arduino 8 位元 (0-255) 更精細
-   支援多片串接，擴展性強

**Q7: 為什麼要用 D2 而不是 D3？**

-   D2 (INT0) 和 D3 (INT1) 都支援中斷
-   使用 D2 可保留 D3 供未來擴充（第二組感測器或其他中斷功能）
-   使用 PCA9685 後無 Timer 衝突問題

**Q3: 計數不準確？**

-   調整去彈跳延遲時間
-   檢查磁鐵是否鬆動
-   確認中斷觸發模式（RISING/FALLING/CHANGE）

**Q4: 系統當機或重啟？**

-   檢查記憶體使用情況
-   移除不必要的 Serial 輸出
-   確認電源供應穩定

**Q5: 燈光閃爍？**

-   增加去耦電容（0.1uF）
-   調整 PWM 更新頻率
-   檢查電源線是否過細

---

## 文件版本控制

| 版本 | 日期       | 修改內容 | 作者           |
| ---- | ---------- | -------- | -------------- |
| 1.0  | 2025-10-07 | 初版建立 | GitHub Copilot |

---

**文件狀態：** 📝 草稿 → 審核 → ✅ 核准  
**最後更新：** 2025 年 10 月 7 日
