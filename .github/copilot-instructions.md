# Copilot Instructions for PedalStar 光光星

## 專案概述

這是一個基於 Arduino Uno 的互動式兒童遊樂設施控制系統,使用 **PlatformIO** 框架開發。核心功能是透過霍爾感測器偵測腳踏車輪子旋轉,並根據旋轉圈數控制最多 32 顆單色 LED 燈的能量顯示效果,形成進度條視覺回饋。系統使用 2 個 PCA9685 PWM 驅動模組透過 I²C 控制 LED,大幅節省 Arduino 腳位。

## 架構與設計原則

### 系統架構

-   **主控制器**: Arduino Uno (ATmega328P)
-   **感測輸入**: 霍爾感測器模組 A3144 (D2) + 磁鐵
-   **輸出驅動**: PCA9685 PWM 驅動模組 × 2 (I²C 位址 0x40, 0x41)
-   **輸出裝置**: 單色 LED × 32 (數量可調整 1-32)
-   **資料流**: 霍爾中斷 → 計數累積 → 能量計算 → 進度條燈光顯示
-   **通訊協定**: I²C (SDA=A4, SCL=A5)

### 關鍵設計模式

**1. 完整波形偵測 (防止誤計數)**

```cpp
// 使用 CHANGE 模式中斷 + 狀態追蹤
// LOW (磁鐵靠近) → HIGH (磁鐵遠離) = 完成一圈
volatile bool magnetDetected = false;
attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, CHANGE);
```

**2. 非阻塞計時 (主迴圈設計)**

```cpp
// 絕對禁止使用 delay(),改用 millis() 實現多任務
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= TIME_WINDOW) {
    // 處理能量計算與 LED 更新
}
```

**3. 能量衰減機制**

-   閒置 3 秒後開始衰減 (`IDLE_TIMEOUT`)
-   每秒降低 8% 能量 (`DECAY_RATE`)
-   透過 `lastRotationTime` 追蹤最後活動時間

### 腳位配置 (I²C 簡化架構)

```
Arduino Uno 腳位使用:
  D2: 霍爾感測器 (INT0 中斷腳位)
  A4: I²C SDA (資料線,連接兩個 PCA9685)
  A5: I²C SCL (時鐘線,連接兩個 PCA9685)

PCA9685 #1 (位址 0x40):
  通道 0-15: LED #1-16

PCA9685 #2 (位址 0x41,需焊接 A0):
  通道 0-15: LED #17-32
```

**設計理由**:

-   僅使用 3 個 Arduino 腳位 (D2 + A4 + A5)
-   I²C 匯流排架構,模組並聯連接
-   剩餘 11 個數位腳位 (D3-D13) 可用於擴充
-   PCA9685 提供 12-bit PWM 控制 (僅用於開/關)
-   LED 數量可彈性調整 (1-32 顆)

## 開發工作流程

### ⚠️ AI Agent 限制

**禁止直接執行 PlatformIO 指令** - 只能生成/修改程式碼,不能執行:

-   `pio run` (編譯)
-   `pio run --target upload` (上傳)
-   `pio device monitor` (監控)

使用者需手動執行上述指令。

### Windows PowerShell 工作流程

在 Windows 環境下,推薦使用 PowerShell 執行 PlatformIO 指令:

```powershell
# 編譯專案
pio run

# 上傳到 Arduino (自動偵測 COM port)
pio run --target upload

# 監控序列埠輸出 (鮑率 9600)
pio device monitor

# 一鍵編譯並上傳
pio run --target upload && pio device monitor
```

**注意**: `update-instructions.ps1` 目前為空檔案,未來可用於自動化工作流程。

### 程式碼組織

-   **`src/main.cpp`**: 所有功能實作集中於此 (單一檔案架構)
-   **`include/`**: 空目錄 (未來可放標頭檔)
-   **`lib/`**: 空目錄 (未來可模組化為自訂函式庫)
-   **`platformio.ini`**: 依賴 Adafruit PWM Servo Driver Library v3.0.2+

### 除錯流程

1. 程式中已使用 `Serial.begin(9600)` 初始化
2. 關鍵資訊輸出 (旋轉偵測、能量計算、LED 狀態) 已實作
3. 提醒使用者執行 `pio device monitor` 查看即時輸出
4. 啟動時會執行 `ledStartupTest()` 測試硬體連線

## 專案特定慣例

### 調整遊戲難度與 LED 配置

在 `src/main.cpp` 頂部調整以下參數:

```cpp
#define NUM_LEDS 32         // ⚙️ LED 總數量 (可調整: 1-32)
#define MAX_ROTATIONS 20    // ⚙️ 達到 100% 能量所需每秒圈數 (增加=更難)
#define DECAY_RATE 8        // ⚙️ 能量衰減速度 %/秒 (增加=能量消失更快)
#define IDLE_TIMEOUT 3000   // 開始衰減前的等待時間 (毫秒)
```

**LED 數量配置**:

-   **1-16 顆**: 只使用 PCA9685 #1 (位址 0x40)
-   **17-32 顆**: 使用兩個 PCA9685 (位址 0x40 + 0x41)
-   程式會自動判斷是否需要初始化第二個模組

**難度建議**:

-   簡單 (4-6 歲): `MAX_ROTATIONS=15`, `DECAY_RATE=5`
-   適中 (7-9 歲): `MAX_ROTATIONS=20-25`, `DECAY_RATE=6-8`
-   困難 (10 歲+): `MAX_ROTATIONS=35-50`, `DECAY_RATE=10-12`

### 能量與燈光映射

能量百分比決定點亮 LED 數量 (進度條效果):

```cpp
// 能量階差 = 100% ÷ NUM_LEDS
// 範例 (NUM_LEDS = 32):
能量   0% → 0 顆亮
能量  25% → 8 顆亮
能量  50% → 16 顆亮
能量  75% → 24 顆亮
能量 100% → 32 顆亮
```

實作於 `updateLEDsByEnergy(int energyLevel)` 函式,使用線性映射:

```cpp
int numLEDsToLight = map(energyLevel, 0, 100, 0, NUM_LEDS);
```

### 除錯參數

在 `src/main.cpp` 中可切換除錯輸出:

```cpp
#define PRINT_ROTATION true  // 即時輸出每次旋轉偵測 (除錯用)
```

設為 `false` 可減少序列埠輸出,提升效能。

### 中斷服務函式 (ISR) 規則

```cpp
void hallSensorISR() {
    // ✓ 只做狀態更新與簡單變數操作
    // ✗ 禁止 Serial.print (可能導致當機)
    // ✗ 禁止複雜運算或呼叫其他函式
    rotationCount++;
    lastRotationTime = millis();
}
```

### 硬體模組查證流程

**在實作任何硬體相關程式碼前,必須先網路查證:**

1. 搜尋模組規格與接線方式 (例如: "Arduino Uno 霍爾感測器 A3144 接線")
2. 確認 Arduino Uno 腳位功能 (數位/類比/PWM/中斷)
3. 驗證工作電壓與邏輯電平 (3.3V 或 5V)
4. 查詢相關函式庫或範例程式碼

**查證結果必須註解在程式碼中** (參考 `main.cpp` 第 18-82 行的腳位定義註解)。

## 常見開發情境

### 新增功能

1. 在 `src/main.cpp` 中實作
2. 需要模組化時,可在 `lib/` 建立新函式庫
3. 標頭檔放在 `include/` 或函式庫目錄

### 修改硬體設定

-   **COM port**: 修改 `platformio.ini` 的 `upload_port` (Windows: `COM3`, Linux: `/dev/ttyUSB0`)
-   **鮑率**: 修改 `monitor_speed` (預設 9600)

### 新增外部函式庫

目前專案依賴:

```ini
lib_deps =
    adafruit/Adafruit PWM Servo Driver Library@^3.0.2
```

若未來需要其他函式庫 (如 FastLED),優先使用 PlatformIO Registry 或 GitHub URL:

```ini
lib_deps =
    adafruit/Adafruit PWM Servo Driver Library@^3.0.2
    https://github.com/FastLED/FastLED.git
```

## 未來擴充方向

已規劃但尚未實作的功能:

-   [ ] 音效回饋 (需蜂鳴器模組,可使用剩餘腳位)
-   [ ] 最高分數記錄 (需 EEPROM 儲存)
-   [ ] 多人競賽模式 (需多組感測器)
-   [ ] RGB LED 燈條支援 (可串接更多 PCA9685 或使用 WS2812B)
-   [ ] 多種燈光效果模式 (跑馬燈、呼吸燈等)
-   [ ] OLED 顯示器顯示即時數據 (透過 I²C,與 PCA9685 共用匯流排)

**擴充前查證範例**:

-   搜尋「Arduino 蜂鳴器接線」
-   搜尋「Arduino EEPROM.write 範例」
-   搜尋「FastLED GitHub official repository」
