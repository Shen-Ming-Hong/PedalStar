# Copilot Instructions for PedalStar 光光星

## 專案概述

這是一個基於 Arduino Uno 的互動式兒童遊樂設施控制系統,使用 **PlatformIO** 框架開發。核心功能是透過霍爾感測器偵測腳踏車輪子旋轉,並根據旋轉圈數控制 5 顆雙色 LED (RG) 燈的能量顯示效果,模擬星星從紅色到綠色的發光變化。

## 架構與設計原則

### 系統架構

-   **主控制器**: Arduino Uno (ATmega328P)
-   **感測輸入**: 霍爾感測器模組 A3144 (D2) + 磁鐵
-   **輸出裝置**: 5 顆雙色 LED (紅、綠,共陰極)
-   **資料流**: 霍爾中斷 → 計數累積 → 能量計算 → 燈光顏色漸變 (紅 → 橙 → 黃 → 黃綠 → 綠)

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

### 腳位配置 (相鄰腳位優化)

```
霍爾感測器: D2 (INT0 中斷腳位)
LED 配置 (G-R 盡量相鄰以簡化接線):
  LED1: G=D3(PWM), R=D4 ★相鄰
  LED2: G=D5(PWM), R=D7
  LED3: G=D6(PWM), R=D8
  LED4: G=D9(PWM), R=D12
  LED5: G=D10(PWM), R=D11 ★相鄰
```

**設計理由**:

-   綠色通道全部使用 PWM 腳位 → 支援平滑漸變
-   紅色通道使用數位腳位 → 僅需開/關控制
-   避免 D13 (板載 LED 干擾除錯)

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
-   **`platformio.ini`**: 無外部依賴,使用 Arduino 內建功能

### 除錯流程

1. 程式中已使用 `Serial.begin(9600)` 初始化
2. 關鍵資訊輸出 (旋轉偵測、能量計算、LED 狀態) 已實作
3. 提醒使用者執行 `pio device monitor` 查看即時輸出
4. 啟動時會執行 `ledStartupTest()` 測試硬體連線

## 專案特定慣例

### 調整遊戲難度

在 `src/main.cpp` 頂部調整以下參數:

```cpp
#define MAX_ROTATIONS 20    // ⚙️ 達到 100% 能量所需每秒圈數 (增加=更難)
#define DECAY_RATE 8        // ⚙️ 能量衰減速度 %/秒 (增加=能量消失更快)
#define IDLE_TIMEOUT 3000   // 開始衰減前的等待時間 (毫秒)
```

**難度建議**:

-   簡單 (4-6 歲): `MAX_ROTATIONS=15`, `DECAY_RATE=5`
-   適中 (7-9 歲): `MAX_ROTATIONS=20-25`, `DECAY_RATE=6-8`
-   困難 (10 歲+): `MAX_ROTATIONS=35-50`, `DECAY_RATE=10-12`

### 能量與燈光映射

能量百分比決定點亮 LED 數量與顏色:

| 能量範圍 | LED 數量 | 顏色 (R,G) | 視覺效果         |
| -------- | -------- | ---------- | ---------------- |
| 0-20%    | 1 顆     | (255,0)    | 🔴 紅色 (冷星)   |
| 20-40%   | 2 顆     | (255,85)   | 🟠 橙色          |
| 40-60%   | 3 顆     | (255,170)  | 🟡 黃色 (太陽色) |
| 60-80%   | 4 顆     | (127,255)  | 🟢 黃綠色        |
| 80-100%  | 5 顆     | (0,255)    | 💚 綠色 (熱星)   |

實作於 `updateLEDsByEnergy(int energyLevel)` 函式。

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

目前專案不依賴外部函式庫。若未來需要 (如 FastLED),優先使用 GitHub URL:

```ini
lib_deps =
    https://github.com/FastLED/FastLED.git
```

## 未來擴充方向

已規劃但尚未實作的功能:

-   [ ] 音效回饋 (需蜂鳴器模組)
-   [ ] 最高分數記錄 (需 EEPROM 儲存)
-   [ ] 多人競賽模式 (需多組感測器)
-   [ ] WS2812B 燈條支援 (需 FastLED 函式庫)

**擴充前查證範例**:

-   搜尋「Arduino 蜂鳴器接線」
-   搜尋「Arduino EEPROM.write 範例」
-   搜尋「FastLED GitHub official repository」
