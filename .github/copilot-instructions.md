# Copilot Instructions for PedalStar 光光星# Copilot Instructions for PedalStar 光光星# Copilot Instructions for PedalStar 光光星

## 專案概述

這是一個基於 Arduino Uno 的互動式兒童遊樂設施控制系統，使用 **PlatformIO** 框架開發。核心功能是透過霍爾感測器偵測腳踏車輪子旋轉，並根據旋轉圈數控制 RGB LED 燈的能量顯示效果。## 專案概述## 專案概述

## 架構與設計原則

### 硬體架構這是一個基於 Arduino Uno 的互動式兒童遊樂設施控制系統，使用 **PlatformIO** 框架開發。核心功能是透過霍爾感測器偵測腳踏車輪子旋轉，並根據旋轉圈數控制 RGB LED 燈的能量顯示效果。這是一個基於 Arduino Uno 的互動式兒童遊樂設施控制系統，使用 **PlatformIO** 框架開發。核心功能是透過霍爾感測器偵測腳踏車輪子旋轉，並根據旋轉圈數控制 RGB LED 燈的能量顯示效果。

-   **主控制器**: Arduino Uno (ATmega328P)

-   **感測輸入**: 霍爾感測器 + 磁鐵（每轉一圈觸發一次）

-   **輸出裝置**: RGB LED 燈（使用 PWM 控制）

-   **工作流程**: 感測 → 計數 → 能量累積 → 燈光回饋## 架構與設計原則## 架構與設計原則

### 程式碼組織

-   `src/main.cpp`: 主程式邏輯（目前為範本，需實作霍爾感測器與 LED 控制）

-   `include/`: 自訂標頭檔（目前為空）### 硬體架構### 硬體架構

-   `lib/`: 專案特定函式庫（目前為空）

-   `platformio.ini`: 板子配置（`board = uno`, `framework = arduino`）- **主控制器**: Arduino Uno (ATmega328P)

## 開發工作流程- **感測輸入**: 霍爾感測器 + 磁鐵（每轉一圈觸發一次）- **主控制器**: Arduino Uno (ATmega328P)

### ⚠️ 重要：AI Agent 限制- **輸出裝置**: RGB LED 燈（使用 PWM 控制）- **感測輸入**: 霍爾感測器 + 磁鐵（每轉一圈觸發一次）

-   **禁止直接執行 `pio` 指令**：AI agent 無法使用 PlatformIO CLI 指令

-   編譯、上傳、監控等操作需由使用者手動執行- **工作流程**: 感測 → 計數 → 能量累積 → 燈光回饋- **輸出裝置**: RGB LED 燈（使用 PWM 控制）

-   Agent 只負責生成或修改程式碼，不執行建置流程

-   **工作流程**: 感測 → 計數 → 能量累積 → 燈光回饋

### 編譯與上傳（使用者手動執行）

```bash### 程式碼組織

# 編譯專案

pio run- `src/main.cpp`: 主程式邏輯（目前為範本，需實作霍爾感測器與 LED 控制）### 程式碼組織



# 編譯並上傳到 Arduino Uno- `include/`: 自訂標頭檔（目前為空）

pio run --target upload

- `lib/`: 專案特定函式庫（目前為空）-   `src/main.cpp`: 主程式邏輯（目前為範本，需實作霍爾感測器與 LED 控制）

# 清除編譯產物

pio run --target clean- `platformio.ini`: 板子配置（`board = uno`, `framework = arduino`）-   `include/`: 自訂標頭檔（目前為空）

```

-   `lib/`: 專案特定函式庫（目前為空）

### 除錯與監控（使用者手動執行）

```bash## 開發工作流程-   `platformio.ini`: 板子配置（`board = uno`, `framework = arduino`）

# 開啟序列埠監控（用於 Serial.print 除錯）

pio device monitor

# 或使用 PlatformIO 整合的監控功能### ⚠️ 重要：AI Agent 限制## 開發工作流程

pio run --target upload --target monitor

```- **禁止直接執行 `pio` 指令\*\*：AI agent 無法使用 PlatformIO CLI 指令

## 專案特定慣例- 編譯、上傳、監控等操作需由使用者手動執行### 編譯與上傳

### ⚠️ 硬體實作前置作業- Agent 只負責生成或修改程式碼，不執行建置流程

**在實作任何硬體控制程式碼之前，必須先查證：**

````bash

1. **使用網路搜尋工具**查詢模組規格與接線方式

2. 確認 Arduino Uno 腳位功能（數位/類比/PWM/中斷）### 編譯與上傳（使用者手動執行）# 編譯專案

3. 驗證感測器工作電壓與邏輯電平（3.3V 或 5V）

4. 查詢相關函式庫或範例程式碼```bashpio run

5. 確認腳位衝突（例如 PWM 與中斷共用腳位）

# 編譯專案

**範例查證流程：**

- 搜尋「Arduino Uno 霍爾感測器接線」pio run# 編譯並上傳到 Arduino Uno

- 搜尋「Arduino Uno PWM 腳位」

- 搜尋「Arduino attachInterrupt 可用腳位」pio run --target upload

- 搜尋「RGB LED 共陰極 共陽極 接線」

# 編譯並上傳到 Arduino Uno

**查證後必須在程式碼註解中說明：**

```cpppio run --target upload# 清除編譯產物

// 霍爾感測器型號：A3144（查證：工作電壓 5V，輸出為數位訊號）

// 接線：VCC -> 5V, GND -> GND, OUT -> D2（支援外部中斷 INT0）pio run --target clean

#define HALL_SENSOR_PIN 2

# 清除編譯產物```

// RGB LED：共陰極（查證：需要高電位點亮）

// R -> D9 (PWM), G -> D10 (PWM), B -> D11 (PWM)pio run --target clean

#define LED_RED_PIN 9

#define LED_GREEN_PIN 10```### 除錯與監控

#define LED_BLUE_PIN 11

````

### 腳位配置慣例### 除錯與監控（使用者手動執行）```bash

當實作硬體控制時，建議遵循以下慣例：

-   **霍爾感測器**: 使用數位腳位（建議 D2 或 D3，支援中斷）```bash# 開啟序列埠監控（用於 Serial.print 除錯）

-   **RGB LED**: 使用 PWM 腳位（D3, D5, D6, D9, D10, D11）

-   在 `main.cpp` 開頭使用 `#define` 定義腳位# 開啟序列埠監控（用於 Serial.print 除錯）pio device monitor

-   腳位定義必須包含查證來源的註解

pio device monitor

### 中斷處理模式

霍爾感測器應使用中斷模式偵測轉速，避免 polling 造成延遲：# 或使用 PlatformIO 整合的監控功能

````cpp

volatile unsigned int rotationCount = 0;# 或使用 PlatformIO 整合的監控功能pio run --target upload --target monitor



void hallSensorISR() {pio run --target upload --target monitor```

    rotationCount++;

}```



void setup() {## 專案特定慣例

    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);## 專案特定慣例

}

```### 腳位配置慣例



### 能量計算邏輯### ⚠️ 硬體實作前置作業

- 使用**時間窗口**（例如每 5 秒）計算旋轉圈數

- 能量值 = 圈數累積，並映射到 LED 燈數或亮度級別**在實作任何硬體控制程式碼之前，必須先查證：**當實作硬體控制時，建議遵循以下慣例：

- 使用 `map()` 函數將圈數對應到 LED 亮度（0-255）



### LED 控制模式

- 使用 `analogWrite()` 控制 PWM 輸出實現燈光效果1. **使用網路搜尋工具**查詢模組規格與接線方式-   **霍爾感測器**: 使用數位腳位（建議 D2 或 D3，支援中斷）

- 建議實作漸變效果而非突變，提升視覺體驗

- 考慮使用狀態機管理不同能量等級的燈光模式2. 確認 Arduino Uno 腳位功能（數位/類比/PWM/中斷）-   **RGB LED**: 使用 PWM 腳位（D3, D5, D6, D9, D10, D11）



## 常見開發情境3. 驗證感測器工作電壓與邏輯電平（3.3V 或 5V）-   在 `main.cpp` 開頭使用 `#define` 定義腳位，例如：



### 新增功能時4. 查詢相關函式庫或範例程式碼    ```cpp

1. **先使用網路搜尋工具查證硬體規格與接線方式**

2. 所有硬體相關程式碼寫在 `src/main.cpp`5. 確認腳位衝突（例如 PWM 與中斷共用腳位）    #define HALL_SENSOR_PIN 2

3. 若需模組化，可在 `lib/` 建立自訂函式庫

4. 標頭檔放在 `include/` 或函式庫的對應目錄    #define LED_RED_PIN 9



### 測試硬體時**範例查證流程：**    #define LED_GREEN_PIN 10

1. 使用 `Serial.begin(9600)` 初始化序列埠

2. 用 `Serial.print()` 輸出除錯資訊（旋轉次數、能量值等）- 搜尋「Arduino Uno 霍爾感測器接線」    #define LED_BLUE_PIN 11

3. 提醒使用者執行 `pio device monitor` 查看即時輸出（Agent 無法執行）

- 搜尋「Arduino Uno PWM 腳位」    ```

### 調整硬體設定

- 修改 `platformio.ini` 的 `upload_port` 指定 COM port（Windows）或 `/dev/ttyUSB0`（Linux）- 搜尋「Arduino attachInterrupt 可用腳位」

- 調整 `monitor_speed` 設定序列埠鮑率（預設通常是 9600）

- 搜尋「RGB LED 共陰極 共陽極 接線」### 中斷處理模式

### 新增函式庫依賴

在 `platformio.ini` 中新增函式庫時，**優先使用 GitHub URL** 引用最新版本：



```ini**查證後必須在程式碼註解中說明：**霍爾感測器應使用中斷模式偵測轉速，避免 polling 造成延遲：

[env:uno]

platform = atmelavr```cpp

board = uno

framework = arduino// 霍爾感測器型號：A3144（查證：工作電壓 5V，輸出為數位訊號）```cpp

lib_deps =

    # 使用 GitHub URL 引用最新版本（推薦）// 接線：VCC -> 5V, GND -> GND, OUT -> D2（支援外部中斷 INT0）volatile unsigned int rotationCount = 0;

    https://github.com/FastLED/FastLED.git

    https://github.com/adafruit/Adafruit_NeoPixel.git#define HALL_SENSOR_PIN 2



    # 或使用 PlatformIO 函式庫管理器 IDvoid hallSensorISR() {

    fastled/FastLED@^3.0.0

```// RGB LED：共陰極（查證：需要高電位點亮）    rotationCount++;



**函式庫引用原則：**// R -> D9 (PWM), G -> D10 (PWM), B -> D11 (PWM)}

1. **優先使用 GitHub URL**（`https://github.com/owner/repo.git`）以獲取最新版本

2. 使用前先**網路搜尋確認函式庫的官方 GitHub repository**#define LED_RED_PIN 9

3. 確認函式庫與 Arduino Uno 相容性

4. 如需特定版本，可使用標籤語法：`https://github.com/owner/repo.git#v1.2.3`#define LED_GREEN_PIN 10void setup() {

5. 若 GitHub URL 失效，再改用 PlatformIO Library Registry ID

#define LED_BLUE_PIN 11    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

**範例查證流程：**

- 搜尋「FastLED Arduino library GitHub」```}

- 確認 repository 是否為官方維護

- 檢查 README 中的 Arduino Uno 相容性說明```

- 查看最近更新時間與 issues 回應狀況

### 腳位配置慣例

## 重要提醒

當實作硬體控制時，建議遵循以下慣例：### 能量計算邏輯

- **硬體實作前必須先網路查證**：使用搜尋工具確認模組規格、腳位、接線方式

- **函式庫優先使用 GitHub URL**：確保獲取最新版本，需先網路查證官方 repository- **霍爾感測器**: 使用數位腳位（建議 D2 或 D3，支援中斷）

- **不要使用 `delay()` 阻塞主迴圈**：改用 `millis()` 實作非阻塞計時

- **去彈跳處理**：霍爾感測器可能產生雜訊，需實作去彈跳邏輯- **RGB LED**: 使用 PWM 腳位（D3, D5, D6, D9, D10, D11）-   使用**時間窗口**（例如每 5 秒）計算旋轉圈數

- **中斷函數要簡短**：在 ISR 中只做必要操作，避免複雜運算

- **序列埠除錯**：開發階段保留 Serial 輸出，但上線版本可移除以節省記憶體- 在 `main.cpp` 開頭使用 `#define` 定義腳位-   能量值 = 圈數累積，並映射到 LED 燈數或亮度級別

- **Agent 無法執行 PlatformIO 指令**：只能修改程式碼，不能執行編譯上傳

- 腳位定義必須包含查證來源的註解-   使用 `map()` 函數將圈數對應到 LED 亮度（0-255）

## 未來擴充方向



根據 README.md，規劃的功能包括：

- 音效回饋（需加入蜂鳴器或音效模組）### 中斷處理模式### LED 控制模式

- 最高分數記錄（需 EEPROM 儲存）

- 多人競賽模式（需多組感測器與狀態管理）霍爾感測器應使用中斷模式偵測轉速，避免 polling 造成延遲：

- 更多燈光效果（可使用 NeoPixel 或 WS2812B 燈條）

```cpp-   使用 `analogWrite()` 控制 PWM 輸出實現燈光效果

**擴充功能前的查證範例：**

- 搜尋「Arduino 蜂鳴器接線」volatile unsigned int rotationCount = 0;-   建議實作漸變效果而非突變，提升視覺體驗

- 搜尋「Arduino EEPROM 讀寫範例」

- 搜尋「Arduino WS2812B FastLED 函式庫」-   考慮使用狀態機管理不同能量等級的燈光模式

- 搜尋「FastLED GitHub official repository」

void hallSensorISR() {

    rotationCount++;## 常見開發情境

}

### 新增功能時

void setup() {

    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);1. 所有硬體相關程式碼寫在 `src/main.cpp`

    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);2. 若需模組化，可在 `lib/` 建立自訂函式庫

}3. 標頭檔放在 `include/` 或函式庫的對應目錄

````

### 測試硬體時

### 能量計算邏輯

-   使用**時間窗口**（例如每 5 秒）計算旋轉圈數 1. 使用 `Serial.begin(9600)` 初始化序列埠

-   能量值 = 圈數累積，並映射到 LED 燈數或亮度級別 2. 用 `Serial.print()` 輸出除錯資訊（旋轉次數、能量值等）

-   使用 `map()` 函數將圈數對應到 LED 亮度（0-255）3. 執行 `pio device monitor` 查看即時輸出

### LED 控制模式### 調整硬體設定

-   使用 `analogWrite()` 控制 PWM 輸出實現燈光效果

-   建議實作漸變效果而非突變，提升視覺體驗- 修改 `platformio.ini` 的 `upload_port` 指定 COM port（Windows）或 `/dev/ttyUSB0`（Linux）

-   考慮使用狀態機管理不同能量等級的燈光模式- 調整 `monitor_speed` 設定序列埠鮑率（預設通常是 9600）

## 常見開發情境## 重要提醒

### 新增功能時- **不要使用 `delay()` 阻塞主迴圈**：改用 `millis()` 實作非阻塞計時

1. **先使用網路搜尋工具查證硬體規格與接線方式**- **去彈跳處理**：霍爾感測器可能產生雜訊，需實作去彈跳邏輯

2. 所有硬體相關程式碼寫在 `src/main.cpp`- **中斷函數要簡短**：在 ISR 中只做必要操作，避免複雜運算

3. 若需模組化，可在 `lib/` 建立自訂函式庫- **序列埠除錯**：開發階段保留 Serial 輸出，但上線版本可移除以節省記憶體

4. 標頭檔放在 `include/` 或函式庫的對應目錄

## 未來擴充方向

### 測試硬體時

1. 使用 `Serial.begin(9600)` 初始化序列埠根據 README.md，規劃的功能包括：

2. 用 `Serial.print()` 輸出除錯資訊（旋轉次數、能量值等）

3. 提醒使用者執行 `pio device monitor` 查看即時輸出（Agent 無法執行）- 音效回饋（需加入蜂鳴器或音效模組）

-   最高分數記錄（需 EEPROM 儲存）

### 調整硬體設定- 多人競賽模式（需多組感測器與狀態管理）

-   修改 `platformio.ini` 的 `upload_port` 指定 COM port（Windows）或 `/dev/ttyUSB0`（Linux）- 更多燈光效果（可使用 NeoPixel 或 WS2812B 燈條）

-   調整 `monitor_speed` 設定序列埠鮑率（預設通常是 9600）

## 重要提醒

-   **硬體實作前必須先網路查證**：使用搜尋工具確認模組規格、腳位、接線方式
-   **不要使用 `delay()` 阻塞主迴圈**：改用 `millis()` 實作非阻塞計時
-   **去彈跳處理**：霍爾感測器可能產生雜訊，需實作去彈跳邏輯
-   **中斷函數要簡短**：在 ISR 中只做必要操作，避免複雜運算
-   **序列埠除錯**：開發階段保留 Serial 輸出，但上線版本可移除以節省記憶體
-   **Agent 無法執行 PlatformIO 指令**：只能修改程式碼，不能執行編譯上傳

## 未來擴充方向

根據 README.md，規劃的功能包括：

-   音效回饋（需加入蜂鳴器或音效模組）
-   最高分數記錄（需 EEPROM 儲存）
-   多人競賽模式（需多組感測器與狀態管理）
-   更多燈光效果（可使用 NeoPixel 或 WS2812B 燈條）

**擴充功能前的查證範例：**

-   搜尋「Arduino 蜂鳴器接線」
-   搜尋「Arduino EEPROM 讀寫範例」
-   搜尋「Arduino WS2812B FastLED 函式庫」
