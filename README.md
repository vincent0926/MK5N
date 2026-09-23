# FRC 9427 Robot Project (MYMK5N)

本專案是 **FRC Team 9427** 的機器人控制程式，基於 WPILib 的 **Command-based** 架構開發，採用 Java 語言編寫，並專為 2026 賽季進行配置。

## 🤖 專案概述 (Project Overview)

本專案控制一台配備 **Swerve Drive (四輪獨立轉向底盤)** 以及多個輔助機構的 FRC 機器人，包括：
- 齒條伸縮進氣機構 (Intake)
- 雙滾輪進氣機構 (Roller Intake)
- 雙排軌道傳輸機構 (Orbit)
- 輸彈機構 (Indexer)
- 角度可調發射罩 (Hood)
- 四馬達飛輪發射器 (Shooter)
- **三顆 Limelight 視覺系統** (前方 + 左方 + 右方)，支援 MegaTag 2 全場定位與 AprilTag 自動瞄準

### ⚙️ 開發環境與硬體諸元

| 項目 | 規格 |
| :--- | :--- |
| WPILib 版本 | `2026.2.1` |
| Java 版本 | JDK 17 |
| 馬達控制器 | CTRE TalonFX (Kraken X60 / Falcon 500) |
| 感測器 | CTRE CANcoder (轉向角度感測), Pigeon 2.0 (陀螺儀/IMU) |
| 視覺系統 | Limelight × 3 (`limelight-up`, `limelight-left`, `limelight-right`) |
| 通訊協定 | CAN Bus (CTRE Phoenix 6 API `26.3.0`) |
| 路徑規劃 | PathPlannerLib |

---

## 🏗️ 系統架構 (Subsystems)

本程式採用模組化設計，將機器人的硬體劃分為以下子系統 (Subsystems)：

### 1. 🛞 底盤子系統 (`DriveSubsystem.java`)
* **類型**: Swerve Drive (四輪獨立轉向與驅動)
* **模組配置**: 每個 Swerve 模組包含一個驅動馬達 (TalonFX)、一個轉向馬達 (TalonFX) 與一個用於絕對角度定位的 CANcoder。
* **定位系統**: 採用 Pigeon 2 陀螺儀進行場地導向 (Field-Relative) 控制，並使用 `SwerveDrivePoseEstimator` 進行姿態估測（結合里程計 + 視覺）。
* **路徑規劃**: 整合 PathPlannerLib `AutoBuilder`，支援自動階段路徑追蹤。
* **場地視覺化**: 透過 `Field2d` 將機器人即時位置發佈至 SmartDashboard / Elastic。
* **特色**:
  * **防抖動設計 (Anti-Jitter/Deadband)**: 當搖桿輸入極小時，自動停止驅動並鎖定角度，防止馬達微幅抖動。
  * **狀態優化 (Swerve Optimize)**: 利用 WPILib 演算法優化目標轉角，避免模組旋轉超過 90 度。
  * **X 陣型鎖定**: 支援將四輪鎖定為 X 型防撞陣型。

### 2. 🏹 發射器子系統 (`ShooterSubsystem.java`)
* **馬達配置**: 4 顆 TalonFX 馬達，分別為：
  * 左上 (LT, ID 9) - 主控馬達
  * 左下 (LD, ID 10) - 跟隨 LT (同向 Aligned)
  * 右上 (RT, ID 11) - 跟隨 LT (反向 Opposed)
  * 右下 (RD, ID 12) - 跟隨 LT (反向 Opposed)
* **控制模式**: 速度電壓閉迴路控制 (`VelocityVoltage`)，預設速度 20.0 RPS，支援透過 `setRPS()` 動態調整。
* **停止模式**: 使用 `NeutralOut` 讓飛輪自然滑行停止 (Coast)。

### 3. 🎯 調整罩子系統 (`HoodSubsystem.java`)
* **馬達配置**: 1 顆 TalonFX 馬達 (ID 15)。
* **控制模式**: 利用 `MotionMagicVoltage` (運動魔法) 控制角度，設定平滑的加速度 (1.0 RPS/s) 和巡航速度 (0.5 RPS)。
* **特色**:
  * 角度安全限制：`0.0°` ~ `45.0°` (直驅無齒輪箱)
  * 軟體極限開關防護
  * 提供 `isAtAngle()` 方法回報是否到位（容許誤差 1.0°）

### 4. 📦 齒條進氣子系統 (`IntakeSubsystem.java`)
* **類型**: 齒條伸縮機構 (Rack and Pinion)
* **馬達配置**: 1 顆 TalonFX 馬達 (ID 30)，配置為剎車模式。
* **控制模式**: 位置電壓控制 (`PositionVoltage`)。將旋轉圈數換算為公尺，精準控制進氣口伸出 (0.3m) 與收回 (0.0m) 狀態。
* **齒比換算**: 齒輪比 12:1，節圓直徑 3 英吋。

### 5. 🌀 滾輪進氣子系統 (`RollerIntakeSubsystem.java`)
* **馬達配置**: 2 顆 TalonFX 馬達 (前方 ID 45, 後方 ID 41)，後方為前方的反向從動 (Opposed Follower)。
* **控制模式**: 電壓輸出控制 (`VoltageOut`)，預設電壓 5.5V。
* **特色**: 支援正轉吸球 (`runRollers`) 與反轉吐球 (`reverseRollers`)。

### 6. 🔄 軌道傳輸子系統 (`OrbitSubsystem.java`)
* **馬達配置**: 2 顆 TalonFX 馬達 (上方 ID 14, 下方 ID 13)，上方跟隨下方 (Aligned Follower)。
* **控制模式**: 電壓輸出控制 (`VoltageOut`)，預設電壓 10.0V。
* **特色**: 支援正轉 (`runorbit`) 與反轉 (`norunorbit`)。

### 7. 📤 輸彈子系統 (`IndexerSubsystem.java`)
* **馬達配置**: 1 顆 TalonFX 馬達 (ID 31)。
* **控制模式**: 電壓輸出控制 (`VoltageOut`)，預設電壓 10.0V。
* **特色**: 控制進彈滾輪，將遊戲對象送入發射器。

### 8. 👁️ 視覺子系統 (`VisionSubsystem.java`)
* **硬體**: 三顆 Limelight 攝影機 (`limelight-up`, `limelight-left`, `limelight-right`)
* **功能**:
  * **MegaTag 2 全場定位**: 同步陀螺儀角度至各 Limelight，融合多視角姿態估測回 `DriveSubsystem` 的 `SwerveDrivePoseEstimator`。
  * **AprilTag 距離計算**: 使用場地佈局 (`AprilTagFieldLayout`) 與 `botpose_wpiblue` 計算機器人與目標 AprilTag 的平面距離。
  * **Hub 距離偵測**: 自動依聯盟顏色 (紅方 ID 9/10, 藍方 ID 25/26) 鎖定對應 AprilTag。
  * **多鏡頭最佳目標**: 從三個鏡頭中選取最近的有效目標距離與水平偏差 (tx)。

---

## 🤖 指令系統 (Commands)

### 自動瞄準與射擊 (`AutoAimAndShoot.java`)

全自動瞄準發射指令，整合視覺系統、底盤、Hood、飛輪、Indexer 與 Orbit：

1. **視覺偵測**: 從三顆 Limelight 取得最佳目標距離與水平偏差
2. **內插查表**: 根據距離自動查表取得目標仰角與飛輪轉速 (RPS)
3. **仰角鎖定**: 首次看到目標時立即鎖定 Hood 仰角（快門式），直到重新按鈕才更新
4. **底盤自動對準**: 使用 P 控制器 (Kp=0.08) 旋轉底盤對準目標，限制最大旋轉速度 ±3.0 rad/s
5. **發射判定**: 當仰角到位 + 飛輪轉速到位 + 底盤對準（tx < 1.0°）時，自動啟動 Indexer 與 Orbit 送球
6. **安全機制**: 搖桿/扳機輸入 > 0.1 時自動中斷指令，歸還底盤控制權

**距離 → 仰角查表 (kDistanceToAngleMap):**

| 距離 (m) | 仰角 (°) |
| :---: | :---: |
| 1.0 | 2.0 |
| 2.0 | 7.0 |
| 3.0 | 10.0 |
| 4.0 | 20.0 |

**距離 → 飛輪轉速查表 (kDistanceToRPSMap):**

| 距離 (m) | 轉速 (RPS) |
| :---: | :---: |
| 1.0 | 18.0 |
| 2.0 | 20.0 |
| 3.0 | 30.0 |
| 4.0 | 40.0 |

### PathPlanner 自動指令 (Named Commands)

| 指令名稱 | 功能 |
| :--- | :--- |
| `ShootAndIndex` | 依序啟動飛輪 → 等 0.5s → Orbit → 等 0.5s → Indexer → 等 0.5s → 全部停止 |
| `IntakeExtend` | 伸出進氣口 |
| `IntakeRetract` | 收回進氣口 |
| `StartRoller` | 啟動滾輪吸球 |
| `StopRoller` | 停止滾輪 |

---

## 🎮 控制器按鍵配置 (Controller Bindings)

所有的按鍵綁定均在 `RobotContainer.java` 中設定。目前配置使用單個 Xbox 搖桿 (Port 0) 控制：

| 按鍵 (Button) | 觸發方式 | 控制動作 (Action) | 機構反應 (Mechanism) |
| :--- | :--- | :--- | :--- |
| **左搖桿 (Left Stick)** | 持續 | 機器人平移 (X/Y Axis) | 底盤移動 (Field-Relative) |
| **左/右扳機 (L/R Triggers)** | 持續 | 機器人旋轉 | 左扳機左轉 / 右扳機右轉 (差值控制，平方曲線) |
| **A 鍵** | Toggle | 全自動瞄準與射擊 | 啟動 `AutoAimAndShoot`（視覺 + 底盤 + Hood + 飛輪 + 送球）；推動搖桿/扳機可中斷 |
| **B 鍵** | Toggle | 啟動/停止飛輪 | 單純運轉 `ShooterSubsystem` (預設 20.0 RPS) |
| **X 鍵** | Toggle | 啟動/停止輸彈與軌道 | 同時控制 `IndexerSubsystem` + `OrbitSubsystem` |
| **Y 鍵** | Toggle | 啟動/停止滾輪進件 | 切換 `RollerIntakeSubsystem` 滾輪轉動 (吸球) |
| **LB 鍵 (Left Bumper)** | 單次 | 重設陀螺儀 + 收回 Intake | 重設 Pigeon 2 角度 + 強制收回進氣口 |
| **RB 鍵 (Right Bumper)** | 單次 | 切換進氣口伸出/收回 | 切換 `IntakeSubsystem` 位置 (0.0m ↔ 0.3m) |
| **右搖桿 ↓ (推超過 50%)** | 按住 | 滾輪反轉吐球 | `RollerIntakeSubsystem.reverseRollers()` |
| **右搖桿 ↑ (推超過 50%)** | 按住 | Orbit 反轉 | `OrbitSubsystem.norunorbit()` |
| **POV 左 / 右鍵** | 按住 | 連續調整發射罩角度 | 按住時每 20ms 連續增加 / 減少 1.0 度 |

---

## 🛠️ 常數設定 (Constants)

所有的常數與硬體 ID 均妥善定義在 `Constants.java` 中，主要包含：

### SwerveConstants (底盤)
| 參數 | 值 |
| :--- | :--- |
| 輪距 (Track Width) | 22.75" |
| 軸距 (Wheel Base) | 20.75" |
| 驅動齒輪比 | 4.71 (L3) |
| 轉向齒輪比 | 287/11 |
| 最大速度 | 4.0 m/s |
| 旋轉速度倍率 | 1.0 |
| 驅動電流限制 | 供電 50A / 定子 50A |
| 轉向電流限制 | 供電 40A / 定子 40A |
| 陀螺儀 ID | 11 |

**Swerve 模組 CAN ID 與偏移量：**

| 模組 | 驅動 ID | 轉向 ID | 編碼器 ID | 偏移量 |
| :--- | :---: | :---: | :---: | :--- |
| 左前 (FL) | 6 | 5 | 3 | -0.2585 |
| 右前 (FR) | 2 | 46 | 1 | 0.2898 |
| 左後 (BL) | 8 | 7 | 4 | 0.3013 |
| 右後 (BR) | 4 | 3 | 2 | 0.1958 |

### ShooterConstants (發射器)
* 馬達 CAN ID: 9 (LT), 10 (LD), 11 (RT), 12 (RD)
* 供電電流限制: 30A / 定子電流限制: 30A
* PID: kP=0.07, kI=0.0, kD=0.0
* 前饋: kS=0.6, kV=0.2, kA=0.0
* 預設速度: 20.0 RPS

### HoodConstants (發射罩)
* 馬達 CAN ID: 15
* 供電電流限制: 40A
* 角度範圍: 0.0° ~ 45.0°
* 齒比: 10°/圈 (kOneMotor = 10/360)
* PID: kP=65.0, kI=0.0, kD=0.0
* 前饋: kS=0.6, kG=2.0

### 其他機構常數
| 機構 | 馬達 CAN ID | 電壓 | 供電電流限制 |
| :--- | :--- | :---: | :---: |
| Indexer (輸彈) | 31 | 10.0V | 30A |
| Orbit 上 | 14 | 10.0V | 30A |
| Orbit 下 | 13 | 10.0V | 30A |
| Intake (齒條) | 30 | 位置控制 | 30A |
| Roller 前 | 45 | 5.5V | 30A |
| Roller 後 | 41 | 5.5V | 30A |

### VisionConstants (視覺)
* 前方 Limelight: `limelight-up`
* 左方 Limelight: `limelight-left`
* 右方 Limelight: `limelight-right`

### AimConstants (自動瞄準)
* 仰角補償: 0.0° (可調整)
* 距離→仰角查表: 1.0m→2.0° / 2.0m→7.0° / 3.0m→10.0° / 4.0m→20.0°
* 距離→轉速查表: 1.0m→18 RPS / 2.0m→20 RPS / 3.0m→30 RPS / 4.0m→40 RPS

---

## 🚀 建置與部署 (Build & Deploy)

確保你的電腦已安裝 WPILib 2026 開發環境，並可透過 VS Code 或命令列進行操作：

### 1. 編譯專案 (Build Project)
在專案根目錄下執行：
```bash
./gradlew build
```

### 2. 部署至 RoboRIO (Deploy to Robot)
將電腦連接至機器人網路（透過 Wi-Fi、USB 或網路線），然後執行：
```bash
./gradlew deploy
```

### 3. 開啟模擬器 (Simulation)
若要在本機電腦上進行模擬測試：
```bash
./gradlew simulateJava
```

---

## 📦 第三方依賴 (Vendor Dependencies)

| 依賴 | 版本 | 用途 |
| :--- | :--- | :--- |
| WPILib / GradleRIO | 2026.2.1 | FRC 核心框架 |
| CTRE Phoenix 6 | 26.3.0 | TalonFX / CANcoder / Pigeon 2 控制 |
| PathPlannerLib | — | 自動階段路徑規劃 |

---

## 📁 專案結構 (Project Structure)

```
src/main/java/frc/robot/
├── Main.java                          # 程式進入點
├── Robot.java                         # 機器人主程式 (TimedRobot)
├── RobotContainer.java                # 子系統初始化與按鍵綁定
├── Constants.java                     # 所有常數定義
├── commands/
│   ├── AutoAimAndShoot.java           # 全自動瞄準與射擊指令
│   └── Autopose.java                  # (預留) 自動定位指令
└── subsystems/
    ├── DriveSubsystem.java            # Swerve 底盤控制
    ├── SwerveModule.java              # 單一 Swerve 模組
    ├── SwerveModuleConfigurator.java   # Swerve 模組配置器
    ├── ShooterSubsystem.java          # 四馬達飛輪發射器
    ├── HoodSubsystem.java             # 角度可調發射罩
    ├── IndexerSubsystem.java          # 輸彈機構
    ├── OrbitSubsystem.java            # 軌道傳輸機構
    ├── IntakeSubsystem.java           # 齒條伸縮進氣
    ├── RollerIntakeSubsystem.java     # 滾輪進氣機構
    └── VisionSubsystem.java           # 三鏡頭 Limelight 視覺系統
```
