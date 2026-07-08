# FRC 9427 Robot Project (MYMK5N)
123
本專案是 **FRC Team 9427** 的機器人控制程式，基於 WPILib 的 **Command-based** 架構開發，採用 Java 語言編寫，並專為 2026 賽季進行配置。

## 🤖 專案概述 (Project Overview)

本專案控制一台配備 **Swerve Drive (四輪獨立轉向底盤)** 以及多個輔助機構（包括滾輪進氣口、齒條伸縮機構、雙排滾輪傳輸、角度可調發射罩、以及四馬達飛輪發射器）的 FRC 機器人。

### ⚙️ 開發環境與硬體諸元
* **WPILib 版本**: `2026.2.1`
* **Java 版本**: JDK 17
* **馬達控制器**: CTRE TalonFX (Kraken X60 / Falcon 500)
* **感測器**: CTRE CANcoder (轉向角度感測), Pigeon 2.0 (陀螺儀/IMU)
* **通訊協定**: CAN Bus (利用 CTRE Phoenix 6 API)

---

## 🏗️ 系統架構 (Subsystems)

本程式採用模組化設計，將機器人的硬體劃分為以下子系統 (Subsystems)：

### 1. 🛞 底盤子系統 ([DriveSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/DriveSubsystem.java))
* **類型**: Swerve Drive (四輪獨立轉向與驅動)
* **模組配置**: 每個 Swerve 模組包含一個驅動馬達 ([TalonFX](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/SwerveModule.java))、一個轉向馬達 ([TalonFX](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/SwerveModule.java)) 與一個用於絕對角度定位的 [CANcoder](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/SwerveModule.java)。
* **定位系統**: 採用 [Pigeon 2](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/DriveSubsystem.java) 陀螺儀進行場地導向 (Field-Relative) 控制，並使用 `SwerveDriveOdometry` 進行即時里程計更新。
* **特色**:
  * **防抖動設計 (Anti-Jitter/Deadband)**: 當搖桿輸入極小時，自動停止驅動並鎖定角度，防止馬達微幅抖動。
  * **狀態優化 (Swerve Optimize)**: 利用 WPILib 演算法優化目標轉角，避免模組旋轉超過 90 度，藉由反轉驅動馬達方向提高響應速度。

### 2. 🏹 發射器子系統 ([ShooterSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/ShooterSubsystem.java))
* **馬達配置**: 4 顆 TalonFX 馬達，分別為：
  * 左上 (LT) - 主控馬達
  * 左下 (LD) - 跟隨 LT (同向 Aligned)
  * 右上 (RT) - 跟隨 LT (反向 Opposed)
  * 右下 (RD) - 跟隨 LT (反向 Opposed)
* **控制模式**: 速度電壓閉迴路控制 (`VelocityVoltage`)，實現高精度的飛輪轉速控制。

### 3. 調整罩子系統 ([HoodSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/HoodSubsystem.java))
* **馬達配置**: 1 顆 TalonFX 馬達。
* **控制模式**: 利用 `MotionMagicVoltage` (運動魔法) 控制角度，設定平滑的加速度和巡航速度。
* **特色**: 內置角度安全限制 (`khoodMinAngle`: 0.0 度 ~ `khoodMaxAngle`: 45.0 度，直驅無齒輪箱)，避免機構超出物理極限。

### 4. 伸縮進氣子系統 ([IntakeSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/IntakeSubsystem.java))
* **類型**: 齒條伸縮機構 (Rack and Pinion)
* **馬達配置**: 1 顆 TalonFX 馬達，配置為剎車模式 (`NeutralModeValue.Brake`)。
* **控制模式**: 位置電壓控制 (`PositionVoltage`)。將旋轉圈數換算為公尺，精準控制進氣口伸出 (0.3m) 與收回 (0.0m) 狀態。

### 5. 滾輪進氣子系統 ([RollerIntakeSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/RollerIntakeSubsystem.java))
* **馬達配置**: 2 顆 TalonFX 馬達 (Frollintake 與 Rrollintake)，呈反向跟隨關係。
* **控制模式**: 比例輸出 (`DutyCycleOut`) 控制滾輪轉動以吸入遊戲對象。

### 6. 傳輸通道子系統 ([IndexerSubsystem.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/subsystems/IndexerSubsystem.java))
* **馬達配置**: 3 顆 TalonFX 馬達，包含兩個軌道馬達 (uorbit, dorbit) 與一個進彈馬達 (indexer)。
* **特色**: 控制通道內的傳輸滾輪，將遊戲對象穩定位移並送入發射器中。

---

## 🎮 控制器按鍵配置 (Controller Bindings)

所有的按鍵綁定均在 [RobotContainer.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/RobotContainer.java) 中設定。目前配置使用單個 Xbox 搖桿 (Port 0) 控制：

| 按鍵 (Button) | 控制動作 (Action) | 機構反應 (Mechanism) |
| :--- | :--- | :--- |
| **左搖桿 (Left Stick)** | 機器人平移 (X/Y Axis) | 底盤移動 (Field-Relative) |
| **左/右扳機 (L/R Triggers)**| 機器人旋轉 (Left/Right Turn)| 左/右扳機差值決定旋轉速度與方向 |
| **A 鍵** | 開啟/關閉 發射器飛輪 | 切換 `ShooterSubsystem` (設定為 60 rps) |
| **Y 鍵** | 開啟/關閉 進氣滾輪 | 切換 `RollerIntakeSubsystem` 滾輪轉動 |
| **X 鍵** | 開啟/關閉 傳輸通道 | 切換 `IndexerSubsystem` 進彈與軌道馬達 |
| **LB 鍵 (Left Bumper)** | 重設陀螺儀 (Zero Odometry) | 重設 Pigeon 2 角度，以當前朝向作為場地前方 |
| **RB 鍵 (Right Bumper)** | 切換進氣口伸出/收回 | 切換 `IntakeSubsystem` 位置 (0.0m ↔ 0.3m) |
| **POV 上 / 下鍵** | 單次調整發射罩角度 | 每次按下增加 / 減少 1.0 度 |
| **POV 左 / 右鍵 (按住)** | 連續微調發射罩角度 | 按住時每 20ms 連續增加 / 減少 0.1 度 |

---

## 🛠️ 常數設定 (Constants)

所有的常數與硬體 ID 均妥善定義在 [Constants.java](file:///c:/JAVA/MYMK5N/src/main/java/frc/robot/Constants.java) 中，主要包含：
* **SwerveConstants**: 底盤輪距 (`kTrackWidth`: 22.75" / `kWheelBase`: 20.75")、齒輪比 (L3 驅動: 4.71, 轉向: 287/11)、馬達 CAN ID (1~8)、編碼器 ID、PID 與前饋常數 ($k_S, k_V, k_A$) 等。
* **ShooterConstants**: 發射器馬達 CAN ID (9~12)、電流限制 (100A)、PID 與速度設定 (60.0)。
* **HoodConstants**: 發射罩馬達 ID (15)、Motion Magic 限制與角度極限。
* **Intake/RollerIntake/Indexer/OrbitConstants**: 各輔助馬達之 ID、運作電壓、齒比換算及電流限制。

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
