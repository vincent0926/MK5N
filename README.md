# FRC 9427 機器人程式（2026 賽季）

FRC Team 9427 的機器人控制程式。WPILib Command-based 架構，Java 17，Swerve 底盤，Limelight 視覺自動瞄準，PathPlanner 自動路線。

| 項目 | 版本 / 規格 |
| :--- | :--- |
| WPILib / GradleRIO | 2026.2.1 |
| CTRE Phoenix 6 | 26.3.0（TalonFX、CANcoder、Pigeon 2） |
| PathPlannerLib | 見 `vendordeps/PathplannerLib.json` |
| 視覺 | Limelight：`limelight-up`（前）、`limelight-left`、`limelight-right`；回傳球用 `Limelight4Subsystem` |
| 底盤 | SDS MK5n L3（驅動齒比 5.27），最高速 4.0 m/s |

> 目前狀態、已知問題與待決定事項請看 [HANDOFF.md](HANDOFF.md)；完整更動紀錄看 [CHANGE_REPORT.md](CHANGE_REPORT.md)。**程式尚未全面上機驗證**，首次上機請在空曠處低速測試。

## 快速開始

需要 WPILib 2026 開發環境（VS Code + WPILib 擴充套件）。

```bash
# Windows：系統 Java 太新（26）時，Gradle 8.11 會失敗，先指向 WPILib 內建 JDK
export JAVA_HOME="C:\\Users\\Public\\wpilib\\2026\\jdk"

./gradlew build --offline     # 編譯 + 跑單元測試
./gradlew deploy              # 部署到 roboRIO（需連上機器人網路）
./gradlew simulateJava        # 本機模擬
```

## 操作方式（Xbox 手把，Port 0）

| 按鍵 | 動作 |
| :--- | :--- |
| 左搖桿 | 平移（駕駛座標，左 Bumper 設定的「前方」） |
| 左 / 右扳機 | 旋轉（左扳機左轉、右扳機右轉） |
| **A**（切換） | 射擊選擇器 `ShootSelector`，依狀況自動選模式（見下） |
| B（切換） | 單純運轉飛輪 |
| X（切換） | 同時運轉 Indexer 與 Orbit |
| Y（切換） | 滾輪進球 |
| 左 Bumper | 以目前車頭方向重設「前方」並收回 Intake（不動 Swerve 位姿） |
| 右 Bumper | 切換 Intake 伸出 / 收回 |
| 右搖桿 下（>50%，按住） | 滾輪反轉吐球 |
| 右搖桿 上（>50%，按住） | Orbit 反轉 |
| D-pad 左 / 右（按住） | 連續微調 Hood 角度 ∓/±1° |

### A 鍵射擊模式

`ShootSelector` 每個週期依下列優先順序決定要跑哪個指令：

1. **`ReturnBall`**：Limelight4 看到回傳球 tag → 依 tag 距離查表設定 Hood 與飛輪，1 秒後送球。
2. **`AutoAimAndShoot`**：看得到 HUB 且駕駛沒推左搖桿 / 扳機（推了就改走手動的 `Operate`）→ 底盤原地自動對準、Hood 與飛輪依距離查表，1 秒後送球。
3. **`Operate`**：其他情況 → 固定參數（飛輪 15 RPS、Hood 30°），1 秒後送球。

查表與常數都在 `Constants.java`（`AimConstants`、`ReturnConstants`）。

## 程式結構

```
src/main/java/frc/robot/
├── Main.java / Robot.java        # 進入點、TimedRobot 生命週期
├── RobotContainer.java           # 子系統建立、預設指令、按鍵綁定、PathPlanner Named Commands
├── Constants.java                # 馬達 ID、PID、查表、限制
├── commands/
│   ├── ShootSelector.java        # A 鍵：依條件在下面三個指令間切換
│   ├── AutoAimAndShoot.java      # 視覺對準 HUB + 查表射擊
│   ├── ReturnBall.java           # 中場回傳球
│   └── Operate.java              # 固定參數射擊
└── subsystems/
    ├── Swerve/                   # 目前使用的底盤（自製里程計、PathPlanner 整合）
    ├── Swerve2/                  # 舊式 WPILib 底盤，保留但目前未使用
    ├── Math/SwerveMath.java      # 底盤純數學（可單元測試）
    ├── Shooter/                  # ShooterSubsystem、HoodSubsystem、OrbitSubsystem
    ├── Hopper/                   # IndexerSubsystem
    ├── Intake/                   # IntakeSubsystem（齒條）、RollerIntakeSubsystem（滾輪）
    └── Vision/                   # VisionSubsystem、Limelight4Subsystem、LimelightHelpers
src/main/deploy/pathplanner/      # 自動路線（Blue1、Blue3）與設定
src/test/java/                    # 單元測試（SwerveMath、Swerve、AutoLoad、Vision）
```

## 自動階段

自動路線在 PathPlanner 編輯，存放於 `src/main/deploy/pathplanner/`。`Swerve` 建構時呼叫 `AutoBuilder.configure()`；`RobotContainer` 註冊的 Named Commands：

| 名稱 | 功能 |
| :--- | :--- |
| `AutoAimAndShoot` | 自動瞄準射擊（逾時 4 秒） |
| `IntakeExtend` / `IntakeRetract` | 伸出 / 收回 Intake |
| `StartRoller` / `StopRoller` | 啟動 / 停止進球滾輪 |

`AutoLoadTest` 會確認自動路線都能被載入，且用到的 Named Commands 皆已註冊。新增路線後請重跑測試。

## 硬體 CAN ID

| 機構 | ID |
| :--- | :--- |
| 底盤驅動 FL / FR / BL / BR | 6 / 2 / 8 / 4 |
| 底盤轉向 FL / FR / BL / BR | 5 / 46 / 7 / 3 |
| 轉向 CANcoder FL / FR / BL / BR | 3 / 1 / 4 / 2 |
| Pigeon 2 | 11 |
| 飛輪 LT（主）/ LD / RT / RD | 9 / 10 / 11 / 12 |
| Hood | 15 |
| Indexer | 31 |
| Orbit 上 / 下 | 14 / 13 |
| Intake（齒條） | 30 |
| 滾輪 前 / 後 | 45 / 41 |

以 `Constants.java` 為準，換硬體時請同步更新此表。

## 開發注意事項

- 改完請更新 `HANDOFF.md`；沒有明確要求不要 commit。
- 機器是**定點射擊**：`ShootSelector` 只負責在三種射擊指令間排程切換，執行期間會佔用底盤與射擊相關子系統（A 鍵開著時不能開車，再按一次 A 結束）。
- 上機前建議確認：遙控前後左右方向、扳機轉向、左 Bumper 重設前方、紅方跑完自動後遙控方向、A 鍵各模式切換是否穩定。
- `UseFocControl` 預設為 `false`，確認每顆馬達都有 Phoenix Pro 授權後才改為 `true`。
