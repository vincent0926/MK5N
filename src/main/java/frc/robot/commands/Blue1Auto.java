package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Blue1 / Red1 自動路線（同一份程式，自動依聯盟鏡像）
 *
 * 路徑鏡像原理：
 *   - AutoBuilder.followPath() 已透過 AutoBuilder.configure() 的聯盟提供者 (alliance supplier)
 *     自動判斷是否為紅方，紅方時路徑水平翻轉，藍方維持原路徑。
 *   - 里程計起始點也在此統一依聯盟重置，確保座標系正確。
 *
 * 流程概要：
 *  0. 依聯盟重置里程計到 Blue1-1 起點（為紅方時自動鏡像）
 *  1. 自動開始 → 立即伸出進件機構 (intake)
 *  2. T=1.5s → 開啟進件機構滾輪
 *  3. [並行] 跑 Blue1-1 路徑（以路徑結束為截止點 (deadline)，路徑跑完才停滾輪）
 *  4. Blue1-1 終點 → 進行定點射擊 4 秒
 *  5. 射擊完 → 收回進件機構，等 0.8s 確保收回到位
 *  6. 出發跑 Blue1-2 路徑
 *  7. Blue1-2 開始 3 秒後 → 伸出進件機構 + 開啟滾輪
 *  8. Blue1-2 終點 → 進行定點射擊 5 秒
 *  9. 結束，關閉所有機構
 */
public class Blue1Auto extends SequentialCommandGroup {

    /**
     * 建構子：初始化 Blue1 自動程序
     *
     * @param drive   底盤驅動子系統
     * @param intake  進件機構子系統
     * @param roller  滾輪進件子系統
     * @param shooter 發射器子系統
     * @param indexer 送球機構子系統
     * @param hood    發射器仰角子系統
     * @param vision  視覺子系統
     */
    public Blue1Auto(
            DriveSubsystem drive,
            IntakeSubsystem intake,
            RollerIntakeSubsystem roller,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            HoodSubsystem hood,
            VisionSubsystem vision) {

        // ── 載入路徑檔案 ──────────────────────────────────────────────────
        PathPlannerPath path1;
        PathPlannerPath path2;
        try {
            path1 = PathPlannerPath.fromPathFile("Blue1-1");
            path2 = PathPlannerPath.fromPathFile("Blue1-2");
        } catch (Exception e) {
            e.printStackTrace();
            addCommands(Commands.print("[Blue1Auto] 路徑載入失敗，自動階段中止！"));
            return;
        }

        // 把 path1 存成 final，讓 lambda 可以使用
        final PathPlannerPath finalPath1 = path1;

        addCommands(

            // ════════════════════════════════════════════════════════════════
            // 步驟 0：重置里程計到正確起始點
            //   藍方 → Blue1-1 起點原始座標
            //   紅方 → Blue1-1 起點水平鏡像座標
            //
            //   為什麼需要這步？
            //   AutoBuilder.followPath() 會用里程計判斷現在在哪，
            //   如果里程計是錯的，路徑跟蹤會產生偏差。
            // ════════════════════════════════════════════════════════════════
            Commands.runOnce(() -> {
                // 取得 Blue1-1 的起始姿態（PathPlanner 儲存的原始藍方座標）
                Optional<Pose2d> startPoseOpt = finalPath1.getStartingHolonomicPose();
                if (startPoseOpt.isEmpty()) return;

                Pose2d startPose = startPoseOpt.get();

                // 若為紅方，將起始點水平鏡像到紅方場地座標
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    startPose = FlippingUtil.flipFieldPose(startPose);
                }

                // 重置里程計
                drive.resetPose(startPose);
            }, drive),

            // ════════════════════════════════════════════════════════════════
            // 階段 1：Blue1-1 路徑 + 進件機構時序
            //   Deadline (截止點) = 路徑跑完
            //   並行   = 進件機構伸出（T=0） + 滾輪啟動（T=1.5s）
            //   ※ 紅方時 followPath() 自動鏡像路徑，不需額外處理
            // ════════════════════════════════════════════════════════════════
            new ParallelDeadlineGroup(
                // ── 截止點：Blue1-1 路徑（紅方自動鏡像）──
                AutoBuilder.followPath(path1),

                // ── 並行：進件機構時序控制 ──
                new SequentialCommandGroup(
                    // T=0s：立即伸出進件機構
                    new InstantCommand(intake::extend, intake),
                    // 等待 1.5 秒
                    new WaitCommand(1.5),
                    // T=1.5s：開啟進件機構滾輪
                    new InstantCommand(roller::runRollers, roller)
                    // 路徑（deadline）結束後此群組 (group) 會被中斷
                )
            ),

            // ── 路徑結束後立即停止進件機構滾輪 ──────────────────────────────
            new InstantCommand(roller::stop, roller),

            // ════════════════════════════════════════════════════════════════
            // 階段 2：Blue1-1 終點 — 設定 10 度仰角 + 射擊 4 秒
            // ════════════════════════════════════════════════════════════════
            new SequentialCommandGroup(
                new InstantCommand(() -> hood.setAngle(10.0), hood),
                new WaitCommand(0.5),
                new InstantCommand(shooter::runshooter, shooter),
                new WaitCommand(0.5),
                new InstantCommand(indexer::runindexer, indexer),
                new WaitCommand(4.0),
                new InstantCommand(() -> {
                    shooter.stop();
                    indexer.stop();
                }, shooter, indexer)
            ),

            // ════════════════════════════════════════════════════════════════
            // 階段 3：收回進件機構，等待 0.8 秒確保完全到位後才出發
            // ════════════════════════════════════════════════════════════════
            new InstantCommand(intake::retract, intake),
            new WaitCommand(0.2),

            // ════════════════════════════════════════════════════════════════
            // 階段 4：Blue1-2 路徑 + 3 秒後伸出進件機構
            //   Deadline (截止點) = 路徑跑完
            //   並行   = 等 3 秒後伸出進件機構 + 開啟滾輪
            //   ※ 紅方時 followPath() 自動鏡像路徑
            // ════════════════════════════════════════════════════════════════
            new ParallelDeadlineGroup(
                // ── 截止點：Blue1-2 路徑（紅方自動鏡像）──
                AutoBuilder.followPath(path2),

                // ── 並行：Blue1-2 開始 3 秒後啟動進件機構 ──
                new SequentialCommandGroup(
                    new WaitCommand(3.0),
                    new InstantCommand(intake::extend, intake),
                    new InstantCommand(roller::runRollers, roller)
                    // 路徑（deadline）結束後此群組 (group) 會被中斷
                )
            ),

            // ════════════════════════════════════════════════════════════════
            // 階段 5：Blue1-2 終點 — 設定 10 度仰角 + 射擊 5 秒
            // ════════════════════════════════════════════════════════════════
            new SequentialCommandGroup(
                new InstantCommand(() -> hood.setAngle(10.0), hood),
                new WaitCommand(0.5),
                new InstantCommand(shooter::runshooter, shooter),
                new WaitCommand(0.5),
                new InstantCommand(indexer::runindexer, indexer),
                new WaitCommand(5.0),
                new InstantCommand(() -> {
                    shooter.stop();
                    indexer.stop();
                }, shooter, indexer)
            ),

            // ════════════════════════════════════════════════════════════════
            // 收尾：關閉所有機構
            // ════════════════════════════════════════════════════════════════
            new InstantCommand(roller::stop, roller),
            new InstantCommand(intake::retract, intake)
        );
    }
}
