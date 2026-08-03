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

/**
 * Blue2 / Red2 自動路線（同一份程式，自動依聯盟鏡像）
 * 包含兩段路徑與兩次定點射擊。
 */
public class Blue2Auto extends SequentialCommandGroup {

    /**
     * 建構子：初始化 Blue2 自動程序
     *
     * @param drive   底盤驅動子系統
     * @param intake  進件機構子系統
     * @param roller  滾輪進件子系統
     * @param shooter 發射器子系統
     * @param indexer 送球機構子系統
     * @param hood    發射器仰角子系統
     */
    public Blue2Auto(
            DriveSubsystem drive,
            IntakeSubsystem intake,
            RollerIntakeSubsystem roller,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            HoodSubsystem hood) {

        PathPlannerPath path1;
        PathPlannerPath path2;
        try {
            // 嘗試載入自動路線檔案
            path1 = PathPlannerPath.fromPathFile("Blue2-1");
            path2 = PathPlannerPath.fromPathFile("Blue2-2");
        } catch (Exception e) {
            e.printStackTrace();
            addCommands(Commands.print("[Blue2Auto] 路徑載入失敗，自動階段中止！"));
            return;
        }

        final PathPlannerPath finalPath1 = path1;

        addCommands(
            // 0. 依聯盟重置里程計到 Blue2-1 起點
            Commands.runOnce(() -> {
                Optional<Pose2d> startPoseOpt = finalPath1.getStartingHolonomicPose();
                if (startPoseOpt.isEmpty()) return;

                Pose2d startPose = startPoseOpt.get();
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    startPose = FlippingUtil.flipFieldPose(startPose);
                }
                drive.resetPose(startPose);
            }, drive),

            // 1. 跑 Blue2-1，並在出發後延遲指定時間才伸出進件機構 (Intake) 收球
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path1),
                new SequentialCommandGroup(
                    new WaitCommand(1.6), // ⬅️ 修改這裡的數字：決定出發後幾秒要降下進件機構
                    new InstantCommand(intake::extend, intake),
                    new InstantCommand(roller::runRollers, roller)
                )
            ),
            
            // 路徑結束，收回進件機構
            new InstantCommand(roller::stop, roller),
            new InstantCommand(intake::retract, intake),
            new WaitCommand(0.5), // 確保收回到位

            // 2. 第一次定點發射 (維持 PathPlanner 的旋轉角度，只啟動發射機構)
            new SequentialCommandGroup(
                new InstantCommand(() -> hood.setAngle(10.0), hood),
                new InstantCommand(shooter::runshooter, shooter),
                new WaitCommand(0.5), // 等待摩擦輪加速
                new InstantCommand(indexer::runindexer, indexer),
                new WaitCommand(0.5), // 等待球射出
                new InstantCommand(() -> {
                    shooter.stop();
                    indexer.stop();
                }, shooter, indexer)
            ),

            // 3. 跑 Blue2-2，並在出發後延遲指定時間才啟動進件機構 (Intake)
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path2),
                new SequentialCommandGroup(
                    new WaitCommand(1.7), // ⬅️ 修改這裡的數字：決定跑第二段時幾秒後降下進件機構
                    new InstantCommand(intake::extend, intake),
                    new InstantCommand(roller::runRollers, roller)
                )
            ),

            // 路徑結束，收回進件機構
            new InstantCommand(roller::stop, roller),
            new InstantCommand(intake::retract, intake),
            new WaitCommand(0.5),

            // 4. 第二次定點發射
            new SequentialCommandGroup(
                new InstantCommand(() -> hood.setAngle(10.0), hood),
                new InstantCommand(shooter::runshooter, shooter),
                new WaitCommand(0.5),
                new InstantCommand(indexer::runindexer, indexer),
                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    shooter.stop();
                    indexer.stop();
                }, shooter, indexer)
            )
        );
    }
}
