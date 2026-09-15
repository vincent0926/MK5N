
package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OrbitSubsystem;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer 類別是機器人主要架構所在之處。
 * 包含子系統、操作搖桿以及各種按鈕觸發的指令綁定。
 */
public class RobotContainer {

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
        private final OrbitSubsystem orbitSubsystem = new OrbitSubsystem();
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final RollerIntakeSubsystem rollerIntakeSubsystem = new RollerIntakeSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // 註冊 PathPlanner Named Commands (必須在 AutoChooser 建立之前註冊)
                NamedCommands.registerCommand("ShootAndIndex",
                                Commands.sequence(
                                                Commands.runOnce(() -> shooterSubsystem.runshooter(), shooterSubsystem),
                                                Commands.waitSeconds(0.5), // 等待摩擦輪加速
                                                Commands.runOnce(() -> indexerSubsystem.runindexer(), indexerSubsystem),
                                                Commands.waitSeconds(0.5), // 等待球射出
                                                Commands.runOnce(() -> {
                                                        shooterSubsystem.stop();
                                                        indexerSubsystem.stop();
                                                }, shooterSubsystem, indexerSubsystem)));

                NamedCommands.registerCommand("IntakeExtend",
                                Commands.runOnce(() -> intakeSubsystem.extend(), intakeSubsystem));
                NamedCommands.registerCommand("IntakeRetract",
                                Commands.runOnce(() -> intakeSubsystem.retract(), intakeSubsystem));
                NamedCommands.registerCommand("StartRoller",
                                Commands.runOnce(() -> rollerIntakeSubsystem.runRollers(), rollerIntakeSubsystem));
                NamedCommands.registerCommand("StopRoller",
                                Commands.runOnce(() -> rollerIntakeSubsystem.stop(), rollerIntakeSubsystem));

                // 建立 PathPlanner Auto Chooser 並發布到 SmartDashboard
                // AutoBuilder.configure() 已在 DriveSubsystem 建構子中完成，這裡只需建立選單
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                // 設定底盤預設指令 (搖桿控制)
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.run(() -> {
                                        // 1. 處理移動 (左搖桿)
                                        // 這裡將原本的 y 和 x 加上負號 (反轉)，讓 Shooter 變成車頭
                                        double y = -MathUtil.applyDeadband(driverController.getLeftY(), 0.06);
                                        double x = -MathUtil.applyDeadband(driverController.getLeftX(), 0.06);

                                        // 2. 處理旋轉 (扳機鍵：右扳機 - 左扳機)
                                        // 假設：按下右扳機 -> 右轉，按下左扳機 -> 左轉
                                        double turnLeft = driverController.getLeftTriggerAxis();
                                        double turnRight = driverController.getRightTriggerAxis();

                                        // 計算總旋轉量
                                        double rot = (turnLeft - turnRight);

                                        // 應用 Deadband 和平方曲線（讓低速更絲滑）
                                        rot = MathUtil.applyDeadband(rot, 0.06);
                                        rot = Math.copySign(rot * rot, rot);

                                        rot = rot * SwerveConstants.turnSpeed;

                                        // 指令底盤以指定的 X 軸、Y 軸平移速度以及旋轉速度移動，並且啟用『場地導向 (Field-Relative)』模式。
                                        driveSubsystem.drive(y, x, rot, true);
                                }));

                configureBindings();
        }

        private void configureBindings() {

                // 綁定 A 鍵：全自動瞄準與發射 (按一下啟動，再按一下停止)
                // 並加上 `.until()`，只要使用者去推動左搖桿或左右扳機鍵，就會自動中斷 A 鍵指令
                driverController.a().toggleOnTrue(
                                new AutoAimAndShoot(
                                                visionSubsystem, hoodSubsystem, shooterSubsystem,
                                                indexerSubsystem, driveSubsystem, orbitSubsystem)
                                                .until(() -> Math.abs(driverController.getLeftY()) > 0.1 ||
                                                                Math.abs(driverController.getLeftX()) > 0.1 ||
                                                                Math.abs(driverController.getLeftTriggerAxis()) > 0.1 ||
                                                                Math.abs(driverController.getRightTriggerAxis()) > 0.1));

                // 綁定 Y 鍵：啟動/停止滾輪進件機構 (Roller Intake)
                driverController.y().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> rollerIntakeSubsystem.runRollers(),
                                                () -> rollerIntakeSubsystem.stop(),
                                                rollerIntakeSubsystem));

                // 綁定 X 鍵：同時啟動/停止輸彈機構 (Indexer) 與 Orbit 機構
                driverController.x().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> {
                                                        indexerSubsystem.runindexer();
                                                        orbitSubsystem.runorbit();
                                                },
                                                () -> {
                                                        indexerSubsystem.stop();
                                                        orbitSubsystem.stop();
                                                },
                                                indexerSubsystem,
                                                orbitSubsystem));

                driverController.leftBumper().onTrue(
                                edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
                                        driveSubsystem.resetpigeonOdometry();
                                        intakeSubsystem.retract();

                                }, driveSubsystem, intakeSubsystem));

                // 綁定右保險桿 (Right Bumper)：切換進件機構 (Intake) 伸出/收回狀態
                driverController.rightBumper().onTrue(
                                new InstantCommand(
                                                () -> intakeSubsystem.togglePosition(),
                                                intakeSubsystem));

                // 綁定 B 鍵：射出機構 (Shooter) - 單純啟動/停止摩擦輪
                driverController.b().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> shooterSubsystem.runshooter(),
                                                () -> shooterSubsystem.stop(),
                                                shooterSubsystem));
                // 吐球
                driverController.axisGreaterThan(
                                edu.wpi.first.wpilibj.XboxController.Axis.kRightY.value, 0.5).whileTrue(
                                                Commands.startEnd(
                                                                () -> rollerIntakeSubsystem.reverseRollers(),
                                                                () -> rollerIntakeSubsystem.stop(),
                                                                rollerIntakeSubsystem));
                driverController.axisLessThan(
                                edu.wpi.first.wpilibj.XboxController.Axis.kRightY.value, -0.5).whileTrue(
                                                Commands.startEnd(
                                                                () -> orbitSubsystem.norunorbit(),
                                                                () -> orbitSubsystem.stop(),
                                                                orbitSubsystem));

                // 設定仰角對應表
                edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap distanceToAngleMap = new edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap();
                for (double[] point : frc.robot.Constants.AimConstants.kDistanceToAngleMap) {
                        distanceToAngleMap.put(point[0], point[1]);
                }

                // 1. 移除了原本的預設自動追蹤指令
                // 現在仰角「平常不會動」，只有在按下 A 鍵啟動 AutoAimAndShoot 時才會去讀取並設定仰角。

                // // 2. 上下鍵：單次移動 1 度
                // driverController.povUp().onTrue(
                //                 edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> hoodSubsystem.addAngle(1.0),
                //                                 hoodSubsystem));
                // driverController.povDown().onTrue(
                //                 edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> hoodSubsystem.addAngle(-1.0),
                //                                 hoodSubsystem));

                // 3. 左右鍵：持續移動 (按住時每 20ms 移動 1 度)
                driverController.povRight().whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(1.0), hoodSubsystem));

                driverController.povLeft().whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(-1.0), hoodSubsystem));
        }

        /**
         * 獲取所選擇的自動模式指令。
         *
         * @return 將在自動模式中執行的指令。
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * 這個方法將會被 Robot.java 的 robotPeriodic() 呼叫。
         * 確保無論在手動還是自動模式 (Auto 執行時)，全場視覺定位都會持續在背景更新。
         */
        public void periodic() {
                // 同步陀螺儀角度給 Limelight，啟用 MegaTag 2 定位
                visionSubsystem.updateMegaTag2(
                                driveSubsystem.getHeading(),
                                driveSubsystem.getYawRate());

                List<VisionSubsystem.EstimatedRobotPose> poses = visionSubsystem.getEstimatedPoses();
                for (VisionSubsystem.EstimatedRobotPose p : poses) {
                        driveSubsystem.addVisionMeasurement(p.estimatedPose, p.timestampSeconds);
                }
        }

        /**
         * 讓 Robot.java 可以在自動/手動模式初始時強制收回 Intake
         */
        public void retractIntake() {
                intakeSubsystem.retract();
        }
}
