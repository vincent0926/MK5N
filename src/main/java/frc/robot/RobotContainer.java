
package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.commands.Operate;
import frc.robot.commands.ReturnBall;
import frc.robot.commands.ShootSelector;
import frc.robot.subsystems.Hopper.IndexerSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.OrbitSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Vision.Limelight4Subsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/**
 * RobotContainer 類別是機器人主要架構所在之處。
 * 包含子系統、操作搖桿以及各種按鈕觸發的指令綁定。
 */
public class RobotContainer {

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
        private final OrbitSubsystem orbitSubsystem = new OrbitSubsystem();
        private final Swerve driveSubsystem = new Swerve();
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final RollerIntakeSubsystem rollerIntakeSubsystem = new RollerIntakeSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final Limelight4Subsystem limelight4Subsystem = new Limelight4Subsystem();
        private final SendableChooser<Command> autoChooser;
        private final Operate operateCommand = new Operate(hoodSubsystem, shooterSubsystem, indexerSubsystem,
                        orbitSubsystem);
        private final PS5Controller PS5 = new PS5Controller(1);

        // 駕駛的「前方」相對 Swerve 朝向的偏移 (rad)。左 bumper 只改這個，不動 Swerve 位姿 (自動與視覺還要用)
        private double driverHeadingOffsetRad = 0.0;

        public RobotContainer() {
                // 註冊 PathPlanner Named Commands (必須在 AutoChooser 建立之前註冊)

                NamedCommands.registerCommand("AutoAimAndShoot",
                                new AutoAimAndShoot(
                                                visionSubsystem, hoodSubsystem, shooterSubsystem,
                                                indexerSubsystem, driveSubsystem, orbitSubsystem)
                                                .withTimeout(4.0));
                NamedCommands.registerCommand("IntakeExtend",
                                Commands.runOnce(() -> intakeSubsystem.extend(), intakeSubsystem));
                NamedCommands.registerCommand("IntakeRetract",
                                Commands.runOnce(() -> intakeSubsystem.retract(), intakeSubsystem));
                NamedCommands.registerCommand("StartRoller",
                                Commands.runOnce(() -> rollerIntakeSubsystem.runRollers(), rollerIntakeSubsystem));
                NamedCommands.registerCommand("StopRoller",
                                Commands.runOnce(() -> rollerIntakeSubsystem.stop(), rollerIntakeSubsystem));

                // 建立 PathPlanner Auto Chooser 並發布到 SmartDashboard
                // TODO: 新的 Swerve 還沒呼叫 AutoBuilder.configure()，在接好之前自動模式選單是空的。
                // 沒有 configure 就直接 buildAutoChooser 會丟例外，機器人程式一開機就當掉，所以先擋住。
                if (AutoBuilder.isConfigured()) {
                        autoChooser = AutoBuilder.buildAutoChooser("Blue3");
                } else {
                        DriverStation.reportWarning("AutoBuilder 尚未設定，自動模式停用", false);
                        autoChooser = new SendableChooser<>();
                        autoChooser.setDefaultOption("None", Commands.none());
                }
                SmartDashboard.putData("Blue3", autoChooser);

                // 設定底盤預設指令 (搖桿控制)
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.run(() -> {
                                        // 自動瞄準、自動路線會佔用 driveSubsystem，排程器自動把這個預設指令擠掉，不用另外判斷。
                                        // 非遙控時 (例如自動路線跑完、還在自動時段) 停在原地，不沿用舊速度
                                        if (!DriverStation.isTeleopEnabled()) {
                                                driveSubsystem.stop();
                                                return;
                                        }

                                        // 1. 處理移動 (左搖桿)
                                        // 這裡將原本的 y 和 x 加上負號 (反轉)，讓 Shooter 變成車頭
                                        double y;
                                        double x;
                                        double turnLeft;
                                        double turnRight;
                                        if (driverController.isConnected()) {
                                                y = -MathUtil.applyDeadband(driverController.getLeftY() , 0.06);
                                                x = -MathUtil.applyDeadband(driverController.getLeftX(), 0.06);
                                                turnLeft = MathUtil.applyDeadband(
                                                                driverController.getLeftTriggerAxis()  ,0.01);
                                                turnRight = MathUtil
                                                                .applyDeadband(
                                                                driverController.getRightTriggerAxis() ,0.01);

                                        } else {

                                                y = -MathUtil.applyDeadband(PS5.getLeftY() , 0.06);
                                                x = -MathUtil.applyDeadband(PS5.getLeftX() , 0.06);
                                                turnLeft = MathUtil.applyDeadband(ps5Trigger(PS5.getL2Axis() ),
                                                                0.01);
                                                turnRight = MathUtil.applyDeadband(ps5Trigger(PS5.getR2Axis() ),
                                                                0.01);
                                        }
                                        // 2. 處理旋轉 (扳機鍵：右扳機 - 左扳機)
                                        // 假設：按下右扳機 -> 右轉，按下左扳機 -> 左轉

                                        // 計算總旋轉量
                                        double rot = (turnLeft - turnRight);

                                        // 應用 Deadband 和平方曲線（讓低速更絲滑）
                                        rot = MathUtil.applyDeadband(rot, 0.06);
                                        rot = Math.copySign(rot * rot, rot);

                                        rot = rot * SwerveConstants.turnSpeed;

                                        // 搖桿是「駕駛座標」(左 bumper 設定的前方)，轉成 Swerve 的場地座標後再送出
                                        double speed = Constants.Swerve.MaxDriveVelocityMps;
                                        double cos = Math.cos(driverHeadingOffsetRad);
                                        double sin = Math.sin(driverHeadingOffsetRad);
                                        double forwardMps = (y * cos - x * sin) * speed;
                                        double leftMps = (y * sin + x * cos) * speed;

                                        // 除錯：enable 就旋轉時看這幾個值是哪個不是 0
                                        SmartDashboard.putBoolean("Drive/PS5 Connected", PS5.isConnected());
                                        SmartDashboard.putNumber("Drive/Raw L2", PS5.getL2Axis());
                                        SmartDashboard.putNumber("Drive/Raw R2", PS5.getR2Axis());
                                        SmartDashboard.putNumber("Drive/Raw LeftX", PS5.getLeftX());
                                        SmartDashboard.putNumber("Drive/Raw LeftY", PS5.getLeftY());
                                        SmartDashboard.putNumber("Drive/Rot Cmd", rot);
                                        SmartDashboard.putNumber("Drive/Forward Cmd", forwardMps);
                                        SmartDashboard.putNumber("Drive/Left Cmd", leftMps);

                                        driveSubsystem.setFieldVelocity(
                                                        (float) forwardMps, (float) leftMps, (float) rot, false);
                                })
                                                // 被 A 鍵 (ShootSelector) 等指令擠掉時清掉最後的搖桿速度，
                                                // 否則 Swerve.operate() 會沿用舊速度一直開
                                                .finallyDo(interrupted -> driveSubsystem.stop()));

                configureBindings();
        }

        private void configureBindings() {
                // PS5 (port 1) 依 Xbox 的相對位置綁同一個指令，兩支搖桿共用 toggle 狀態：
                // A=Cross(×) B=Circle(○) X=Square(□) Y=Triangle(△) LB=L1 RB=R1
                // 右搖桿上下、十字鍵左右位置相同

                driverController.a().or(PS5::getCrossButton).toggleOnTrue(
                                new ShootSelector(
                                                new AutoAimAndShoot(
                                                                visionSubsystem, hoodSubsystem, shooterSubsystem,
                                                                indexerSubsystem, driveSubsystem, orbitSubsystem),
                                                new ReturnBall(limelight4Subsystem, hoodSubsystem, shooterSubsystem,
                                                                indexerSubsystem, orbitSubsystem),
                                                operateCommand,
                                                // 回傳球條件：Limelight4 看得到回傳球的 tag 嗎？
                                                () -> limelight4Subsystem.canReturnBall(),
                                                // HUB 條件：看得到 HUB，而且距離有算出來嗎？
                                                () -> visionSubsystem.hasHubTarget()
                                                                && visionSubsystem.getHubDistance() > 0,
                                                // 駕駛操作條件：任一支搖桿的左搖桿或板機有沒有在動？
                                                // PS5 扳機放開可能是 -1，要先經過 ps5Trigger() 換算
                                                () -> Math.abs(driverController.getLeftY()) > 0.1
                                                                || Math.abs(driverController.getLeftX()) > 0.1
                                                                || Math.abs(driverController.getLeftTriggerAxis()) > 0.1
                                                                || Math.abs(driverController
                                                                                .getRightTriggerAxis()) > 0.1
                                                                || Math.abs(PS5.getLeftY()) > 0.1
                                                                || Math.abs(PS5.getLeftX()) > 0.1
                                                                || ps5Trigger(PS5.getL2Axis()) > 0.1
                                                                || ps5Trigger(PS5.getR2Axis()) > 0.1));

                // 綁定 Y 鍵 / PS5 △：啟動/停止滾輪進件機構 (Roller Intake)
                driverController.y().or(PS5::getTriangleButton).toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> rollerIntakeSubsystem.runRollers(),
                                                () -> rollerIntakeSubsystem.stop(),
                                                rollerIntakeSubsystem));

                // // 綁定 X 鍵 / PS5 □：同時啟動/停止輸彈機構 (Indexer) 與 Orbit 機構
                // driverController.x().or(PS5::getSquareButton).toggleOnTrue(
                //                 edu.wpi.first.wpilibj2.command.Commands.startEnd(
                //                                 () -> {
                //                                         indexerSubsystem.runindexer();
                //                                         orbitSubsystem.runorbit();
                //                                 },
                //                                 () -> {
                //                                         indexerSubsystem.stop();
                //                                         orbitSubsystem.stop();
                //                                 },
                //                                 indexerSubsystem,
                //                                 orbitSubsystem));

                // // 綁定 B 鍵 / PS5 ○：射出機構 (Shooter) - 單純啟動/停止摩擦輪
                // driverController.b().or(PS5::getCircleButton).toggleOnTrue(
                //                 edu.wpi.first.wpilibj2.command.Commands.startEnd(
                //                                 () -> shooterSubsystem.runshooter(),
                //                                 () -> shooterSubsystem.stop(),
                //                                 shooterSubsystem));

                // 左 bumper / PS5 L1：歸零駕駛前方並收回 Intake
                driverController.leftBumper().or(PS5::getL1Button).onTrue(
                                edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
                                        resetDriverHeading();
                                        intakeSubsystem.retract();

                                }, intakeSubsystem));

                // PS5 Options 鍵：歸零陀螺儀 (把車頭現在朝的方向設為「前方」)，不收 Intake
                new Trigger(PS5::getOptionsButton).onTrue(
                                Commands.runOnce(this::resetDriverHeading).ignoringDisable(true));

                // 綁定右保險桿 (Right Bumper) / PS5 R1：切換進件機構 (Intake) 伸出/收回狀態
                driverController.rightBumper().or(PS5::getR1Button).onTrue(
                                new InstantCommand(
                                                () -> intakeSubsystem.togglePosition(),
                                                intakeSubsystem));

                // 吐球 (右搖桿往下，Xbox 與 PS5 都是 Y 軸正值)
                driverController.axisGreaterThan(
                                edu.wpi.first.wpilibj.XboxController.Axis.kRightY.value, 0.5)
                                .or(() -> PS5.getRightY() > 0.5).whileTrue(
                                                Commands.startEnd(
                                                                () -> rollerIntakeSubsystem.reverseRollers(),
                                                                () -> rollerIntakeSubsystem.stop(),
                                                                rollerIntakeSubsystem));
                // 右搖桿往上：Orbit 反轉
                driverController.axisLessThan(
                                edu.wpi.first.wpilibj.XboxController.Axis.kRightY.value, -0.5)
                                .or(() -> PS5.getRightY() < -0.5).whileTrue(
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
                // edu.wpi.first.wpilibj2.command.Commands.runOnce(() ->
                // hoodSubsystem.addAngle(1.0),
                // hoodSubsystem));
                // driverController.povDown().onTrue(
                // edu.wpi.first.wpilibj2.command.Commands.runOnce(() ->
                // hoodSubsystem.addAngle(-1.0),
                // hoodSubsystem));

                // 3. 左右鍵：持續移動 (按住時每 20ms 移動 1 度)
                driverController.povRight().or(() -> PS5.getPOV() == 90).whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(1.0), hoodSubsystem));

                driverController.povLeft().or(() -> PS5.getPOV() == 270).whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(-1.0), hoodSubsystem));
        }

        /** 只重設遙控的「前方」，不動陀螺儀與 Swerve 的位姿 (自動與視覺還要用)。 */
        private void resetDriverHeading() {
                driverHeadingOffsetRad = Math.toRadians(driveSubsystem.getHeadingDeg());
                // 除錯：按下歸零鍵時這個值應該變成當下的 Swerve/Heading Deg
                SmartDashboard.putNumber("Drive/Heading Offset Deg", Math.toDegrees(driverHeadingOffsetRad));
        }

        /**
         * PS5 扳機原始值換成 0 (放開) ~ 1 (按到底)。
         * getL2Axis() 沒有換算：接上 Driver Station 時放開是 -1、還沒按過常讀 0，
         * 直接相減會變成 ±1，車子以 turnSpeed 一直轉。負值一律當作沒按 (只用後半段行程)。
         */
        private static double ps5Trigger(double raw) {
                return MathUtil.clamp(raw, 0.0, 1.0);
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
                                driveSubsystem.getHeadingDeg(),
                                driveSubsystem.getYawRateDegPerSec());

                // 鏡頭看到 tag 時，在 Field 上畫出鏡頭算的位置 (物件 "Vision")；沒看到就停在最後位置。
                // MegaTag2 需要場地座標的陀螺儀朝向，跑過自動 (resetPose) 前改用 MegaTag1。
                visionSubsystem.getFieldPose(driveSubsystem.isFieldPoseKnown())
                                .ifPresent(pose -> driveSubsystem.getField().getObject("Vision").setPose(pose));

                // 所有模式 (自動、遙控、Disabled) 都用鏡頭 (MegaTag1) 修正里程計。
                // 還沒有場地座標時 (沒跑過自動)：停住且看到多個 tag，直接把位姿設成鏡頭位置。
                // 之後 位置：移動中也融合 (Swerve 依拍攝時間補償延遲)，轉太快時畫面糊掉不用。
                // 朝向：只在車子停住時修正 (起點擺歪的誤差)。
                // AutoAimAndShoot 的距離是直接讀 Limelight botpose，不受影響。
                List<VisionSubsystem.EstimatedRobotPose> poses = visionSubsystem.getEstimatedPoses(); // 每張畫面只回傳一次
                boolean stationary = driveSubsystem.isStationary();
                boolean turningSlowly = Math.abs(driveSubsystem
                                .getYawRateDegPerSec()) < Constants.Swerve.VisionMaxYawRateDegPerSec;
                double headingBeforeDeg = driveSubsystem.getHeadingDeg();

                for (VisionSubsystem.EstimatedRobotPose p : poses) {
                        if (!driveSubsystem.isFieldPoseKnown()) {
                                // 朝向還是開機方向，跟鏡頭差多少都可能，只能整個重設 (單一 tag 的朝向不可靠)
                                if (stationary && p.isGoodForHeading()) {
                                        driveSubsystem.resetPose(p.estimatedPose);
                                }
                                continue;
                        }
                        if (stationary && p.isGoodForHeading()) {
                                driveSubsystem.addVisionHeading(
                                                (float) p.estimatedPose.getRotation().getRadians());
                        }
                        if (turningSlowly && p.isGoodForPosition()) {
                                driveSubsystem.addVisionMeasurement(p.estimatedPose, p.timestampSeconds);
                        }
                }

                // 鏡頭改了 Swerve 朝向時，駕駛的前方跟著轉同樣角度，遙控中搖桿方向不會突然改變
                driverHeadingOffsetRad += Math.toRadians(MathUtil.inputModulus(
                                driveSubsystem.getHeadingDeg() - headingBeforeDeg, -180.0, 180.0));
        }

        /**
         * 讓 Robot.java 可以在自動/手動模式初始時強制收回 Intake
         */
        public void retractIntake() {
                intakeSubsystem.retract();
        }

        /**
         * 進遙控時呼叫：跑過自動後 Swerve 朝向是藍方原點的場地座標，駕駛的「前方」要朝對面聯盟。
         * 藍方前方 = 場地 +X (偏移 0)；紅方前方 = 場地 -X (偏移 π)。
         * 跑過自動或鏡頭設定過位姿 (Disabled 時看到多個 tag) 才是場地座標；否則朝向仍是開機方向，
         * 維持原本偏移 (開機為 0，或左 bumper 設定的值)。
         */
        public void alignDriverHeadingToAlliance() {
                if (!driveSubsystem.isFieldPoseKnown()) {
                        return;
                }
                boolean isRed = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
                driverHeadingOffsetRad = isRed ? Math.PI : 0.0;
        }
}
