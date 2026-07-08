
package frc.robot;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.concurrent.ForkJoinPool;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final RollerIntakeSubsystem rollerIntakeSubsystem = new RollerIntakeSubsystem();

        public RobotContainer() {
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.run(() -> {
                                        // 1. 處理移動 (左搖桿)
                                        double y = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
                                        double x = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);

                                        // 2. 處理旋轉 (扳機鍵：右扳機 - 左扳機)
                                        // 假設：按下右扳機 -> 右轉，按下左扳機 -> 左轉
                                        double turnLeft = driverController.getLeftTriggerAxis();
                                        double turnRight = driverController.getRightTriggerAxis();

                                        // 計算總旋轉量
                                        double rot = (turnLeft - turnRight);

                                        // 應用 Deadband 和平方曲線（讓低速更絲滑）
                                        rot = MathUtil.applyDeadband(rot, 0.1);
                                        rot = Math.copySign(rot * rot, rot);

                                        rot = rot * SwerveConstants.turnSpeed;

                                        // 指令底盤以指定的 X 軸、Y 軸平移速度以及旋轉速度移動，並且啟用『場地導向 (Field-Relative)』模式。
                                        driveSubsystem.drive(y, x, rot, true);
                                }));

                configureBindings();
        }

        private void configureBindings() {

                // shooterSubsystem
                driverController.a().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> shooterSubsystem.runshooter(),
                                                () -> shooterSubsystem.stop(),
                                                shooterSubsystem));

                // rollerIntakeSubsystem
                driverController.y().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> rollerIntakeSubsystem.runRollers(),
                                                () -> rollerIntakeSubsystem.stop(),
                                                rollerIntakeSubsystem));

                // indexer
                driverController.x().toggleOnTrue(
                                edu.wpi.first.wpilibj2.command.Commands.startEnd(
                                                () -> indexerSubsystem.runindexer(),
                                                () -> indexerSubsystem.stop(),
                                                indexerSubsystem));

                // 歸0
                driverController.leftBumper().onTrue(
                                edu.wpi.first.wpilibj2.command.Commands.runOnce(
                                                driveSubsystem::resetpigeonOdometry, driveSubsystem));

                // intakeSubsystem
                driverController.rightBumper().onTrue(
                                new InstantCommand(
                                                () -> intakeSubsystem.togglePosition(),
                                                intakeSubsystem));

                // 1. 預設命令：永遠鎖死目標角度
                hoodSubsystem.setDefaultCommand(
                                edu.wpi.first.wpilibj2.command.Commands.run(
                                                () -> hoodSubsystem.setAngle(hoodSubsystem.getTargetAngle()),
                                                hoodSubsystem));

                // 2. 上下鍵：單次移動 1 度
                driverController.povUp().onTrue(
                                edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> hoodSubsystem.addAngle(1.0),
                                                hoodSubsystem));
                driverController.povDown().onTrue(
                                edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> hoodSubsystem.addAngle(-1.0),
                                                hoodSubsystem));

                // 3. 左右鍵：持續移動 (按住時每 20ms 移動 0.5 度)
                driverController.povRight().whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(0.1), hoodSubsystem));

                driverController.povLeft().whileTrue(
                                Commands.run(() -> hoodSubsystem.addAngle(-0.1), hoodSubsystem));
        }

}
