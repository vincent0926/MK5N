package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Limelight4Subsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReturnBall extends Command {
    private final Limelight4Subsystem limelight4Subsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    public ReturnBall(Limelight4Subsystem limelight4Subsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        this.limelight4Subsystem = limelight4Subsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(limelight4Subsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem);

        // 讀取設定檔中的距離對應角度/轉速表
        for (double[] point : AimConstants.kDistanceToAngleMap) {
            distanceToAngleMap.put(point[0], point[1]);
        }
        for (double[] point : AimConstants.kDistanceToRPSMap) {
            distanceToRPSMap.put(point[0], point[1]);
        }
    }

    @Override
    public void initialize() {
        System.out.println("ReturnBall Command Started");
    }

    @Override
    public void execute() {
        double distance = limelight4Subsystem.getDistanceToTarget();

        if (distance > 0) {
            // 透過查表內插法取得目標仰角與目標轉速
            double targetAngle = distanceToAngleMap.get(distance);
            double targetRPS = distanceToRPSMap.get(distance);

            // 設置機構
            hoodSubsystem.setAngle(targetAngle);
            shooterSubsystem.setRPS(targetRPS);

            // 當馬達達到目標轉速，且 Hood 到達目標角度時，啟動 indexer 輸彈
            if (shooterSubsystem.isAtSpeed(targetRPS) && hoodSubsystem.isAtAngle()) {
                indexerSubsystem.runindexer();
            } else {
                indexerSubsystem.stop();
            }
        } else {
            // 如果沒看到距離 (或算不出來)，停止射擊與進件機構，確保安全
            shooterSubsystem.stop();
            indexerSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReturnBall Command Ended");
        shooterSubsystem.stop();
        indexerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // 持續執行，直到玩家放開按鍵或觸發中斷條件
    }
}
