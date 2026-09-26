package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Limelight4Subsystem;
import frc.robot.subsystems.OrbitSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReturnBall extends Command {

    private final Limelight4Subsystem vision;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final OrbitSubsystem orbit;

    private final Timer timer = new Timer();

    private final InterpolatingDoubleTreeMap distancetohood = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    private double targetRPS = 0.0;
    private boolean hasValidTarget = false;
    private double currentTargetAngle = -1.0;
    private boolean isAngleLocked = false;

    public ReturnBall(Limelight4Subsystem vision, HoodSubsystem hood, ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            OrbitSubsystem orbit) {
        this.vision = vision;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.orbit = orbit;

        addRequirements(hood, shooter, indexer, orbit);

        for (double[] point : AimConstants.kDistanceToAngleMap) {
            distancetohood.put(point[0], point[1]);
        }
        for (double[] point : AimConstants.kDistanceToRPSMap) {
            distanceToRPSMap.put(point[0], point[1]);
        }

    }

    @Override
    public void initialize() {
        hasValidTarget = false;
        targetRPS = 0.0;
        currentTargetAngle = -1.0;
        isAngleLocked = false;

        timer.stop();
        timer.reset();
    }

  @Override
public void execute() {
    hasValidTarget = vision.canReturnBall();          
    double distance = vision.getDistanceToTarget();    

    if (hasValidTarget) {
        if (!isAngleLocked) {
            currentTargetAngle = distancetohood.get(distance);
            hood.setAngle(currentTargetAngle);
            isAngleLocked = true; // 拍照鎖死，直到下次重新按按鈕前不再改變
        }

        targetRPS = distanceToRPSMap.get(distance);
        shooter.setRPS(targetRPS); 

        timer.start();

        if (timer.get() >= 1.0) {
            indexer.runindexer();
            orbit.runorbit();
        } else {
            indexer.stop();
            orbit.stop();
        }
    } else {
        timer.stop();
        timer.reset();
        shooter.stop();
        indexer.stop();
        orbit.stop();
    }
}
    

    // 指令結束時執行：停止所有相關機構
    //
    // @param interrupted 指令是否被中斷

    @Override
    public void end(boolean interrupted) {
        // 指令結束時停止底盤、發射器與送球機構
        shooter.stop();
        indexer.stop();
        orbit.stop();

        // 重置並停止計時器
        timer.stop();
        timer.reset();

        // 指令結束（放開 A 鍵）時，把它歸零
        hood.setAngle(0);

        currentTargetAngle = -1.0;
    }

    // 檢查指令是否完成
    @Override
    public boolean isFinished() {
        return false;
    }
}
