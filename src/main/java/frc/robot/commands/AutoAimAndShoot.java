package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.Hopper.IndexerSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.OrbitSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Vision.VisionSubsystem;

// 以 HUB AprilTag 自動瞄準並射擊。
// 每週期：依 HUB 距離查表得 hood 仰角與飛輪 RPS，用 tx 做 P 控制原地轉向；
// 看到目標滿 1.0 s 後啟動 Indexer/Orbit 送球。
// 注意：
// - 仰角只在首次看到目標時設定一次（之後不隨距離更新）；RPS 每週期依距離更新。
// - 送球只看計時，不檢查 isAligned、飛輪轉速或 hood 是否到位。
// - 失去目標時重置計時、停飛輪，但不解除仰角鎖定、不歸零 hood。
// 從不自行結束；由 ShootSelector 呼叫生命週期方法。
public class AutoAimAndShoot extends Command {

    private final VisionSubsystem vision;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final Swerve drive;
    private final OrbitSubsystem orbit;

    // 連續看到目標的時間；滿 1.0 s 才送球，失去目標即歸零。
    private final Timer timer = new Timer();

    // 距離 (m) → hood 仰角 (deg)，資料來自 AimConstants.kDistanceToAngleMap。
    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
    // 距離 (m) → 飛輪 RPS，資料來自 AimConstants.kDistanceToRPSMap。
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    private double targetRPS = 0.0;
    // 只寫不讀，目前僅供除錯。
    private boolean hasValidTarget = false;

    // 鎖定的 hood 仰角 (deg)；-1 代表尚未鎖定。
    private double currentTargetAngle = -1.0;
    // true 後本次指令不再更新仰角，直到下次 initialize()。
    private boolean isAngleLocked = false;

    public AutoAimAndShoot(VisionSubsystem vision, HoodSubsystem hood, ShooterSubsystem shooter,
            IndexerSubsystem indexer, Swerve drive, OrbitSubsystem orbit) {
        this.vision = vision;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.orbit = orbit;

        addRequirements(hood, shooter, indexer, drive, orbit);

        // 常數為 {距離, 值} 陣列
        for (double[] point : AimConstants.kDistanceToAngleMap) {
            distanceToAngleMap.put(point[0], point[1]);
        }
        for (double[] point : AimConstants.kDistanceToRPSMap) {
            distanceToRPSMap.put(point[0], point[1]);
        }
    }

    // 清除仰角鎖定與計時；ShootSelector 每次切進此模式都會呼叫。
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
        double distance = vision.getHubDistance(); // m，無效時 <= 0
        double tx = vision.getHubTx(); // deg，目標在畫面右側為正
        boolean hasHub = vision.hasHubTarget();

        if (hasHub && distance > 0) {
            hasValidTarget = true;
            double targetAngle = distanceToAngleMap.get(distance) + AimConstants.kElevationOffset;
            targetRPS = distanceToRPSMap.get(distance);

            // 仰角只在第一次看到目標時設定
            if (!isAngleLocked) {
                currentTargetAngle = targetAngle;
                hood.setAngle(currentTargetAngle);
                isAngleLocked = true;
            }

            shooter.setRPS(targetRPS);

            // 轉向 P 控制：omega (rad/s) = -kP * tx (deg)。tx > 0 需順時針 (負 omega)
            double kP = 0.02;
            double rotationSpeed = -tx * kP;

            // 限幅 ±1 rad/s
            if (rotationSpeed > 1.0)
                rotationSpeed = 1.0;
            if (rotationSpeed < -1.0)
                rotationSpeed = -1.0;

            // 容許誤差 1°；kP=0.02 時誤差 1° 只對應 0.02 rad/s，在此停止避免在死區附近蠕動
            boolean isAligned = Math.abs(tx) < 1.0;
            if (isAligned) {

                rotationSpeed = 0.0;
                drive.stop();
            } else {
                // 平移為 0，fieldRelative 與否結果相同
                drive.setFieldVelocity(0.0f, 0.0f, (float) rotationSpeed, false);
            }

            // Timer.start() 已在跑時不會重置，因此只在首次看到目標時開始計時
            timer.start();

            if (timer.get() >= 2.5) {
                indexer.runindexer();
                orbit.runorbit();
            } else {
                indexer.stop();
                orbit.stop();
            }
        } else {
            // 失去目標：重置計時，避免重新看到時立即送球。hood 維持鎖定角度
            hasValidTarget = false;
            timer.stop();
            timer.reset();
            drive.stop();
            shooter.stop();
            indexer.stop();
            orbit.stop();
        }
    }

    // 停止所有機構並將 hood 歸零。interrupted 不影響行為。
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        shooter.stop();
        indexer.stop();
        orbit.stop();

        timer.stop();
        timer.reset();

        hood.setAngle(0);

        currentTargetAngle = -1.0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

