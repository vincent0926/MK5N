package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReturnConstants;
import frc.robot.subsystems.Hopper.IndexerSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.OrbitSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.Limelight4Subsystem;

// 以 Limelight4 看到的回傳球 tag 定距離，回傳射球。
// 流程與 AutoAimAndShoot 相同（仰角首次鎖定、RPS 每週期更新、看到目標滿 1.0 s 送球），
// 差別：不轉底盤（不佔 Swerve）、不加仰角偏移、使用 ReturnConstants 的查表。
// 注意：
// - 只檢查 canReturnBall()，不檢查距離是否有效；getDistanceToTarget()
//   回傳 -1 時查表會取最近端點的值。
// - 失去目標時 hood 歸零，但 isAngleLocked 仍為 true；同一次指令內重新看到目標時
//   hood 不會再設定，會停在 0°，直到下次 initialize()。
public class ReturnBall extends Command {

    private final Limelight4Subsystem vision;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final OrbitSubsystem orbit;

    // 連續看到目標的時間；滿 1.0 s 才送球，失去目標即歸零。
    private final Timer timer = new Timer();

    // 距離 (m) → hood 仰角 (deg)。
    private final InterpolatingDoubleTreeMap distancetohood = new InterpolatingDoubleTreeMap();
    // 距離 (m) → 飛輪 RPS。
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    private double targetRPS = 0.0;
    private boolean hasValidTarget = false;
    // 鎖定的 hood 仰角 (deg)；-1 代表尚未鎖定。
    private double currentTargetAngle = -1.0;
    // true 後本次指令不再設定 hood，直到下次 initialize()。
    private boolean isAngleLocked = false;

    public ReturnBall(Limelight4Subsystem vision, HoodSubsystem hood, ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            OrbitSubsystem orbit) {
        this.vision = vision;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.orbit = orbit;

        // vision 只讀 NetworkTables，不列入 requirements
        addRequirements(hood, shooter, indexer, orbit);

        // 常數為 {距離, 值} 陣列
        for (double[] point : ReturnConstants.kReturnBalltohood) {
            distancetohood.put(point[0], point[1]);
        }
        for (double[] point : ReturnConstants.kReturnBalltorps) {
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
        hasValidTarget = vision.canReturnBall(); // tv==1 且 tid 為本聯盟回傳球 tag
        double distance = vision.getDistanceToTarget(); // m，無目標時 -1

        if (hasValidTarget) {
            // 仰角只在第一次看到目標時設定
            if (!isAngleLocked) {
                currentTargetAngle = distancetohood.get(distance);
                hood.setAngle(currentTargetAngle);
                isAngleLocked = true;
            }

            targetRPS = distanceToRPSMap.get(distance);
            shooter.setRPS(targetRPS);

            // Timer.start() 已在跑時不會重置，因此只在首次看到目標時開始計時
            timer.start();

            if (timer.get() >= 1.0) {
                indexer.runindexer();
                orbit.runorbit();
            } else {
                indexer.stop();
                orbit.stop();
            }
        } else {
            // 失去目標：重置計時並停機構；hood 歸零但未解除 isAngleLocked（見類別說明）
            timer.stop();
            timer.reset();
            shooter.stop();
            indexer.stop();
            orbit.stop();
            hood.setAngle(0);
        }
    }

    // 停止所有機構並將 hood 歸零。interrupted 不影響行為。
    @Override
    public void end(boolean interrupted) {

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
