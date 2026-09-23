package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OrbitSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * 自動瞄準與射擊指令
 * 負責透過視覺系統獲取目標距離與仰角，並自動控制底盤對準、調整發射器仰角、
 * 啟動飛輪，並在所有條件達成時啟動送球機構發射。
 */
public class AutoAimAndShoot extends Command {

    private final VisionSubsystem vision;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final DriveSubsystem drive;
    private final OrbitSubsystem orbit;

    // 距離與仰角的內插法查表
    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
    // 距離與飛輪轉速(RPS)的內插法查表
    private final InterpolatingDoubleTreeMap distanceToRPSMap = new InterpolatingDoubleTreeMap();

    /** 目標飛輪轉速 */
    private double targetRPS = 0.0;
    /** 是否有有效的目標 */
    private boolean hasValidTarget = false;

    // 紀錄目前的目標仰角 (快門式瞄準)
    private double currentTargetAngle = -1.0;
    /** 是否已鎖定仰角 */
    private boolean isAngleLocked = false;

    public AutoAimAndShoot(VisionSubsystem vision, HoodSubsystem hood, ShooterSubsystem shooter,
            IndexerSubsystem indexer, DriveSubsystem drive, OrbitSubsystem orbit) {
        this.vision = vision;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.orbit = orbit;

        addRequirements(hood, shooter, indexer, drive, orbit);


        // 初始化查表資料
        for (double[] point : AimConstants.kDistanceToAngleMap) {
            distanceToAngleMap.put(point[0], point[1]); // 距離 → 仰角
        }
        for (double[] point : AimConstants.kDistanceToRPSMap) {
            distanceToRPSMap.put(point[0], point[1]); // 距離 → 轉速
        }
    }

    // 每次按鈕按下時，都會重置
    @Override
    public void initialize() {
        hasValidTarget = false;
        targetRPS = 0.0;
        currentTargetAngle = -1.0;
        isAngleLocked = false; // 重置鎖定狀態，每次按下按鈕都可以重新瞄準
    }

    /**
     * 指令執行期間持續呼叫：獲取視覺資料並控制各個機構
     */
    @Override
    public void execute() {
        // 獲取最佳目標距離
        double distance = vision.getBestTargetDistance();
        // 獲取最佳目標水平偏移角度
        double tx = vision.getBestTargetTx();

        if (distance > 0) {
            hasValidTarget = true;
            // 透過查表取得目標仰角與轉速，並加上預設的仰角增加度數
            double targetAngle = distanceToAngleMap.get(distance) + AimConstants.kElevationOffset;
            targetRPS = distanceToRPSMap.get(distance);

            // 選項二：按下按鈕後，只要一看到目標就馬上更新並「鎖定仰角」
            if (!isAngleLocked) {
                currentTargetAngle = targetAngle;
                hood.setAngle(currentTargetAngle);
                isAngleLocked = true; // 拍照鎖死，直到下次重新按按鈕前不再改變
            }

            shooter.setRPS(targetRPS); // 啟動飛輪

            // 控制底盤水平對準 (P 控制器)
            // 將 tx (度) 轉換為旋轉角速度 (rad/s)，這裡的 0.05 是比例增益 (Kp)，可依實際旋轉速度調整
            double rotationSpeed = -tx * 0.08;

            // 限制最大旋轉速度，避免太猛烈
            if (rotationSpeed > 3.0)
                rotationSpeed = 3.0;
            if (rotationSpeed < -3.0)
                rotationSpeed = -3.0;

            // 如果誤差很小 (例如小於 1.0 度)，代表已經對準
            boolean isAligned = Math.abs(tx) < 1.0;
            if (isAligned) {
                rotationSpeed = 0.0;
                drive.setX(); // 對準後設定為 X 陣型以防撞
            } else {
                drive.drive(0, 0, rotationSpeed, false); // 未對準時繼續轉向
            }

            // 發射邏輯：當仰角到位、飛輪轉速足夠且底盤已對準時，啟動 indexer 送球
            if (hood.isAtAngle() && shooter.isAtSpeed(targetRPS) && isAligned) {
                indexer.runindexer();
                orbit.runorbit();
            } else {
                indexer.stop();
                orbit.stop();
            }
        } else {
            // 沒有看到目標，維持原位 (可視需求打 X 或停止)
            hasValidTarget = false;
            drive.setX(); // 沒看到目標也可以鎖定防撞
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
        drive.drive(0, 0, 0, false);
        shooter.stop();
        indexer.stop();
        orbit.stop();

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

// --------------------------------------------------------------------------------
// 上面看起來很猛，待測試
// --------------------------------------------------------------------------------

// package frc.robot.commands;

// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AimConstants;
// import frc.robot.subsystems.HoodSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class AutoAimAndShoot extends Command {

// private final HoodSubsystem hoodSubsystem;
// private final VisionSubsystem visionSubsystem;
// private final InterpolatingDoubleTreeMap distanceToAngleMap = new
// InterpolatingDoubleTreeMap();

// private boolean isTargetSet = false;

// public AutoAimAndShoot(HoodSubsystem hoodSubsystem, VisionSubsystem
// visionSubsystem) {
// this.hoodSubsystem = hoodSubsystem;
// this.visionSubsystem = visionSubsystem;

// // 引入你現有的距離-仰角對照表
// for (double[] point : AimConstants.kDistanceToAngleMap) {
// distanceToAngleMap.put(point[0], point[1]);
// }

// // 宣告需求子系統
// addRequirements(hoodSubsystem);
// }

// private int attempts = 0;

// @Override
// public void initialize() {
// // 每次按下 A 鍵重置鎖定狀態與嘗試計數
// isTargetSet = false;
// attempts = 0;
// }

// @Override
// public void execute() {
// attempts++;
// // 若尚未設定過仰角，則持續向 Limelight 詢問距離
// if (!isTargetSet) {
// double distance = visionSubsystem.getHubDistance();

// // 當有看到 Hub 的 AprilTag 時（距離 > 0）
// if (distance > 0) {
// // 透過插值表計算角度
// double targetAngle = distanceToAngleMap.get(distance);

// // 設定仰角
// hoodSubsystem.setAngle(targetAngle);

// // 標記已完成設定
// isTargetSet = true;
// }
// }
// }

// @Override
// public boolean isFinished() {
// // 一旦設定完仰角，或嘗試超過 50 次 (約 1 秒沒看到目標)，指令結束，防止鎖死 Hood 手動控制
// return isTargetSet || attempts > 50;
// }
// }
