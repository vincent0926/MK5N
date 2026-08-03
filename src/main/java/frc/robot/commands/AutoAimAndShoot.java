package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * 自動瞄準與射擊指令
 * 負責透過視覺系統獲取目標距離與仰角，並自動控制底盤對準、調整發射器仰角、
 * 啟動飛輪，並在所有條件達成時啟動送球機構發射。
 */
public class AutoAimAndShoot extends Command {
    /** 視覺系統子系統 */
    private final VisionSubsystem vision;
    /** 發射器仰角子系統 */
    private final HoodSubsystem hood;
    /** 發射器飛輪子系統 */
    private final ShooterSubsystem shooter;
    /** 送球機構子系統 */
    private final IndexerSubsystem indexer;
    /** 底盤子系統 */
    private final DriveSubsystem drive;

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

    /**
     * 建構子：初始化自動瞄準與射擊指令
     * 
     * @param vision  視覺子系統
     * @param hood    仰角控制子系統
     * @param shooter 飛輪發射子系統
     * @param indexer 送球機構子系統
     * @param drive   底盤驅動子系統
     */
    public AutoAimAndShoot(VisionSubsystem vision, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer, DriveSubsystem drive) {
        this.vision = vision;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;

        addRequirements(hood, shooter, indexer, drive);

        // 初始化查表資料
        for (double[] point : AimConstants.kDistanceToAngleMap) {
            distanceToAngleMap.put(point[0], point[1]);
        }
        for (double[] point : AimConstants.kDistanceToRPSMap) {
            distanceToRPSMap.put(point[0], point[1]);
        }
    }

    /**
     * 指令初始化時執行：重置目標狀態與仰角鎖定
     */
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
            double rotationSpeed = -tx * 0.05;
            
            // 限制最大旋轉速度，避免太猛烈
            if (rotationSpeed > 2.0) rotationSpeed = 2.0;
            if (rotationSpeed < -2.0) rotationSpeed = -2.0;
            
            // 如果誤差很小 (例如小於 1.5 度)，代表已經對準
            boolean isAligned = Math.abs(tx) < 1.5;
            if (isAligned) {
                rotationSpeed = 0.0;
                drive.setX(); // 對準後設定為 X 陣型以防撞
            } else {
                drive.drive(0, 0, rotationSpeed, false); // 未對準時繼續轉向
            }

            // 發射邏輯：當仰角到位、飛輪轉速足夠且底盤已對準時，啟動 indexer 送球
            if (hood.isAtAngle() && shooter.isAtSpeed(targetRPS) && isAligned) {
                indexer.runindexer();
            } else {
                indexer.stop(); 
            }
        } else {
            // 沒有看到目標，維持原位 (可視需求打 X 或停止)
            hasValidTarget = false;
            drive.setX(); // 沒看到目標也可以鎖定防撞
            shooter.stop();
            indexer.stop();
        }
    }

    /**
     * 指令結束時執行：停止所有相關機構
     *
     * @param interrupted 指令是否被中斷
     */
    @Override
    public void end(boolean interrupted) {
        // 指令結束時停止底盤、發射器與送球機構
        drive.drive(0, 0, 0, false);
        shooter.stop();
        indexer.stop();
        
        // 使用者要求釘住仰角，所以在指令結束（放開 A 鍵）時，不要把它歸零
        // hood.setAngle(0);
        
        currentTargetAngle = -1.0;
    }

    /**
     * 檢查指令是否完成
     *
     * @return 永遠回傳 false，此指令綁定按鈕 (whileTrue)，由使用者放開按鍵來結束
     */
    @Override
    public boolean isFinished() {
        return false; // 這個指令會綁定按鈕 (whileTrue)，由使用者放開按鍵來結束
    }
}
