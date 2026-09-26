package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;

public class Limelight4Subsystem extends SubsystemBase {

    // 取得 Limelight 4 的 NetworkTable 參照
    // "limelight" 是這顆 Limelight 在網頁介面中設定的名稱
    // 所有 Limelight 的數據（目標 ID、是否有目標、機器人姿態等）都會透過這個 table 存取
    private final NetworkTable limelight4Table = NetworkTableInstance.getDefault().getTable("limelight");

    // WPILib 提供的 AprilTag 場地佈局資料
    // 包含本賽季所有 AprilTag 的 ID 以及它們在場地上的精確 3D 座標
    // 用於 getDistanceToTarget() 中查詢標籤的真實世界座標來計算距離
    private AprilTagFieldLayout fieldLayout;

    public Limelight4Subsystem() {
        try {
            // 載入 WPILib 官方提供的當前賽季預設場地佈局
            // 這份資料會告訴我們每個 AprilTag 編號對應的場地座標
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            // 載入失敗時回報錯誤（不會導致機器人程式崩潰）
            // 但 getDistanceToTarget() 將無法正常運作（fieldLayout 會是 null）
            DriverStation.reportError("Failed to load AprilTagFieldLayout for Limelight4", e.getStackTrace());
        }
    }

    /**
     * 判斷是否滿足回傳球條件：
     * - 藍方聯盟且看到 AprilTag 17 或 22
     * - 紅方聯盟且看到 AprilTag 6 或 1
     */
    public boolean canReturnBall() {
        // 第一關：檢查 Limelight 是否有看到任何 AprilTag
        // "tv" (Target Valid)：1.0 = 有看到目標，0.0 = 沒看到
        // 如果沒看到任何目標，直接回傳 false，後面的 tid 也不可信
        if (limelight4Table.getEntry("tv").getDouble(0.0) != 1.0) {
            return false;
        }

        // 第二關：讀取當前鎖定的 AprilTag 編號
        // "tid" (Target ID)：Limelight 當前追蹤到的 AprilTag 的編號
        long tid = (long) limelight4Table.getEntry("tid").getInteger(0);
        // 第三關：判斷我方是哪個聯盟
        // DriverStation.getAlliance() 會從 FMS 或 Driver Station 取得當前聯盟顏色
        // isPresent() 確認資料已取得（剛開機時可能還沒拿到）
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // 第四關：根據聯盟顏色檢查 AprilTag 編號是否為中場回傳球的目標
        if (isRed) {
            // 紅方聯盟：看到 6 號或 1 號 AprilTag 時，允許回傳球
            return tid == 6 || tid == 1;
        } else {
            // 藍方聯盟：看到 17 號或 22 號 AprilTag 時，允許回傳球
            return tid == 17 || tid == 22;
        }
    }

    /**
     * 取得當前距離目標 AprilTag 的平面距離（單位：公尺）。
     * 此計算方式與 VisionSubsystem 一致，使用機器人場地座標與 AprilTag 場地座標進行距離計算。
     * 
     * @return 距離目標的公尺數，若無目標或無法計算則回傳 -1.0
     */
    public double getDistanceToTarget() {
        // 1. 檢查是否有看到目標
        // Limelight 的 "tv" (Target Valid) 數值若是 1.0，代表鏡頭當前有鎖定到 AprilTag。
        // 如果沒看到任何目標，就直接回傳 -1.0 表示無效距離。
        if (limelight4Table.getEntry("tv").getDouble(0.0) != 1.0) {
            return -1.0;
        }

        // 2. 取得機器人的場地座標
        // "botpose_wpiblue" 會回傳一個陣列，包含機器人在場地上的位置 [X, Y, Z, 翻滾角, 俯仰角, 偏航角]。
        // 這個座標是以「藍方聯盟牆左下角」為原點的絕對座標。
        double[] botpose = limelight4Table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        if (botpose.length >= 6) {
            // 3. 取得當前看到的 AprilTag ID
            long tid = (long) limelight4Table.getEntry("tid").getInteger(0);

            // 如果 ID 大於 0 且場地圖資 (fieldLayout) 已經成功載入
            if (tid > 0 && fieldLayout != null) {

                // 4. 查出該 AprilTag 在場地上的真實 3D 座標
                Optional<Pose3d> tagPose = fieldLayout.getTagPose((int) tid);

                // 如果這張標籤確實存在於官方的場地佈局中
                if (tagPose.isPresent()) {

                    // 5. 抓出機器人與標籤的 2D 平面座標 (X, Y)
                    // botpose[0] 是機器人的 X，botpose[1] 是機器人的 Y (忽略高度 Z)
                    Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                    // tagPose.get().getTranslation().toTranslation2d() 是將標籤的 3D 座標轉為 2D 平面座標
                    Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();

                    // 6. 計算並回傳兩點之間的直線距離（單位：公尺）
                    return robotTrans.getDistance(tagTrans);
                }
            }
        }

        // 如果中間有任何步驟失敗（例如陣列長度不對、找不到該標籤），都回傳 -1.0
        return -1.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight4 Can Return Ball", canReturnBall());
        SmartDashboard.putNumber("Limelight4 Target ID", limelight4Table.getEntry("tid").getInteger(0));
        SmartDashboard.putNumber("Limelight4 Target Distance", getDistanceToTarget());
    }
}
