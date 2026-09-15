package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.Optional;

/**
 * 處理影像辨識系統 (Limelight) 的子系統，負責讀取目標距離、角度與估算機器人姿態。
 */
public class VisionSubsystem extends SubsystemBase {

    /** 前方 Limelight 的網路表格 */
    private final NetworkTable frontLimelight = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.kLimelightFront);
    /** 左側 Limelight 的網路表格 */
     private final NetworkTable leftLimelight = NetworkTableInstance.getDefault()
             .getTable(VisionConstants.kLimelightLeft);
     /** 右側 Limelight 的網路表格 */
     private final NetworkTable rightLimelight = NetworkTableInstance.getDefault()
             .getTable(VisionConstants.kLimelightRight);

    /** AprilTag 場地佈局，用於計算目標真實世界座標 */
    private AprilTagFieldLayout fieldLayout;

    /**
     * 初始化視覺子系統，嘗試載入預設的 AprilTag 場地佈局。
     */
    public VisionSubsystem() {
        try {
            // 載入 2026 或官方預設 AprilTag 場地佈局
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            // 若載入失敗則回報錯誤
            DriverStation.reportError("Failed to load AprilTagFieldLayout (載入 AprilTag 場地佈局失敗)", e.getStackTrace());
        }
    }

    /**
     * 更新 MegaTag 2 所需的機器人陀螺儀角度與角速度。
     * 需由 DriveSubsystem 在 periodic 中呼叫此方法。
     *
     * @param yaw     機器人當前 Yaw (度)
     * @param yawRate 機器人當前 Yaw 轉速 (度/秒)
     */
    public void updateMegaTag2(double yaw, double yawRate) {
        // 設定機器人方向陣列 [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        double[] orientation = new double[] { yaw, yawRate, 0, 0, 0, 0 };
        // 將資訊發送至各個 Limelight，供 MegaTag 2 演算法使用
        frontLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
        leftLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
        rightLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
    }

    /**
     * 取得目前有看到目標且最可靠的距離（以 tv=1 且面積最大或最接近中心的為優先，這裡簡化為找到的距離）
     * 如果沒有看到目標，回傳 -1。
     * 
     * @return 距離目標的水平距離 (公尺)
     */
    public double getBestTargetDistance() {
        double frontDist = getDistanceFromLimelight(frontLimelight);
        double leftDist = getDistanceFromLimelight(leftLimelight);
        double rightDist = getDistanceFromLimelight(rightLimelight);

        // 簡單邏輯：選取有數值且大於 0 的最小距離，或依據實際情況調整
        double bestDist = -1;
        if (frontDist > 0)
            bestDist = frontDist;
        if (leftDist > 0 && (bestDist == -1 || leftDist < bestDist))
            bestDist = leftDist;
        if (rightDist > 0 && (bestDist == -1 || rightDist < bestDist))
            bestDist = rightDist;

        // 將最佳距離顯示於 SmartDashboard 上
        SmartDashboard.putNumber("Vision Best Target Distance", bestDist);
        return bestDist;
    }

    /**
     * 取得目前有看到對應聯盟 Hub (Speaker) AprilTag 的距離。
     * 自動根據 DriverStation 選擇紅方(4, 3)或藍方(7, 8)的 AprilTag。
     * 
     * @return 距離 (公尺)，如果沒看到則回傳 -1.0
     */
    public double getHubDistance() {
        // 判斷是否為紅方聯盟
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        // 2026 賽季 Hub ID: 紅方 = 9, 10; 藍方 = 25, 26
        int targetId1 = isRed ? 9 : 25;
        int targetId2 = isRed ? 10 : 26;

        // 檢查各個 Limelight 是否有看到指定的 AprilTag 並取得距離
        double dist = checkSpecificTagDistance(frontLimelight, targetId1, targetId2);
        if (dist > 0)
            return dist;
        dist = checkSpecificTagDistance(leftLimelight, targetId1, targetId2);
        if (dist > 0)
            return dist;
        dist = checkSpecificTagDistance(rightLimelight, targetId1, targetId2);
        return dist;
    }

    /**
     * 檢查指定的 Limelight 是否有看到特定的 AprilTag ID 並回傳距離。
     *
     * @param table 欲檢查的 Limelight 網路表格
     * @param id1 第一個目標 AprilTag ID
     * @param id2 第二個目標 AprilTag ID
     * @return 距離 (公尺)，如果沒看到則回傳 -1.0
     */
    private double checkSpecificTagDistance(NetworkTable table, int id1, int id2) {
        // 如果沒有偵測到有效目標 (tv != 1.0)，則回傳 -1.0
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return -1.0;
        }

        // 取得當前鎖定的 AprilTag ID
        long tid = (long) table.getEntry("tid").getInteger(0);
        // 如果 ID 符合目標，則計算並回傳距離
        if (tid == id1 || tid == id2) {
            return getDistanceFromLimelight(table);
        }
        return -1.0;
    }

    /**
     * 取得目前有看到目標的 Limelight 水平誤差 (tx)
     * 
     * @return 水平誤差 (度)，若沒看到則回傳 0.0
     */
    public double getBestTargetTx() {
        // 依序檢查前方、左方、右方 Limelight，回傳第一個有看到目標的 tx
        if (frontLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return frontLimelight.getEntry("tx").getDouble(0.0);
        }
        if (leftLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return leftLimelight.getEntry("tx").getDouble(0.0);
        }
        if (rightLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return rightLimelight.getEntry("tx").getDouble(0.0);
        }
        // 若都沒有看到目標則回傳 0.0
        return 0.0;
    }

    /**
     * 獲取指定 Limelight 到目標的距離
     * 在 MegaTag 2 中，我們可以使用 botpose (相對於場地) 計算與目標(Speaker)的距離
     *
     * @param table 欲讀取的 Limelight 網路表格
     * @return 目標距離 (公尺)
     */
    private double getDistanceFromLimelight(NetworkTable table) {
        // 確保有看到目標
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return -1.0;
        }

        // 使用 botpose 取得機器人在場地上的位置 (MegaTag 2)
        // 格式為: [X, Y, Z, Roll, Pitch, Yaw]
        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length >= 6) {

            // 讀取當前的目標 AprilTag ID
            long tid = (long) table.getEntry("tid").getInteger(0);
            if (tid > 0 && fieldLayout != null) {
                // 取得場地佈局中該 AprilTag 的真實 3D 姿態
                Optional<Pose3d> tagPose = fieldLayout.getTagPose((int) tid);
                if (tagPose.isPresent()) {
                    // 計算機器人位置與 AprilTag 位置之間的平面距離
                    Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                    Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();
                    return robotTrans.getDistance(tagTrans);
                }
            }
        }
        return -1.0;
    }

    /**
     * 封裝估算機器人姿態與時間戳記的資料類別
     */
    public static class EstimatedRobotPose {
        /** 估算的機器人場上姿態 (X, Y, Rotation) */
        public final Pose2d estimatedPose;
        /** 估算時的精確時間戳記 (秒) */
        public final double timestampSeconds;

        /**
         * 建立一個估算姿態的實例
         * 
         * @param estimatedPose 估算姿態
         * @param timestampSeconds 時間戳記
         */
        public EstimatedRobotPose(Pose2d estimatedPose, double timestampSeconds) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
        }
    }

    /**
     * 取得特定 Limelight 的估算位置與時間戳記
     *
     * @param table 指定的 Limelight 網路表格
     * @return 包含估算位置與時間戳記的 Optional，若沒看到目標則為 empty
     */
    private Optional<EstimatedRobotPose> getEstimatedPose(NetworkTable table) {
        // 如果沒有目標，則回傳 empty
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return Optional.empty();
        }

        // 判斷當前聯盟顏色，以讀取正確座標系的 botpose
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        String entryName = isRed ? "botpose_wpired" : "botpose_wpiblue";

        // 讀取位姿資料
        double[] botpose = table.getEntry(entryName).getDoubleArray(new double[6]);
        if (botpose.length < 6)
            return Optional.empty();

        // 建立 2D 位姿物件 (X, Y, 角度)
        Pose2d pose = new Pose2d(botpose[0], botpose[1],
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5]));

        // 計算 FPGA timestamp: 當前時間 - 總延遲 (pipeline latency + capture latency)
        double tl = table.getEntry("tl").getDouble(0.0);
        double cl = table.getEntry("cl").getDouble(0.0);
        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);

        return Optional.of(new EstimatedRobotPose(pose, timestamp));
    }

    /**
     * 取得所有看到目標的 Limelight 估算位置
     * 
     * @return 包含各個視角估算姿態的列表
     */
    public java.util.List<EstimatedRobotPose> getEstimatedPoses() {
        java.util.List<EstimatedRobotPose> poses = new java.util.ArrayList<>();
        // 收集前方、左方、右方 Limelight 的有效位姿估算
        getEstimatedPose(frontLimelight).ifPresent(poses::add);
        getEstimatedPose(leftLimelight).ifPresent(poses::add);
        getEstimatedPose(rightLimelight).ifPresent(poses::add);
        return poses;
    }

    /**
     * 定期執行的任務，用來更新 Dashboard 資料等
     */
    @Override
    public void periodic() {
        getBestTargetDistance(); // 更新 Dashboard 上的最佳距離
    }
}