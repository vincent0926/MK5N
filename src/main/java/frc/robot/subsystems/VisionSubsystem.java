package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
     * 檢查指定的 AprilTag ID 是否屬於當前聯盟的 HUB。
     *
     * @param tid 欲檢查的 AprilTag ID
     * @param isRed 是否為紅方聯盟
     * @return 若屬於當前聯盟 HUB 則為 true
     */
    public boolean isHubTag(int tid, boolean isRed) {
        int[] hubTags = isRed ? VisionConstants.kRedHubTagIds : VisionConstants.kBlueHubTagIds;
        for (int tagId : hubTags) {
            if (tagId == tid) {
                return true;
            }
        }
        return false;
    }

    /**
     * 根據機器人目前估測姿態 (Pose) 計算到對應聯盟 HUB 中心的水平距離 (公尺)。
     *
     * @param robotPose 機器人當前姿態 (融合輪速里程計與視覺定位)
     * @return 距離 (公尺)
     */
    public double getHubDistance(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;
        return robotPose.getTranslation().getDistance(hubCenter);
    }

    /**
     * 取得目前有看到對應聯盟 Hub AprilTag 時，機器人到 HUB 中心的水平距離。
     * 透過 Limelight 的 botpose_wpiblue 計算到 HUB 中心的平面距離。
     * 
     * @return 距離 (公尺)，如果沒看到則回傳 -1.0
     */
    public double getHubDistance() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;

        // 檢查各個 Limelight 是否有看到 HUB AprilTag 並計算至 HUB 中心距離
        double dist = getDistanceToHubFromLimelight(frontLimelight, hubCenter, isRed);
        if (dist > 0)
            return dist;
        dist = getDistanceToHubFromLimelight(leftLimelight, hubCenter, isRed);
        if (dist > 0)
            return dist;
        dist = getDistanceToHubFromLimelight(rightLimelight, hubCenter, isRed);
        return dist;
    }

    /**
     * 檢查指定的 Limelight 是否有看到 HUB AprilTag，若有則利用其 botpose_wpiblue 計算至 HUB 中心的距離。
     *
     * @param table 欲檢查的 Limelight 網路表格
     * @param hubCenter 當前聯盟 HUB 中心座標
     * @param isRed 是否為紅方聯盟
     * @return 距離 (公尺)，如果沒看到則回傳 -1.0
     */
    private double getDistanceToHubFromLimelight(NetworkTable table, Translation2d hubCenter, boolean isRed) {
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return -1.0;
        }

        long tid = (long) table.getEntry("tid").getInteger(0);
        if (isHubTag((int) tid, isRed)) {
            double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            if (botpose.length >= 2 && (botpose[0] != 0.0 || botpose[1] != 0.0)) {
                Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                return robotTrans.getDistance(hubCenter);
            }
        }
        return -1.0;
    }

    /**
     * 根據機器人目前估測姿態 (Pose) 計算車頭 (Shooter) 對準 HUB 中心的水平角度誤差 (tx)。
     *
     * @param robotPose 機器人姿態 (融合輪速里程計與視覺定位)
     * @return 水平誤差 (度)，HUB 在右為正、在左為負 (符合 Limelight tx 慣例)
     */
    public double getHubTx(Pose2d robotPose) {
        if (!hasHubTarget()) {
            return 0.0;
        }

        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;

        Translation2d robotTrans = robotPose.getTranslation();
        double dx = hubCenter.getX() - robotTrans.getX();
        double dy = hubCenter.getY() - robotTrans.getY();
        // 場地座標系中機器人指向 HUB 中心的絕對角度 (度, -180 ~ 180)
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // 機器人當前朝向角度 (度, -180 ~ 180)
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        // 計算角度差：目標角度 - 當前朝向 (正值代表目標在當前車頭左側 CCW)
        double angleErrorDeg = MathUtil.inputModulus(targetAngleDeg - robotHeadingDeg, -180.0, 180.0);

        // Limelight tx 慣例：目標在畫面右側為正 (CW)，在畫面左側為負 (CCW)
        // 故 tx = -angleErrorDeg
        return -angleErrorDeg;
    }

    /**
     * 取得前方 Limelight (frontLimelight) 看到對應聯盟 Hub AprilTag 時，對準 HUB 中心的水平誤差 (tx)。
     * 優先以 Limelight 回傳之 botpose 姿態計算至 HUB 中心之幾何角度；若無法取得 botpose 則退回 AprilTag tx。
     *
     * @return 水平誤差 (度)，若未辨識到 HUB AprilTag 則回傳 0.0
     */
    public double getHubTx() {
        if (!hasHubTarget()) {
            return 0.0;
        }

        double[] botpose = frontLimelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length >= 6 && (botpose[0] != 0.0 || botpose[1] != 0.0)) {
            Pose2d robotPose = new Pose2d(botpose[0], botpose[1],
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5]));
            return getHubTx(robotPose);
        }

        // 若無 botpose，備用退回原始 AprilTag tx
        return frontLimelight.getEntry("tx").getDouble(0.0);
    }

    /**
     * 檢查前方 Limelight 是否成功鎖定對應聯盟的 Hub AprilTag。
     * 
     * @return 若前方相機有看到合法的 Hub AprilTag 則回傳 true，否則回傳 false
     */
    public boolean hasHubTarget() {
        if (frontLimelight.getEntry("tv").getDouble(0) != 1.0) {
            return false;
        }

        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        long tid = (long) frontLimelight.getEntry("tid").getInteger(0);
        return isHubTag((int) tid, isRed);
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