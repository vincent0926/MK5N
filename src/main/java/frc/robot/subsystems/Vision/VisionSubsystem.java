package frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    // 目前只有前方一顆 Limelight；NT 讀寫都透過 LimelightHelpers
    private final String frontLimelight = VisionConstants.kLimelightFront;

    private AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout (載入 AprilTag 場地佈局失敗)", e.getStackTrace());
        }
    }

    public void updateMegaTag2(double yaw, double yawRate) {
        LimelightHelpers.SetRobotOrientation(frontLimelight, yaw, yawRate, 0, 0, 0, 0);
    }

    public double getBestTargetDistance() {
        double frontDist = getDistanceFromLimelight(frontLimelight);

        double bestDist = -1;
        if (frontDist > 0)
            bestDist = frontDist;

        SmartDashboard.putNumber("Vision Best Target Distance", bestDist);
        return bestDist;
    }

    public boolean isHubTag(int tid, boolean isRed) {
        int[] hubTags = isRed ? VisionConstants.kRedHubTagIds : VisionConstants.kBlueHubTagIds;
        for (int tagId : hubTags) {
            if (tagId == tid) {
                return true;
            }
        }
        return false;
    }

    public double getHubDistance(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;
        return robotPose.getTranslation().getDistance(hubCenter);
    }

    public double getHubDistance() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;

        String limelight = getActiveHubCamera();
        if (limelight == null) {
            return -1.0;
        }

        Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
        if (!isZeroPose(robotPose)) {
            return robotPose.getTranslation().getDistance(hubCenter);
        }
        return -1.0;
    }

    /**
     * 依優先順序（前 → 左 → 右）尋找目前看得到 HUB AprilTag 的 Limelight。
     * 找不到則回傳 null，找到則回傳 Limelight 名稱。
     */
    private String getActiveHubCamera() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        for (String limelight : new String[] { frontLimelight }) {
            if (!LimelightHelpers.getTV(limelight)) {
                continue;
            }
            int tid = (int) LimelightHelpers.getFiducialID(limelight);
            if (isHubTag(tid, isRed)) {
                return limelight;
            }
        }
        return null;
    }

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

        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double angleErrorDeg = MathUtil.inputModulus(targetAngleDeg - robotHeadingDeg, -180.0, 180.0);

        return -angleErrorDeg;
    }

    public double getHubTx() {
        String limelight = getActiveHubCamera();
        if (limelight == null) {
            return 0.0;
        }

        Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
        if (!isZeroPose(robotPose)) {
            return getHubTx(robotPose);
        }

        return LimelightHelpers.getTX(limelight);
    }

    public boolean hasHubTarget() {
        return getActiveHubCamera() != null;
    }

    public double getBestTargetTx() {
        // 依優先順序檢查各 Limelight：前方 → 左側 → 右側
        if (LimelightHelpers.getTV(frontLimelight)) {
            return LimelightHelpers.getTX(frontLimelight);
        }

        return 0.0;
    }

    private double getDistanceFromLimelight(String limelight) {
        if (!LimelightHelpers.getTV(limelight)) {
            return -1.0;
        }

        int tid = (int) LimelightHelpers.getFiducialID(limelight);
        if (tid > 0 && fieldLayout != null) {

            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tid);
            if (tagPose.isPresent()) {

                Translation2d robotTrans = LimelightHelpers.getBotPose2d_wpiBlue(limelight).getTranslation();
                Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();
                return robotTrans.getDistance(tagTrans);
            }
        }
        return -1.0;
    }

    /** Limelight 沒算出位置時 botpose 是全零 (LimelightHelpers 資料不足時也回傳原點)。 */
    private static boolean isZeroPose(Pose2d pose) {
        return pose.getX() == 0.0 && pose.getY() == 0.0;
    }

    public static class EstimatedRobotPose {

        public final Pose2d estimatedPose;

        public final double timestampSeconds;

        /** 這張畫面用到的 tag 數 (botpose 第 7 格)；舊格式沒有這欄時為 0。 */
        public final int tagCount;

        /** 到 tag 的平均距離 (公尺，botpose 第 9 格)；舊格式沒有這欄時為 0。 */
        public final double avgTagDistM;

        public EstimatedRobotPose(Pose2d estimatedPose, double timestampSeconds) {
            this(estimatedPose, timestampSeconds, 0, 0.0);
        }

        public EstimatedRobotPose(Pose2d estimatedPose, double timestampSeconds, int tagCount, double avgTagDistM) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
            this.tagCount = tagCount;
            this.avgTagDistM = avgTagDistM;
        }

        /** 位置可用：多 tag，或單一 tag 夠近。 */
        public boolean isGoodForPosition() {
            return tagCount >= 2 || avgTagDistM <= VisionConstants.kSingleTagMaxDistM;
        }

        /** 朝向可用：只相信多 tag 且不太遠的結果 (單一 tag 的 MegaTag1 朝向誤差大)。 */
        public boolean isGoodForHeading() {
            return tagCount >= VisionConstants.kHeadingMinTagCount
                    && avgTagDistM <= VisionConstants.kHeadingMaxTagDistM;
        }
    }

    // 上一張用過的畫面的拍攝時間；Limelight 更新比 50 Hz 慢，同一張畫面不重複使用
    private double lastFrameTimestamp = Double.NaN;

    private Optional<EstimatedRobotPose> getEstimatedPose(String limelight) {
        if (!LimelightHelpers.getTV(limelight)) {
            return Optional.empty();
        }

        // PathPlanner 與 Swerve 里程計永遠用藍方原點 (紅方路線由 AutoBuilder 翻轉)，所以兩隊都讀 wpiblue (MegaTag1)。
        // LimelightHelpers 算好拍攝時間 = roboRIO 收到這張畫面的 NT 時間 - Limelight 總延遲 (botpose 第 6 格)；
        // NT 時間與 FPGA 時間同一個基準。
        // 不用 LimelightHelpers.validPoseEstimate：它要求每個 tag 的原始資料 (rawFiducials)，陣列長度不符時是空的。
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        if (estimate.timestampSeconds == 0.0 && isZeroPose(estimate.pose)) {
            return Optional.empty(); // NT 上沒有 botpose
        }

        if (estimate.timestampSeconds == lastFrameTimestamp) {
            return Optional.empty();
        }
        lastFrameTimestamp = estimate.timestampSeconds;

        return Optional.of(new EstimatedRobotPose(estimate.pose, estimate.timestampSeconds,
                estimate.tagCount, estimate.avgTagDist));
    }

    /**
     * 鏡頭看到 tag 時算出的機器人場地位置 (藍方原點)，只用來顯示，不融合進里程計。
     *
     * @param useMegaTag2 true 讀 MegaTag2 (botpose_orb_wpiblue，朝向用 updateMegaTag2 送的陀螺儀角度，
     *                    所以陀螺儀必須已經是場地座標)；false 讀 MegaTag1 (botpose_wpiblue)。
     * @return 沒看到 tag 或資料無效時回傳 empty。
     */
    public Optional<Pose2d> getFieldPose(boolean useMegaTag2) {
        if (!LimelightHelpers.getTV(frontLimelight)) {
            return Optional.empty();
        }

        Pose2d pose = useMegaTag2
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(frontLimelight).pose
                : LimelightHelpers.getBotPose2d_wpiBlue(frontLimelight);
        if (isZeroPose(pose)) {
            return Optional.empty();
        }

        return Optional.of(pose);
    }

    public java.util.List<EstimatedRobotPose> getEstimatedPoses() {
        java.util.List<EstimatedRobotPose> poses = new java.util.ArrayList<>();

        getEstimatedPose(frontLimelight).ifPresent(poses::add);

        return poses;
    }

    @Override
    public void periodic() {
        getBestTargetDistance();
    }
}
