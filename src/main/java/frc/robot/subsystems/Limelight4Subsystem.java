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

    private final NetworkTable limelight4Table = NetworkTableInstance.getDefault().getTable("limelight");
    private AprilTagFieldLayout fieldLayout;

    public Limelight4Subsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout for Limelight4", e.getStackTrace());
        }
    }

    /**
     * 判斷是否滿足回傳球條件：
     * - 藍方聯盟且看到 AprilTag 17 或 22
     * - 紅方聯盟且看到 AprilTag 6 或 1
     */
    public boolean canReturnBall() {
        if (limelight4Table.getEntry("tv").getDouble(0.0) != 1.0) {
            return false;
        }

        long tid = (long) limelight4Table.getEntry("tid").getInteger(0);
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        if (isRed) {
            return tid == 6 || tid == 1;
        } else {
            return tid == 17 || tid == 22;
        }
    }

    /**
     * 取得當前距離目標 AprilTag 的平面距離（單位：公尺）。
     * 此計算方式與 VisionSubsystem 一致，使用機器人場地座標與 AprilTag 場地座標進行距離計算。
     * @return 距離目標的公尺數，若無目標或無法計算則回傳 -1.0
     */
    public double getDistanceToTarget() {
        if (limelight4Table.getEntry("tv").getDouble(0.0) != 1.0) {
            return -1.0;
        }

        double[] botpose = limelight4Table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length >= 6) {
            long tid = (long) limelight4Table.getEntry("tid").getInteger(0);
            if (tid > 0 && fieldLayout != null) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose((int) tid);
                if (tagPose.isPresent()) {
                    Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                    Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();
                    return robotTrans.getDistance(tagTrans);
                }
            }
        }
        return -1.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight4 Can Return Ball", canReturnBall());
        SmartDashboard.putNumber("Limelight4 Target ID", limelight4Table.getEntry("tid").getInteger(0));
        SmartDashboard.putNumber("Limelight4 Target Distance", getDistanceToTarget());
    }
}
