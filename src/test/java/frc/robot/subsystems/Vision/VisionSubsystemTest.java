package frc.robot.subsystems.Vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** 用假的 NetworkTables 資料測試 VisionSubsystem 給里程計的位姿。 */
class VisionSubsystemTest {
    private static VisionSubsystem vision;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
        vision = new VisionSubsystem();
    }

    @Test
    void 紅方時給里程計的位姿仍是藍方原點座標() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        NetworkTable front = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightFront);
        front.getEntry("tv").setDouble(1.0);
        front.getEntry("botpose_wpiblue").setDoubleArray(new double[] { 12.0, 2.0, 0, 0, 0, 180.0 });
        front.getEntry("botpose_wpired").setDoubleArray(new double[] { 4.54, 6.07, 0, 0, 0, 0.0 });

        List<VisionSubsystem.EstimatedRobotPose> poses = vision.getEstimatedPoses();

        assertEquals(1, poses.size());
        // PathPlanner 與 Swerve 的里程計永遠使用藍方原點，紅方路線由 AutoBuilder 翻轉
        assertEquals(12.0, poses.get(0).estimatedPose.getX(), 1e-9);
        assertEquals(2.0, poses.get(0).estimatedPose.getY(), 1e-9);
    }

    @Test
    void 顯示位置依MegaTag版本讀不同欄位且沒看到tag回傳空() {
        NetworkTable front = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightFront);
        front.getEntry("tv").setDouble(1.0);
        front.getEntry("botpose_wpiblue").setDoubleArray(new double[] { 3.0, 4.0, 0, 0, 0, 90.0 });
        front.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[] { 5.0, 6.0, 0, 0, 0, 180.0 });

        assertEquals(3.0, vision.getFieldPose(false).get().getX(), 1e-9);
        assertEquals(90.0, vision.getFieldPose(false).get().getRotation().getDegrees(), 1e-9);
        assertEquals(5.0, vision.getFieldPose(true).get().getX(), 1e-9);
        assertEquals(6.0, vision.getFieldPose(true).get().getY(), 1e-9);

        // Limelight 有目標但 MegaTag2 還沒算出位置 (全零)
        front.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[6]);
        assertFalse(vision.getFieldPose(true).isPresent());

        front.getEntry("tv").setDouble(0.0);
        assertFalse(vision.getFieldPose(false).isPresent());
    }

    @Test
    void 讀出tag數與距離且同一張畫面只回傳一次() {
        NetworkTable front = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightFront);
        front.getEntry("tv").setDouble(1.0);
        // Limelight botpose：x, y, z, roll, pitch, yaw, 延遲, tag 數, tag 跨度, 平均距離, 平均面積
        front.getEntry("botpose_wpiblue").setDoubleArray(
                new double[] { 3.0, 4.0, 0, 0, 0, 90.0, 30.0, 2, 0.5, 2.5, 0.1 });

        List<VisionSubsystem.EstimatedRobotPose> poses = vision.getEstimatedPoses();
        assertEquals(1, poses.size());
        assertEquals(2, poses.get(0).tagCount);
        assertEquals(2.5, poses.get(0).avgTagDistM, 1e-9);
        assertTrue(poses.get(0).isGoodForHeading());
        assertTrue(poses.get(0).isGoodForPosition());

        // NT 沒有新畫面 → 不再回傳
        assertTrue(vision.getEstimatedPoses().isEmpty());
    }

    @Test
    void 單一tag不用來修朝向且太遠時不用來修位置() {
        VisionSubsystem.EstimatedRobotPose near = new VisionSubsystem.EstimatedRobotPose(
                new Pose2d(), 0.0, 1, VisionConstants.kSingleTagMaxDistM - 0.5);
        VisionSubsystem.EstimatedRobotPose far = new VisionSubsystem.EstimatedRobotPose(
                new Pose2d(), 0.0, 1, VisionConstants.kSingleTagMaxDistM + 0.5);

        assertFalse(near.isGoodForHeading());
        assertTrue(near.isGoodForPosition());
        assertFalse(far.isGoodForPosition());
    }
}
