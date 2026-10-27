package frc.robot.subsystems.Swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** 不接硬體 (hardwareEnabled = false) 測試 Swerve 的位姿與視覺融合邏輯。 */
class SwerveTest {
    private static final float EPSILON = 1e-4f;

    // AutoBuilder 是全域單例，重複建立 Swerve 會一直印「已經設定過」的錯誤，所以整個測試類別共用一個
    private static Swerve swerve;

    @BeforeAll
    static void initHal() {
        assertEquals(true, HAL.initialize(500, 0));
        swerve = new Swerve(false);
    }

    @BeforeEach
    void setUp() {
        swerve.setOutputOnlyWithCommand(false);
        swerve.stop();
        swerve.resetPose(0.0f, 0.0f, 0.0f);
        swerve.clearVisionFix();
    }

    @Test
    void resetPose_立即更新位置與朝向() {
        swerve.resetPose(1.0f, 2.0f, 0.5f);

        assertEquals(1.0f, swerve.getPoseX(), EPSILON);
        assertEquals(2.0f, swerve.getPoseY(), EPSILON);
        assertEquals(0.5f, swerve.getPoseHeadingRad(), EPSILON);
    }

    @Test
    void 第一次視覺資料直接對齊() {
        swerve.resetPose(4.0f, 2.5f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(5.0, 3.0, new Rotation2d()), 0.0);

        assertEquals(5.0f, swerve.getPoseX(), EPSILON);
        assertEquals(3.0f, swerve.getPoseY(), EPSILON);
    }

    @Test
    void 之後的視覺資料依增益逐步靠近() {
        swerve.resetPose(4.0f, 2.5f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(5.0, 3.0, new Rotation2d()), 0.0);
        swerve.addVisionMeasurement(new Pose2d(5.5, 3.0, new Rotation2d()), 0.0);

        // 5.0 + (5.5 - 5.0) * VisionBlendGain
        assertEquals(5.0f + 0.5f * frc.robot.Constants.Swerve.VisionBlendGain, swerve.getPoseX(), EPSILON);
        assertEquals(3.0f, swerve.getPoseY(), EPSILON);
    }

    @Test
    void 離群的視覺資料被丟掉() {
        swerve.resetPose(4.0f, 2.5f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(5.0, 3.0, new Rotation2d()), 0.0);
        swerve.addVisionMeasurement(new Pose2d(15.0, 3.0, new Rotation2d()), 0.0);

        assertEquals(5.0f, swerve.getPoseX(), EPSILON);
    }

    @Test
    void 第一筆視覺資料離路線起點太遠也丟掉() {
        swerve.resetPose(1.0f, 1.0f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(5.0, 3.0, new Rotation2d()), 0.0);

        assertEquals(1.0f, swerve.getPoseX(), EPSILON);
        assertEquals(1.0f, swerve.getPoseY(), EPSILON);
    }

    @Test
    void resetPose後下一筆視覺資料重新直接對齊() {
        swerve.resetPose(4.0f, 2.5f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(5.0, 3.0, new Rotation2d()), 0.0);
        swerve.resetPose(1.0f, 1.0f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(1.5, 1.0, new Rotation2d()), 0.0);

        assertEquals(1.5f, swerve.getPoseX(), EPSILON);
    }

    @Test
    void 視覺朝向依增益修正陀螺儀() {
        swerve.resetPose(2.0f, 2.0f, 0.0f);
        float visionHeading = (float) Math.toRadians(10.0);

        assertTrue(swerve.addVisionHeading(visionHeading));

        float expected = visionHeading * frc.robot.Constants.Swerve.VisionHeadingGain;
        assertEquals(expected, swerve.getPoseHeadingRad(), EPSILON);
        // 位置不受朝向修正影響
        assertEquals(2.0f, swerve.getPoseX(), EPSILON);
        assertEquals(2.0f, swerve.getPoseY(), EPSILON);
    }

    @Test
    void 視覺朝向跨過正負180度取最短方向() {
        swerve.resetPose(2.0f, 2.0f, (float) Math.toRadians(179.0));

        assertTrue(swerve.addVisionHeading((float) Math.toRadians(-179.0)));

        // 誤差是 +2° 不是 -358°
        float expected = (float) Math.toRadians(179.0 + 2.0 * frc.robot.Constants.Swerve.VisionHeadingGain);
        assertEquals(expected, swerve.getPoseHeadingRad(), EPSILON);
    }

    @Test
    void 視覺朝向誤差太大被丟掉() {
        swerve.resetPose(2.0f, 2.0f, 0.0f);

        assertFalse(swerve.addVisionHeading((float) Math.toRadians(90.0)));
        assertEquals(0.0f, swerve.getPoseHeadingRad(), EPSILON);
    }

    @Test
    void 停住時isStationary為true() {
        assertTrue(swerve.isStationary());
    }

    @Test
    void 移動中依拍攝當下的位置補償延遲() {
        // 車子 0.1 s 內從 x=2.0 開到 2.4，里程計比實際少算 0.1 m
        swerve.resetPose(2.0f, 1.0f, 0.0f);
        swerve.recordPoseHistory(10.0);
        swerve.resetPoseKeepHistoryForTest(2.4f, 1.0f);
        swerve.recordPoseHistory(10.1);

        // 鏡頭在 t=10.0 拍到實際位置 2.1 (當時里程計是 2.0)，到 10.1 才送到：偏差是 +0.1，不是 2.1-2.4=-0.3
        swerve.addVisionMeasurement(new Pose2d(2.1, 1.0, new Rotation2d()), 10.0);

        assertEquals(2.5f, swerve.getPoseX(), EPSILON);
    }

    @Test
    void 比位置紀錄還舊的畫面被丟掉() {
        swerve.resetPose(2.0f, 1.0f, 0.0f);
        swerve.recordPoseHistory(10.0);

        swerve.addVisionMeasurement(new Pose2d(2.5, 1.0, new Rotation2d()), 9.0);

        assertEquals(2.0f, swerve.getPoseX(), EPSILON);
    }

    @Test
    void 全零的視覺資料被丟掉() {
        swerve.resetPose(2.0f, 2.0f, 0.0f);
        swerve.addVisionMeasurement(new Pose2d(0.0, 0.0, new Rotation2d()), 0.0);

        assertEquals(2.0f, swerve.getPoseX(), EPSILON);
        assertEquals(2.0f, swerve.getPoseY(), EPSILON);
    }

    @Test
    void resetHeading_保留位置並把朝向歸零() {
        swerve.resetPose(1.0f, 2.0f, 1.0f);
        swerve.resetHeading();

        assertEquals(1.0f, swerve.getPoseX(), EPSILON);
        assertEquals(2.0f, swerve.getPoseY(), EPSILON);
        assertEquals(0.0, swerve.getHeadingDeg(), EPSILON);
    }

    @Test
    void 跟隨路線時不經過加速度限制器() {
        // 3.5 m/s 已接近最高速 4.0，遙控的加速度限制器在這裡幾乎不讓車子再加速
        swerve.driveRobotRelative(new ChassisSpeeds(3.5, 0.0, 0.0));
        swerve.operate();

        assertEquals(3.5f, swerve.getLimitedForwardMps(), EPSILON);
        assertEquals(0.0f, swerve.getLimitedLeftMps(), EPSILON);
    }

    @Test
    void 遙控時仍然套用加速度限制器() {
        swerve.setFieldVelocity(3.5f, 0.0f, 0.0f, false);
        swerve.operate();

        // 一個週期最多加速 MaxForwardAccelMps2 * CycleTimeSec
        float maxStep = frc.robot.Constants.Swerve.MaxForwardAccelMps2 * frc.robot.Constants.CycleTimeSec;
        assertEquals(maxStep, swerve.getLimitedForwardMps(), EPSILON);
    }

    @Test
    void 和Swerve2共用馬達時沒有指令就不輸出且清掉舊目標() {
        swerve.setOutputOnlyWithCommand(true);
        swerve.driveRobotRelative(new ChassisSpeeds(3.5, 0.0, 0.0));
        swerve.operate();

        assertEquals(0.0f, swerve.getLimitedForwardMps(), EPSILON);
        assertEquals(0.0f, swerve.getLimitedLeftMps(), EPSILON);
    }

    @Test
    void PathPlanner重設位姿後朝向視為場地座標() {
        swerve.resetPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(180.0)));

        assertTrue(swerve.isFieldPoseKnown());
        assertEquals((float) Math.PI, Math.abs(swerve.getPoseHeadingRad()), EPSILON);
    }
}
