package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;

public class Limelight4Subsystem extends SubsystemBase {

    private static final String LIMELIGHT = VisionConstants.kLimelightFront; // "limelight-up"

    private AprilTagFieldLayout fieldLayout;

    public Limelight4Subsystem() {
        try {

            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {

            DriverStation.reportError("have a erro", e.getStackTrace());
        }
    }

    public boolean canReturnBall() {

        if (!LimelightHelpers.getTV(LIMELIGHT)) {
            return false;
        }

        int tid = (int) LimelightHelpers.getFiducialID(LIMELIGHT);

        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        if (isRed) {
            return tid == 6 || tid == 1;
        } else {
            return tid == 17 || tid == 22;
        }
    }

    public double getDistanceToTarget() {

        if (!LimelightHelpers.getTV(LIMELIGHT)) {
            return -1.0;
        }

        int tid = (int) LimelightHelpers.getFiducialID(LIMELIGHT);

        if (tid > 0 && fieldLayout != null) {

            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tid);

            if (tagPose.isPresent()) {

                Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT);

                Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();

                return robotPose.getTranslation().getDistance(tagTrans);
            }
        }

        return -1.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight4 Can Return Ball", canReturnBall());
        SmartDashboard.putNumber("Limelight4 Target ID", LimelightHelpers.getFiducialID(LIMELIGHT));
        SmartDashboard.putNumber("Limelight4 Target Distance",
                getDistanceToTarget());
    }

}
