package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final Pigeon2 pigeon;

  private final SwerveDriveOdometry odometry;

  public DriveSubsystem() {
    // 把四個輪子通通 new 出來
    frontLeft = new SwerveModule(0, SwerveConstants.kFLDriveId, SwerveConstants.kFLSteerId,
        SwerveConstants.kFLEncoderId, SwerveConstants.kFLOffset, SwerveConstants.kFLDriveInverted);
    frontRight = new SwerveModule(1, SwerveConstants.kFRDriveId, SwerveConstants.kFRSteerId,
        SwerveConstants.kFREncoderId, SwerveConstants.kFROffset, SwerveConstants.kFRDriveInverted);
    backLeft = new SwerveModule(2, SwerveConstants.kBLDriveId, SwerveConstants.kBLSteerId, SwerveConstants.kBLEncoderId,
        SwerveConstants.kBLOffset, SwerveConstants.kBLDriveInverted);
    backRight = new SwerveModule(3, SwerveConstants.kBRDriveId, SwerveConstants.kBRSteerId,
        SwerveConstants.kBREncoderId, SwerveConstants.kBROffset, SwerveConstants.kBRDriveInverted);
    pigeon = new Pigeon2(11);

    odometry = new SwerveDriveOdometry(
        SwerveConstants.kDriveKinematics,
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  @Override
  public void periodic() {
    odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public void resetpigeonOdometry() {
    pigeon.reset();
    Pose2d currentPose = odometry.getPoseMeters();
    odometry.resetPosition(
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(0))
    );
  }

}
