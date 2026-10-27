package frc.robot.subsystems.Swerve2;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * 底盤子系統 (DriveSubsystem)
 * 負責控制機器的 Swerve (萬向) 底盤，包含四個模組與陀螺儀，並處理里程計與路徑規劃。
 */
public class Swerve2 extends SubsystemBase {
  /** 左前 Swerve 模組 */
  private final SwerveModule2 frontLeft;
  /** 右前 Swerve 模組 */
  private final SwerveModule2 frontRight;
  /** 左後 Swerve 模組 */
  private final SwerveModule2 backLeft;
  /** 右後 Swerve 模組 */
  private final SwerveModule2 backRight;

  /** Pigeon2 陀螺儀 */
  private final Pigeon2 pigeon;

  /** Swerve 底盤姿態估測器 */
  private final SwerveDrivePoseEstimator poseEstimator;

  // 模擬專用變數
  private ChassisSpeeds m_simSpeeds = new ChassisSpeeds();
  private Pose2d m_simOdometryPose = new Pose2d();

  // Elastic Field2d：讓 Dashboard 能顯示機器人在場地上的即時位置
  private final Field2d m_field = new Field2d();

  // 定向保持 (Heading Hold)：沒有手動轉向輸入時，鎖定目前朝向、避免平移時慢慢轉掉
  private final PIDController headingController = new PIDController(SwerveConstants.kHeadingHoldP, 0, 0);
  private boolean headingLocked = false;
  private double headingHoldTargetRad = 0.0;

  // 駕駛用的場地導向朝向偏移。不用 pigeon.reset()：陀螺儀和 Swerve 共用，reset 會讓 Swerve 的朝向跳掉
  private Rotation2d driverHeadingOffset = new Rotation2d();

  // 平移加速度限制 (Slew Rate)：避免搖桿瞬間拉滿造成打滑
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.kMaxAccel);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.kMaxAccel);

  /**
   * 底盤子系統建構子
   * 初始化所有 Swerve 模組、陀螺儀及姿態估測器 (AutoBuilder 由 Swerve 設定)。
   */
  public Swerve2() {
    // 把四個輪子通通 new 出來 (初始化四個 Swerve 模組)
    frontLeft = new SwerveModule2(0, SwerveConstants.kFLDriveId, SwerveConstants.kFLSteerId,
        SwerveConstants.kFLEncoderId, SwerveConstants.kFLOffset, SwerveConstants.kFLDriveInverted);
    frontRight = new SwerveModule2(1, SwerveConstants.kFRDriveId, SwerveConstants.kFRSteerId,
        SwerveConstants.kFREncoderId, SwerveConstants.kFROffset, SwerveConstants.kFRDriveInverted);
    backLeft = new SwerveModule2(2, SwerveConstants.kBLDriveId, SwerveConstants.kBLSteerId, SwerveConstants.kBLEncoderId,
        SwerveConstants.kBLOffset, SwerveConstants.kBLDriveInverted);
    backRight = new SwerveModule2(3, SwerveConstants.kBRDriveId, SwerveConstants.kBRSteerId,
        SwerveConstants.kBREncoderId, SwerveConstants.kBROffset, SwerveConstants.kBRDriveInverted);
    pigeon = new Pigeon2(11);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kDriveKinematics,
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d());

    // 自動模式由新的 Swerve 負責 (它的建構子呼叫 AutoBuilder.configure)。
    // 這裡不能再 configure，否則後建立的 Swerve2 會蓋掉設定，自動路線變成由 Swerve2 執行。

    // 將 Field2d 發布到 SmartDashboard，供 Elastic 訂閱
    SmartDashboard.putData("Swerve2Field", m_field); // "Field" 留給 Swerve (自動路線與視覺用的位姿)
  }

  /**
   * 驅動機器人
   * 
   * @param xSpeed X 軸方向的速度 (前進/後退)
   * @param ySpeed Y 軸方向的速度 (左移/右移)
   * @param rot 旋轉角速度
   * @param fieldRelative 是否為相對於場地的控制模式
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;

    // 根據是否相對於場地來計算底盤速度
    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getDriverHeading());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    // 將連續的底盤速度離散化以補償控制延遲 (預設 0.02 秒)
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    // 轉換為各個 Swerve 模組的狀態
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 避免模組速度超過最大設定值
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

    if (RobotBase.isSimulation()) {
      m_simSpeeds = chassisSpeeds;
    }

    // 將計算出的狀態設置給各個模組
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  /**
   * 搖桿手動駕駛專用：平移加速度限制 (Slew Rate) + 定向保持 (Heading Hold)。
   * 當沒有手動轉向輸入時自動鎖定目前朝向，避免平移時機器人慢慢被轉掉；
   * 一旦有轉向輸入，立刻解除鎖定並採用手動旋轉速度。
   *
   * @param xSpeed        X 軸方向的速度 (前進/後退)
   * @param ySpeed        Y 軸方向的速度 (左移/右移)
   * @param rot           旋轉角速度；在定向保持生效時會被忽略並改用 PID 輸出
   * @param fieldRelative 是否為相對於場地的控制模式
   */
  public void driveWithHeadingHold(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);

    double headingRad = getRotation2d().getRadians();

    if (Math.abs(rot) > SwerveConstants.kHeadingHoldOmegaDeadband) {
      headingLocked = false;
      headingHoldTargetRad = headingRad;
    } else if (xSpeed != 0.0 || ySpeed != 0.0) {
      if (!headingLocked) {
        headingLocked = true;
        headingHoldTargetRad = headingRad;
      }
      rot = headingController.calculate(headingRad, headingHoldTargetRad);
    } else {
      headingLocked = false;
      headingHoldTargetRad = headingRad;
      rot = 0.0;
    }

    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void periodic() {
    if (!RobotBase.isSimulation()) {
      poseEstimator.update(
          getRotation2d(),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
          });
    }

    // 每 20ms 同步機器人位置到 Field2d（Elastic 場地圖會即時更新）
    m_field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // 簡易物理引擎：把目標速度積分成位移，讓虛擬機器人可以在畫面上跑
    double dt = 0.02;
    Twist2d twist = new Twist2d(
        m_simSpeeds.vxMetersPerSecond * dt,
        m_simSpeeds.vyMetersPerSecond * dt,
        m_simSpeeds.omegaRadiansPerSecond * dt
    );
    m_simOdometryPose = m_simOdometryPose.exp(twist);
  }

  /**
   * 新增視覺測量數據，以修正姿態估測
   * 
   * @param visionPose 視覺系統回傳的姿態
   * @param timestamp 測量時間戳記
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /**
   * 取得機器人目前的估測姿態 (Pose)
   * 
   * @return 機器人的姿態
   */
  public Pose2d getPose() {
    if (RobotBase.isSimulation()) {
      return m_simOdometryPose;
    }
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * 重置機器人的姿態 (里程計)
   * 
   * @param pose 目標姿態
   */
  public void resetPose(Pose2d pose) {
    System.out.println("[DriveSubsystem] 呼叫 resetPose，參數: " + pose);
    if (RobotBase.isSimulation()) {
      m_simOdometryPose = pose;
    }
    poseEstimator.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  /**
   * 取得相對於機器人的底盤速度
   * 
   * @return 底盤速度 (ChassisSpeeds)
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  /**
   * 以相對於機器人的模式驅動
   * 通常用於自動階段 (PathPlanner)
   * 
   * @param speeds 相對於機器人的速度
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    if (RobotBase.isSimulation() && (Math.abs(speeds.vxMetersPerSecond) > 0.01 || Math.abs(speeds.vyMetersPerSecond) > 0.01)) {
        System.out.println("[DriveSubsystem] driveRobotRelative 接收速度: " + speeds);
    }
    SwerveModuleState[] states = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeed);
    
    if (RobotBase.isSimulation()) {
      m_simSpeeds = speeds;
    }

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  /**
   * 取得機器人目前的旋轉角度
   * 
   * @return Rotation2d 角度物件
   */
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  /** 駕駛用的場地導向朝向 (陀螺儀角度減掉 resetDriverHeading 記下的偏移)。 */
  public Rotation2d getDriverHeading() {
    return pigeon.getRotation2d().minus(driverHeadingOffset);
  }

  /** 把目前車頭方向設為駕駛的「前方」，不動陀螺儀，所以不影響 Swerve 的位姿。 */
  public void resetDriverHeading() {
    driverHeadingOffset = pigeon.getRotation2d();
  }

  /**
   * 重置陀螺儀及里程計角度，將目前朝向設為 0 度
   */
  public void resetpigeonOdometry() {
    pigeon.reset();
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(0)));
  }

  /**
   * 取得機器人目前的 Yaw 角度 (度)
   */
  public double getHeading() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * 取得機器人目前的 Yaw 角速度 (度/秒)
   */
  public double getYawRate() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * 鎖定底盤為 X 型，產生最大防撞摩擦力
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

}