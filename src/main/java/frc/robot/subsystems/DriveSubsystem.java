package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * 底盤子系統 (DriveSubsystem)
 * 負責控制機器的 Swerve (萬向) 底盤，包含四個模組與陀螺儀，並處理里程計與路徑規劃。
 */
public class DriveSubsystem extends SubsystemBase {
  /** 左前 Swerve 模組 */
  private final SwerveModule frontLeft;
  /** 右前 Swerve 模組 */
  private final SwerveModule frontRight;
  /** 左後 Swerve 模組 */
  private final SwerveModule backLeft;
  /** 右後 Swerve 模組 */
  private final SwerveModule backRight;

  /** Pigeon2 陀螺儀 */
  private final Pigeon2 pigeon;

  /** Swerve 底盤姿態估測器 */
  private final SwerveDrivePoseEstimator poseEstimator;

  // 模擬專用變數
  private ChassisSpeeds m_simSpeeds = new ChassisSpeeds();
  private Pose2d m_simOdometryPose = new Pose2d();

  // Elastic Field2d：讓 Dashboard 能顯示機器人在場地上的即時位置
  private final Field2d m_field = new Field2d();

  /**
   * 底盤子系統建構子
   * 初始化所有 Swerve 模組、陀螺儀、姿態估測器及 PathPlanner 的 AutoBuilder。
   */
  public DriveSubsystem() {
    // 把四個輪子通通 new 出來 (初始化四個 Swerve 模組)
    frontLeft = new SwerveModule(0, SwerveConstants.kFLDriveId, SwerveConstants.kFLSteerId,
        SwerveConstants.kFLEncoderId, SwerveConstants.kFLOffset, SwerveConstants.kFLDriveInverted);
    frontRight = new SwerveModule(1, SwerveConstants.kFRDriveId, SwerveConstants.kFRSteerId,
        SwerveConstants.kFREncoderId, SwerveConstants.kFROffset, SwerveConstants.kFRDriveInverted);
    backLeft = new SwerveModule(2, SwerveConstants.kBLDriveId, SwerveConstants.kBLSteerId, SwerveConstants.kBLEncoderId,
        SwerveConstants.kBLOffset, SwerveConstants.kBLDriveInverted);
    backRight = new SwerveModule(3, SwerveConstants.kBRDriveId, SwerveConstants.kBRSteerId,
        SwerveConstants.kBREncoderId, SwerveConstants.kBROffset, SwerveConstants.kBRDriveInverted);
    pigeon = new Pigeon2(11);

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

    // PathPlanner 2025/2026 AutoBuilder Configuration
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      config = null;
    }

    try {
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              new PIDConstants(10.0, 0.0, 0.0), // 位移 PID 參數
              new PIDConstants(7.0, 0.0, 0.0) // 旋轉 PID 參數
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // 將此子系統作為需求傳入
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure AutoBuilder: " + e.getMessage(), e.getStackTrace());
    }

    // 將 Field2d 發布到 SmartDashboard，供 Elastic 訂閱
    SmartDashboard.putData("Field", m_field);
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
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d());
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