package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Math.SwerveMath;
import org.joml.Math;
import org.joml.Vector2f;

public final class Swerve extends RobotSubsystem {
  private final Vector2f[] modulePositions = {
    new Vector2f(Constants.Swerve.HalfWheelBaseM, Constants.Swerve.HalfTrackWidthM),
    new Vector2f(Constants.Swerve.HalfWheelBaseM, -Constants.Swerve.HalfTrackWidthM),
    new Vector2f(-Constants.Swerve.HalfWheelBaseM, Constants.Swerve.HalfTrackWidthM),
    new Vector2f(-Constants.Swerve.HalfWheelBaseM, -Constants.Swerve.HalfTrackWidthM)
  };
  private final SwerveModule[] modules;
  private final Vector2f wantedFieldVelocity = new Vector2f();
  private final Vector2f wantedRobotVelocity = new Vector2f();
  private final Vector2f compensatedRobotVelocity = new Vector2f();
  private final Vector2f limitedRobotVelocity = new Vector2f();
  private final Vector2f acceleration = new Vector2f();
  private final Vector2f centerOfRotation = new Vector2f();
  private final Vector2f fieldDisplacement = new Vector2f();
  private final Vector2f fieldOffset = new Vector2f();
  private final Vector2f[] moduleDisplacements = {
    new Vector2f(), new Vector2f(), new Vector2f(), new Vector2f()
  };
  private final Vector2f[] moduleFieldPositions = {
    new Vector2f(), new Vector2f(), new Vector2f(), new Vector2f()
  };
  private final float[] moduleSpeeds = new float[4];
  private final float[] moduleAngles = new float[4];
  private final boolean hardwareEnabled;
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yawSignal;
  private final BaseStatusSignal[] signals;

  // PathPlanner:用模組實際速度與角度算機器人座標的速度(AutoBuilder 需要)
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(Constants.Swerve.HalfWheelBaseM, Constants.Swerve.HalfTrackWidthM),
          new Translation2d(Constants.Swerve.HalfWheelBaseM, -Constants.Swerve.HalfTrackWidthM),
          new Translation2d(-Constants.Swerve.HalfWheelBaseM, Constants.Swerve.HalfTrackWidthM),
          new Translation2d(-Constants.Swerve.HalfWheelBaseM, -Constants.Swerve.HalfTrackWidthM));
  private final SwerveModuleState[] measuredStates = {
    new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };
  private final Vector2f autoFieldVelocity = new Vector2f();
  // Elastic / 模擬器都看得到的場地圖
  private final Field2d field = new Field2d();
  // 最近 2 秒的里程計位置 (50 Hz)，視覺融合用來補償相機延遲
  private final PoseHistory poseHistory = new PoseHistory(100);
  private final Vector2f visionReferencePose = new Vector2f();

  // Pigeon 的原始 yaw 不直接寫入歸零 (setYaw 是非同步的，下一輪還會讀到舊值)，
  // 改用軟體偏移：headingRad = rawYawRad - yawOffsetRad，重置朝向時立即生效。
  private float rawYawRad;
  private float yawOffsetRad;
  private boolean hasVisionFix;
  private float headingRad;
  private float previousHeadingRad;
  private float poseX;
  private float poseY;
  private float poseHeadingRad;
  private float requestedOmegaRadPerSec;
  private float omegaRadPerSec;
  private float yawRateRadPerSec;
  private float headingHoldTargetRad;
  private float headingHoldWaitSec;
  private boolean evadeWanted;
  private boolean evading;
  private boolean stopped = true;
  // PathPlanner 跟隨路線中：速度已經照路線的速度/加速度限制規劃好，不再經過加速度限制器與定向保持
  private boolean followingPath;
  // 和 Swerve2 共用同一組馬達時為 true：只有在有指令使用這個底盤 (自動路線、自動瞄準) 時才送馬達指令
  private boolean outputOnlyWithCommand;
  // resetPose(Pose2d) 設過位姿後為 true (PathPlanner 路線起點或鏡頭位置)：朝向是藍方原點的場地座標，不再是開機方向
  private boolean fieldPoseKnown;
  private int evadeModuleIndex;

  public Swerve() {
    // 原本是 RobotBase.isReal():模擬時不建馬達、里程計永遠不動,PathPlanner 自動路線跑不起來。
    // Phoenix 6 在模擬時一樣可以建 TalonFX / Pigeon2(由模擬器驅動),所以一律用硬體物件。
    this(true);
  }

  Swerve(boolean hardwareEnabled) {
    this.hardwareEnabled = hardwareEnabled;
    modules = createModules(hardwareEnabled);

    if (hardwareEnabled) {
      pigeon = new Pigeon2(Constants.Swerve.PigeonId);
      yawSignal = pigeon.getYaw(false);
      configurePigeon();
      signals = createSignals();
      optimizeBusUtilization();
    } else {
      pigeon = null;
      yawSignal = null;
      signals = new BaseStatusSignal[0];
    }

    SmartDashboard.putData("Field", field);
    configureAutoBuilder();
  }

  /** PathPlanner AutoBuilder:位姿、重置位姿、機器人座標速度、輸出(機器人座標 → 場地座標 → setFieldVelocity)。 */
  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();   // deploy/pathplanner/settings.json
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),   // 平移
              new PIDConstants(5.0, 0.0, 0.0)),  // 旋轉
          config,
          () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
              == DriverStation.Alliance.Red,   // 紅方時路線自動翻轉
          this);
    } catch (Exception e) {
      DriverStation.reportError("AutoBuilder 設定失敗:" + e.getMessage(), e.getStackTrace());
    }
  }

  public Pose2d getPose() {
    return new Pose2d(poseX, poseY, new Rotation2d(poseHeadingRad));
  }

  public void resetPose(Pose2d pose) {
    resetPose((float) pose.getX(), (float) pose.getY(), (float) pose.getRotation().getRadians());
    fieldPoseKnown = true;
  }

  /** 朝向是否已經是場地座標 (跑過自動路線，或鏡頭設定過位姿)。 */
  public boolean isFieldPoseKnown() {
    return fieldPoseKnown;
  }

  /** Dashboard 上的 "Field" 場地圖，讓外部可以加其他物件 (例如鏡頭算的位置)。 */
  public Field2d getField() {
    return field;
  }

  /** 由四個模組的實際速度與角度算出機器人座標的底盤速度。 */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    for (int i = 0; i < modules.length; i++) {
      measuredStates[i].speedMetersPerSecond = modules[i].getVelocityMps();
      measuredStates[i].angle = new Rotation2d(modules[i].getAngleRad());
    }
    return kinematics.toChassisSpeeds(measuredStates);
  }

  /** PathPlanner 給的是機器人座標速度;這個底盤吃場地座標速度,所以先依目前朝向轉成場地座標。 */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveMath.robotToField(
        (float) speeds.vxMetersPerSecond, (float) speeds.vyMetersPerSecond, headingRad, autoFieldVelocity);
    setFieldVelocity(
        autoFieldVelocity.x, autoFieldVelocity.y, (float) speeds.omegaRadiansPerSecond, false);
    followingPath = true;
  }

  @Override
  public void reset() {
    wantedFieldVelocity.zero();
    wantedRobotVelocity.zero();
    compensatedRobotVelocity.zero();
    limitedRobotVelocity.zero();
    acceleration.zero();
    centerOfRotation.zero();
    fieldDisplacement.zero();
    fieldOffset.zero();
    headingRad = 0.0f;
    yawOffsetRad = rawYawRad;
    hasVisionFix = false;
    poseHistory.clear();
    previousHeadingRad = 0.0f;
    poseX = 0.0f;
    poseY = 0.0f;
    poseHeadingRad = 0.0f;
    requestedOmegaRadPerSec = 0.0f;
    omegaRadPerSec = 0.0f;
    yawRateRadPerSec = 0.0f;
    headingHoldTargetRad = 0.0f;
    headingHoldWaitSec = 0.0f;
    evadeWanted = false;
    evading = false;
    stopped = true;
    followingPath = false;
    evadeModuleIndex = 0;

    for (int i = 0; i < modules.length; i++) {
      modules[i].reset();
      moduleDisplacements[i].zero();
      moduleFieldPositions[i].zero();
      moduleSpeeds[i] = 0.0f;
      moduleAngles[i] = 0.0f;
    }

    refitModuleFieldPositions();
  }

  @Override
  public void update() {
    float lastHeadingRad = headingRad;

    if (hardwareEnabled) {
      BaseStatusSignal.refreshAll(signals);
      updateHeadingFromSignal();

      for (int i = 0; i < modules.length; i++) {
        modules[i].updateFromSignals();
      }
    } else {
      for (int i = 0; i < modules.length; i++) {
        modules[i].update();
      }
    }

    yawRateRadPerSec = SwerveMath.wrapRad(headingRad - lastHeadingRad) / Constants.CycleTimeSec;
    updateOdometry();
    recordPoseHistory(Timer.getFPGATimestamp());
    field.setRobotPose(getPose());
    // 除錯：Heading 要跟著車子轉 (逆時針變大)；Omega 是實際送出的角速度 (含定向保持)
    SmartDashboard.putNumber("Swerve/Heading Deg", radiansToDegrees(headingRad));
    SmartDashboard.putNumber("Swerve/Yaw Rate DegPerSec", radiansToDegrees(yawRateRadPerSec));
    SmartDashboard.putNumber("Swerve/Omega Cmd", omegaRadPerSec);
    SmartDashboard.putNumber("Swerve/Heading Hold Target Deg", radiansToDegrees(headingHoldTargetRad));
  }

  /** 測試用：模擬里程計走到新位置 (不清除位置紀錄，resetPose 會清)。 */
  void resetPoseKeepHistoryForTest(float x, float y) {
    poseX = x;
    poseY = y;
    refitModuleFieldPositions();
  }

  /** 記下這個週期的位置 (update() 每週期呼叫；測試直接指定時間)。 */
  void recordPoseHistory(double timestampSeconds) {
    poseHistory.add(timestampSeconds, poseX, poseY);
  }

  @Override
  public void operate() {
    if (outputOnlyWithCommand && getCurrentCommand() == null) {
      // 指令結束後馬達交給 Swerve2；清掉舊目標，避免下一個指令開始時沿用舊速度或舊的定向目標
      if (!stopped) {
        stop();
      }
      previousHeadingRad = headingRad;
      return;
    }

    if (stopped) {
      previousHeadingRad = headingRad;
      holdModules();
      return;
    }

    if (followingPath) {
      // 加速度限制器在高速時的上限會降到比路線要求還低 (接近最高速時趨近 0)，車子會一直落後路線，
      // 所以跟隨路線時直接用 PathPlanner 給的速度與角速度，偏差交給 PathPlanner 的 PID 修正。
      omegaRadPerSec = requestedOmegaRadPerSec;
      headingHoldTargetRad = headingRad;
      headingHoldWaitSec = 0.0f;
      SwerveMath.fieldToRobot(
          wantedFieldVelocity.x, wantedFieldVelocity.y, headingRad, limitedRobotVelocity);
      previousHeadingRad = headingRad;
      acceleration.zero();
    } else {
      omegaRadPerSec = calculateOmegaRadPerSec();
      SwerveMath.fieldToRobot(
          wantedFieldVelocity.x, wantedFieldVelocity.y, headingRad, wantedRobotVelocity);
      SwerveMath.applyNormalCompensation(
          wantedRobotVelocity,
          omegaRadPerSec,
          Constants.Swerve.MaxTeleopAngularVelocityRadPerSec,
          Constants.Swerve.NormalCompensationK,
          compensatedRobotVelocity);
      alignLimitedVelocityToHeading();
      SwerveMath.limitVelocity(
          compensatedRobotVelocity,
          limitedRobotVelocity,
          Constants.CycleTimeSec,
          Constants.Swerve.MaxDriveVelocityMps,
          Constants.Swerve.MaxForwardAccelMps2,
          Constants.Swerve.MaxFrontAccelMps2,
          Constants.Swerve.MaxSideAccelMps2,
          Constants.Swerve.MaxSkidAccelMps2,
          acceleration);
    }

    if (limitedRobotVelocity.lengthSquared() < 0.000001f && Math.abs(omegaRadPerSec) < 0.0001f) {
      holdModules();
      return;
    }

    updateCenterOfRotation();
    SwerveMath.calculateModuleTargets(
        limitedRobotVelocity.x,
        limitedRobotVelocity.y,
        omegaRadPerSec,
        centerOfRotation,
        modulePositions,
        moduleSpeeds,
        moduleAngles);
    SwerveMath.desaturate(moduleSpeeds, Constants.Swerve.MaxDriveVelocityMps);

    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(moduleSpeeds[i]) < Constants.Swerve.ModuleHoldSpeedMps) {
        modules[i].setTarget(modules[i].getTargetAngleRad(), 0.0f);
      } else {
        modules[i].setTarget(moduleAngles[i], moduleSpeeds[i]);
      }

      modules[i].operate();
    }
  }

  public void setFieldVelocity(
      float forwardMps, float leftMps, float omegaRadPerSec, boolean evadeWanted) {
    wantedFieldVelocity.set(forwardMps, leftMps);
    requestedOmegaRadPerSec = omegaRadPerSec;
    this.evadeWanted = evadeWanted;
    stopped = false;
    followingPath = false;
  }

  /** true 時只有在有指令使用這個底盤時才送馬達指令 (馬達平常交給 Swerve2 遙控)。 */
  public void setOutputOnlyWithCommand(boolean outputOnlyWithCommand) {
    this.outputOnlyWithCommand = outputOnlyWithCommand;
  }

  public void setHeadingRad(float headingRad) {
    this.headingRad = SwerveMath.wrapRad(headingRad);
    yawOffsetRad = rawYawRad - this.headingRad;
    poseHeadingRad = this.headingRad;
    headingHoldTargetRad = this.headingRad;
    refitModuleFieldPositions();
  }

  public void resetPose(float x, float y, float headingRad) {
    poseX = x;
    poseY = y;
    this.headingRad = SwerveMath.wrapRad(headingRad);
    yawOffsetRad = rawYawRad - this.headingRad;
    poseHeadingRad = this.headingRad;
    previousHeadingRad = this.headingRad;
    limitedRobotVelocity.zero();
    headingHoldTargetRad = this.headingRad;
    headingHoldWaitSec = 0.0f;
    // 位姿換了 (例如連跑兩次自動)，下一筆視覺資料重新直接對齊；舊的位置紀錄也作廢
    hasVisionFix = false;
    poseHistory.clear();

    for (int i = 0; i < modules.length; i++) {
      modules[i].resetOdometry();
    }

    refitModuleFieldPositions();
  }

  /**
   * 融合視覺位姿：只修正 x、y (朝向由 {@link #addVisionHeading} 在停住時修正)。
   * 補償相機延遲：跟畫面拍攝當下 (timestampSeconds) 的里程計位置比，算出偏差再套到現在的位置，
   * 所以移動中也能用。第一次收到視覺資料直接對齊；之後依 VisionBlendGain 逐步靠近；
   * 偏差超過 VisionMaxJumpM 的資料一律丟掉。
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    float x = (float) visionPose.getX();
    float y = (float) visionPose.getY();

    // Limelight 看不到標籤時 botpose 會是全 0；NaN 也一併擋掉
    if (Float.isNaN(x) || Float.isNaN(y) || (Math.abs(x) < 0.0001f && Math.abs(y) < 0.0001f)) {
      return;
    }

    // 拍攝當下的里程計位置；畫面比緩衝區還舊就丟掉，比最新一筆還新 (或沒有紀錄) 就用現在位置
    if (!poseHistory.sample(timestampSeconds, visionReferencePose)) {
      if (poseHistory.isOlderThanOldest(timestampSeconds)) {
        return;
      }
      visionReferencePose.set(poseX, poseY);
    }

    // 第一筆也要檢查：融合前已經 resetPose 到路線起點，壞掉的畫面不能直接把位置拉走
    float dx = x - visionReferencePose.x;
    float dy = y - visionReferencePose.y;

    if (dx * dx + dy * dy > Constants.Swerve.VisionMaxJumpM * Constants.Swerve.VisionMaxJumpM) {
      return;
    }

    float gain = hasVisionFix ? Constants.Swerve.VisionBlendGain : 1.0f;
    float correctionX = dx * gain;
    float correctionY = dy * gain;
    poseX += correctionX;
    poseY += correctionY;
    poseHistory.shift(correctionX, correctionY);
    hasVisionFix = true;
    refitModuleFieldPositions();
  }

  /**
   * 用視覺朝向修正陀螺儀：每次往視覺朝向靠近 VisionHeadingGain，誤差超過 VisionHeadingMaxErrorRad 丟掉。
   * 呼叫前要確認車子停住 ({@link #isStationary()})，移動或旋轉時相機延遲會讓朝向不準。
   *
   * @return 是否有套用
   */
  public boolean addVisionHeading(float visionHeadingRad) {
    if (Float.isNaN(visionHeadingRad)) {
      return false;
    }

    float error = SwerveMath.wrapRad(visionHeadingRad - headingRad);
    if (Math.abs(error) > Constants.Swerve.VisionHeadingMaxErrorRad) {
      return false;
    }

    // setHeadingRad 會一起更新陀螺儀偏移，之後的陀螺儀讀數都跟著修正
    setHeadingRad(headingRad + error * Constants.Swerve.VisionHeadingGain);
    return true;
  }

  /** 四個輪子幾乎不動且沒有在轉：這時視覺資料不受相機延遲影響。 */
  public boolean isStationary() {
    if (Math.abs(yawRateRadPerSec) > Constants.Swerve.VisionStationaryYawRateRadPerSec) {
      return false;
    }

    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(modules[i].getVelocityMps()) > Constants.Swerve.VisionStationarySpeedMps) {
        return false;
      }
    }
    return true;
  }

  /** 清除「已經有視覺定位」的狀態，下一筆視覺資料會直接對齊 (測試或長時間失去視覺後使用)。 */
  public void clearVisionFix() {
    hasVisionFix = false;
  }

  public void stop() {
    wantedFieldVelocity.zero();
    wantedRobotVelocity.zero();
    compensatedRobotVelocity.zero();
    limitedRobotVelocity.zero();
    previousHeadingRad = headingRad;
    requestedOmegaRadPerSec = 0.0f;
    omegaRadPerSec = 0.0f;
    headingHoldTargetRad = headingRad;
    headingHoldWaitSec = 0.0f;
    evadeWanted = false;
    evading = false;
    stopped = true;
    followingPath = false;
    centerOfRotation.zero();

    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
    }
  }

  public SwerveModule getModule(int index) {
    return modules[index];
  }

  public float getCenterOfRotationX() {
    return centerOfRotation.x;
  }

  public float getCenterOfRotationY() {
    return centerOfRotation.y;
  }

  public float getLimitedForwardMps() {
    return limitedRobotVelocity.x;
  }

  public float getLimitedLeftMps() {
    return limitedRobotVelocity.y;
  }

  public float getPoseX() {
    return poseX;
  }

  public float getPoseY() {
    return poseY;
  }

  public float getPoseHeadingRad() {
    return poseHeadingRad;
  }

  /** 把目前朝向設為 0（位置不變），陀螺儀也一起歸零。 */
  public void resetHeading() {
    resetPose(poseX, poseY, 0.0f);
  }

  /** 給 Limelight MegaTag2 用，單位是度。 */
  public double getHeadingDeg() {
    return radiansToDegrees(headingRad);
  }

  /** 給 Limelight MegaTag2 用，單位是度/秒。 */
  public double getYawRateDegPerSec() {
    return radiansToDegrees(yawRateRadPerSec);
  }

  public float getModuleFieldX(int index) {
    return moduleFieldPositions[index].x;
  }

  public float getModuleFieldY(int index) {
    return moduleFieldPositions[index].y;
  }

  public int getEvadeModuleIndex() {
    return evadeModuleIndex;
  }

  private void updateHeadingFromSignal() {
    rawYawRad =
        degreesToRadians((float) yawSignal.getValueAsDouble() * Constants.Swerve.PigeonGyroScalarZ);
    headingRad = rawYawRad - yawOffsetRad;
  }

  private void updateOdometry() {
    float centerX = 0.0f;
    float centerY = 0.0f;

    for (int i = 0; i < modules.length; i++) {
      modules[i].getArcDisplacement(moduleDisplacements[i]);
      SwerveMath.robotToField(
          moduleDisplacements[i].x, moduleDisplacements[i].y, headingRad, fieldDisplacement);
      moduleFieldPositions[i].add(fieldDisplacement);
      SwerveMath.robotToField(
          modulePositions[i].x, modulePositions[i].y, headingRad, fieldOffset);
      centerX += moduleFieldPositions[i].x - fieldOffset.x;
      centerY += moduleFieldPositions[i].y - fieldOffset.y;
    }

    poseX = centerX / modules.length;
    poseY = centerY / modules.length;
    poseHeadingRad = headingRad;
    refitModuleFieldPositions();
  }

  private void refitModuleFieldPositions() {
    for (int i = 0; i < modules.length; i++) {
      SwerveMath.robotToField(
          modulePositions[i].x, modulePositions[i].y, headingRad, fieldOffset);
      moduleFieldPositions[i].set(poseX + fieldOffset.x, poseY + fieldOffset.y);
    }
  }

  private void holdModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
      modules[i].operate();
    }
  }

  private void updateCenterOfRotation() {
    if (!evadeWanted) {
      evading = false;
      centerOfRotation.zero();
      return;
    }

    if (!evading) {
      chooseEvadeModule();
      evading = true;
    }

    centerOfRotation.set(modulePositions[evadeModuleIndex]);
  }

  private float calculateOmegaRadPerSec() {
    if (Math.abs(requestedOmegaRadPerSec) > Constants.Swerve.HeadingHoldOmegaDeadband) {
      headingHoldTargetRad = headingRad;
      headingHoldWaitSec = 0.0f;
      return requestedOmegaRadPerSec;
    }

    if (wantedFieldVelocity.lengthSquared() < 0.000001f) {
      headingHoldTargetRad = headingRad;
      headingHoldWaitSec = 0.0f;
      return 0.0f;
    }

    if (headingHoldWaitSec < Constants.Swerve.HeadingHoldDelaySec) {
      headingHoldTargetRad = headingRad;
      headingHoldWaitSec += Constants.CycleTimeSec;
      return 0.0f;
    }

    return SwerveMath.headingHoldOmega(
        headingHoldTargetRad,
        headingRad,
        Constants.Swerve.HeadingHoldP,
        Constants.Swerve.MaxTeleopAngularVelocityRadPerSec);
  }

  private void chooseEvadeModule() {
    float speedSquared = limitedRobotVelocity.lengthSquared();
    float impactX = Constants.Swerve.RobotRadiusM;
    float impactY = 0.0f;

    if (speedSquared > 0.0001f) {
      float scale = Constants.Swerve.RobotRadiusM / Math.sqrt(speedSquared);
      impactX = limitedRobotVelocity.x * scale;
      impactY = limitedRobotVelocity.y * scale;
    }

    evadeModuleIndex = SwerveMath.nearestModule(modulePositions, impactX, impactY);
  }

  private void configurePigeon() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPoseYaw = Constants.Swerve.PigeonMountPoseYawDeg;
    config.MountPose.MountPosePitch = Constants.Swerve.PigeonMountPosePitchDeg;
    config.MountPose.MountPoseRoll = Constants.Swerve.PigeonMountPoseRollDeg;
    pigeon.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Swerve.StatusSignalHz, yawSignal);
  }

  private void alignLimitedVelocityToHeading() {
    float deltaHeading = SwerveMath.wrapRad(headingRad - previousHeadingRad);
    SwerveMath.fieldToRobot(
        limitedRobotVelocity.x, limitedRobotVelocity.y, deltaHeading, limitedRobotVelocity);
    previousHeadingRad = headingRad;
  }

  private void optimizeBusUtilization() {
    pigeon.optimizeBusUtilization(Constants.Swerve.UnusedSignalHz);

    for (int i = 0; i < modules.length; i++) {
      modules[i].optimizeBusUtilization();
    }
  }

  private BaseStatusSignal[] createSignals() {
    BaseStatusSignal[] createdSignals =
        new BaseStatusSignal[1 + modules.length * SwerveModule.SignalCount];
    createdSignals[0] = yawSignal;

    for (int i = 0; i < modules.length; i++) {
      modules[i].addSignals(createdSignals, 1 + i * SwerveModule.SignalCount);
    }

    return createdSignals;
  }

  private static SwerveModule[] createModules(boolean hardwareEnabled) {
    return new SwerveModule[] {
      new SwerveModule(
          Constants.Swerve.FrontLeftDriveId,
          Constants.Swerve.FrontLeftSteerId,
          Constants.Swerve.FrontLeftEncoderId,
          Constants.Swerve.FrontLeftMagnetOffset,
          Constants.Swerve.FrontLeftDriveInverted,
          hardwareEnabled),
      new SwerveModule(
          Constants.Swerve.FrontRightDriveId,
          Constants.Swerve.FrontRightSteerId,
          Constants.Swerve.FrontRightEncoderId,
          Constants.Swerve.FrontRightMagnetOffset,
          Constants.Swerve.FrontRightDriveInverted,
          hardwareEnabled),
      new SwerveModule(
          Constants.Swerve.BackLeftDriveId,
          Constants.Swerve.BackLeftSteerId,
          Constants.Swerve.BackLeftEncoderId,
          Constants.Swerve.BackLeftMagnetOffset,
          Constants.Swerve.BackLeftDriveInverted,
          hardwareEnabled),
      new SwerveModule(
          Constants.Swerve.BackRightDriveId,
          Constants.Swerve.BackRightSteerId,
          Constants.Swerve.BackRightEncoderId,
          Constants.Swerve.BackRightMagnetOffset,
          Constants.Swerve.BackRightDriveInverted,
          hardwareEnabled)
    };
  }

  private static float degreesToRadians(float degrees) {
    return degrees * SwerveMath.Pi / 180.0f;
  }

  private static double radiansToDegrees(float radians) {
    return radians * 180.0 / SwerveMath.Pi;
  }
}

