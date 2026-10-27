package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.Math.SwerveMath;

import org.joml.Math;
import org.joml.Vector2f;

public final class SwerveModule {
  static final int SignalCount = 3;

  private final int driveId;
  private final int steerId;
  private final int encoderId;
  private final float magnetOffset;
  private final InvertedValue driveInverted;
  private final boolean hardwareEnabled;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final StatusSignal<Angle> drivePositionSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Angle> steerPositionSignal;
  private final VelocityVoltage driveRequest =
      new VelocityVoltage(0.0)
          .withSlot(0)
          .withEnableFOC(Constants.Swerve.UseFocControl)
          .withUpdateFreqHz(Constants.Swerve.ControlRequestHz);
  private final PositionVoltage steerRequest =
      new PositionVoltage(0.0)
          .withSlot(0)
          .withEnableFOC(Constants.Swerve.UseFocControl)
          .withUpdateFreqHz(Constants.Swerve.ControlRequestHz);

  private float angleRad;
  private float velocityMps;
  private float drivePositionM;
  private float previousAngleRad;
  private float previousDrivePositionM;
  private float targetAngleRad;
  private float targetSpeedMps;
  private float targetDriveRotationsPerSec;
  private float targetDriveVoltage;
  private float targetSteerRotations;

  public SwerveModule(int driveId, int steerId, int encoderId, float magnetOffset) {
    this(driveId, steerId, encoderId, magnetOffset, InvertedValue.CounterClockwise_Positive, false);
  }

  SwerveModule(
      int driveId, int steerId, int encoderId, float magnetOffset, boolean hardwareEnabled) {
    this(driveId, steerId, encoderId, magnetOffset, InvertedValue.CounterClockwise_Positive, hardwareEnabled);
  }

  SwerveModule(
      int driveId,
      int steerId,
      int encoderId,
      float magnetOffset,
      InvertedValue driveInverted,
      boolean hardwareEnabled) {
    this.driveId = driveId;
    this.steerId = steerId;
    this.encoderId = encoderId;
    this.magnetOffset = magnetOffset;
    this.driveInverted = driveInverted;
    this.hardwareEnabled = hardwareEnabled;

    if (hardwareEnabled) {
      driveMotor = new TalonFX(driveId);
      steerMotor = new TalonFX(steerId);
      encoder = new CANcoder(encoderId);
      drivePositionSignal = driveMotor.getPosition(false);
      driveVelocitySignal = driveMotor.getVelocity(false);
      steerPositionSignal = steerMotor.getPosition(false);
      configureHardware();
    } else {
      driveMotor = null;
      steerMotor = null;
      encoder = null;
      drivePositionSignal = null;
      driveVelocitySignal = null;
      steerPositionSignal = null;
    }
  }

  public void reset() {
    if (hardwareEnabled) {
      driveMotor.setPosition(0.0);
      update();
    }

    previousDrivePositionM = drivePositionM;
    previousAngleRad = angleRad;
    stop();
  }

  public void update() {
    if (!hardwareEnabled) {
      return;
    }

    BaseStatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal, steerPositionSignal);
    updateFromSignals();
  }

  void updateFromSignals() {
    if (!hardwareEnabled) {
      return;
    }

    drivePositionM =
        (float) drivePositionSignal.getValueAsDouble()
            * Constants.Swerve.WheelCircumferenceM
            * Constants.Swerve.DrivePositionCoefficient;
    velocityMps =
        (float) driveVelocitySignal.getValueAsDouble()
            * Constants.Swerve.WheelCircumferenceM
            * Constants.Swerve.DrivePositionCoefficient;
    angleRad = rotationsToRadians((float) steerPositionSignal.getValueAsDouble());
  }

  void addSignals(BaseStatusSignal[] signals, int offset) {
    signals[offset] = drivePositionSignal;
    signals[offset + 1] = driveVelocitySignal;
    signals[offset + 2] = steerPositionSignal;
  }

  void optimizeBusUtilization() {
    ParentDevice.optimizeBusUtilizationForAll(
        Constants.Swerve.UnusedSignalHz, driveMotor, steerMotor, encoder);
  }

  public void operate() {
    if (!hardwareEnabled) {
      return;
    }

    driveMotor.setControl(driveRequest.withVelocity(targetDriveRotationsPerSec));
    steerMotor.setControl(steerRequest.withPosition(targetSteerRotations));
  }

  public void stop() {
    targetAngleRad = angleRad;
    targetSpeedMps = 0.0f;
    targetDriveRotationsPerSec = 0.0f;
    targetDriveVoltage = 0.0f;
    targetSteerRotations = radiansToRotations(angleRad);
  }

  public void setMeasuredState(float angleRad, float velocityMps) {
    this.angleRad = angleRad;
    this.velocityMps = velocityMps;
  }

  public void setOdometryPosition(float drivePositionM, float angleRad) {
    this.drivePositionM = drivePositionM;
    this.angleRad = angleRad;
  }

  public void resetOdometry() {
    previousDrivePositionM = drivePositionM;
    previousAngleRad = angleRad;
  }

  public void getArcDisplacement(Vector2f out) {
    float distance = drivePositionM - previousDrivePositionM;

    SwerveMath.arcDisplacement(previousAngleRad, angleRad, distance, out);

    previousDrivePositionM = drivePositionM;
    previousAngleRad = angleRad;
  }

  public void setTarget(float wantedAngleRad, float wantedSpeedMps) {
    float delta = SwerveMath.wrapRad(wantedAngleRad - angleRad);
    float speed = wantedSpeedMps;

    if (Math.abs(delta) > SwerveMath.HalfPi) {
      delta = SwerveMath.wrapRad(delta + SwerveMath.Pi);
      speed = -speed;
    }

    speed *= Math.abs(Math.cos(delta));

    targetAngleRad = angleRad + delta;
    targetSpeedMps = speed;
    targetDriveRotationsPerSec =
        speed / (Constants.Swerve.WheelCircumferenceM * Constants.Swerve.DrivePositionCoefficient);
    targetDriveVoltage = speed / Constants.Swerve.MaxDriveVelocityMps * 12.0f;
    targetSteerRotations = radiansToRotations(targetAngleRad);
  }

  public int getDriveId() {
    return driveId;
  }

  public int getSteerId() {
    return steerId;
  }

  public int getEncoderId() {
    return encoderId;
  }

  public float getMagnetOffset() {
    return magnetOffset;
  }

  public float getAngleRad() {
    return angleRad;
  }

  public float getVelocityMps() {
    return velocityMps;
  }

  public float getDrivePositionM() {
    return drivePositionM;
  }

  public float getTargetAngleRad() {
    return targetAngleRad;
  }

  public float getTargetSpeedMps() {
    return targetSpeedMps;
  }

  public float getTargetDriveRotationsPerSec() {
    return targetDriveRotationsPerSec;
  }

  public float getTargetDriveVoltage() {
    return targetDriveVoltage;
  }

  public float getTargetSteerRotations() {
    return targetSteerRotations;
  }

  private void configureHardware() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveInverted;
    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DriveSupplyCurrentLimitA;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.DriveStatorCurrentLimitA;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DriveGearRatio;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.ClosedLoopRampPeriodSec;
    driveConfig.Slot0.kP = Constants.Swerve.DriveP;
    driveConfig.Slot0.kI = Constants.Swerve.DriveI;
    driveConfig.Slot0.kD = Constants.Swerve.DriveD;
    driveConfig.Slot0.kS = Constants.Swerve.DriveS;
    driveConfig.Slot0.kV = Constants.Swerve.DriveV;
    driveConfig.Slot0.kA = Constants.Swerve.DriveA;
    driveConfig.Audio.BeepOnBoot = false;
    driveConfig.Audio.BeepOnConfig = false;
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setSafetyEnabled(false);

    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.SteerSupplyCurrentLimitA;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.SteerStatorCurrentLimitA;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    steerConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.SteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.ClosedLoopRampPeriodSec;
    steerConfig.Slot0.kP = Constants.Swerve.SteerP;
    steerConfig.Slot0.kI = Constants.Swerve.SteerI;
    steerConfig.Slot0.kD = Constants.Swerve.SteerD;
    steerConfig.Audio.BeepOnBoot = false;
    steerConfig.Audio.BeepOnConfig = false;
    steerMotor.getConfigurator().apply(steerConfig);
    steerMotor.setSafetyEnabled(false);

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.Swerve.StatusSignalHz,
        drivePositionSignal,
        driveVelocitySignal,
        steerPositionSignal,
        encoder.getPosition(false),
        encoder.getVelocity(false));
  }

  private static float rotationsToRadians(float rotations) {
    return rotations * SwerveMath.Tau;
  }

  private static float radiansToRotations(float radians) {
    return radians / SwerveMath.Tau;
  }
}
