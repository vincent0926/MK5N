package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
 * 仰角機構 (Hood)
 *
 * 角度單位：度，以開機位置為 0 度 (實際仰角 = 角度 + khoodStartAngle)。
 * 控制單位：馬達圈數，所以 kP、kS、kG、Motion Magic 都是馬達端的數值。
 * 齒比 馬達:hood = 59:4，hood 轉 360 度 = 馬達轉 kHoodGearRatio 圈。
 */
public class HoodSubsystem extends SubsystemBase {

    /** 到位判斷的預設容許誤差 (度) */
    private static final double kAngleToleranceDeg = 1.0;

    /** 軟體極限比可用範圍多留的餘裕 (度)，避免剛好到端點時被擋住 */
    private static final double kSoftLimitMarginDeg = 1.0;

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.khoodId);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    /** 目前的目標角度 (度) */
    private double targetAngleDeg = 0.0;

    public HoodSubsystem() {
        configureMotor();

        // 開機位置 = 0 度
        hoodMotor.setPosition(0.0);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 馬達方向與停止模式
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 電流限制
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.kHoodSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.kHoodSupplyCurrentLimitEnable;

        // 軟體極限
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                degreesToMotorRotations(HoodConstants.khoodMaxAngle + kSoftLimitMarginDeg);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                degreesToMotorRotations(HoodConstants.khoodMinAngle - kSoftLimitMarginDeg);

        // PID 與前饋 (V / 馬達圈)
        config.Slot0.kP = HoodConstants.khoodkP;
        config.Slot0.kI = HoodConstants.khoodkI;
        config.Slot0.kD = HoodConstants.khoodkD;
        config.Slot0.kS = HoodConstants.khoodkS;
        config.Slot0.kG = HoodConstants.khoodkG;

        // Motion Magic (馬達 RPS、RPS/s、RPS/s²)
        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kHoodCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.kHoodAcceleration;
        config.MotionMagic.MotionMagicJerk = HoodConstants.kHoodJerk;

        hoodMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        double angleDeg = getAngle();

        SmartDashboard.putNumber("Hood/Target Angle", targetAngleDeg);
        SmartDashboard.putNumber("Hood/Actual Angle", angleDeg);
        SmartDashboard.putNumber("Hood/Real Elevation", angleDeg + HoodConstants.khoodStartAngle);
        SmartDashboard.putNumber("Hood/Motor Rotations", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Hood/At Angle", isAtAngle());
    }

    /* ---------- 控制 ---------- */

    /** 移動到指定角度 (度)，超出範圍會被限制在 khoodMinAngle ~ khoodMaxAngle */
    public void setAngle(double angleDeg) {
        targetAngleDeg = MathUtil.clamp(angleDeg, HoodConstants.khoodMinAngle, HoodConstants.khoodMaxAngle);
        hoodMotor.setControl(motionMagic.withPosition(degreesToMotorRotations(targetAngleDeg)));
    }

    /** 目標角度再加 deltaDeg 度 (負數就是往下) */
    public void addAngle(double deltaDeg) {
        setAngle(targetAngleDeg + deltaDeg);
    }

    /* ---------- 狀態 ---------- */

    /** 目標角度 (度) */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    /** 目前角度 (度) */
    public double getAngle() {
        return motorRotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    /** 是否已到達目標角度 (誤差 kAngleToleranceDeg 以內) */
    public boolean isAtAngle() {
        return isAtAngle(kAngleToleranceDeg);
    }

    /** 是否已到達目標角度 (誤差 toleranceDeg 以內) */
    public boolean isAtAngle(double toleranceDeg) {
        return Math.abs(getAngle() - targetAngleDeg) <= toleranceDeg;
    }

    /* ---------- 單位換算 ---------- */

    /** hood 角度 (度) → 馬達圈數 */
    private static double degreesToMotorRotations(double degrees) {
        return degrees / 360.0 * HoodConstants.kHoodGearRatio;
    }

    /** 馬達圈數 → hood 角度 (度) */
    private static double motorRotationsToDegrees(double rotations) {
        return rotations / HoodConstants.kHoodGearRatio * 360.0;
    }
}
