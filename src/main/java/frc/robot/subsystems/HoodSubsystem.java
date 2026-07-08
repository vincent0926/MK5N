package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX hoodMotor =
            new TalonFX(HoodConstants.khoodId);

    private final MotionMagicVoltage motionMagic =
            new MotionMagicVoltage(0).withSlot(0);

    private double targetAngleDeg = 0.0;

    public HoodSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID
        config.Slot0.kP = HoodConstants.khoodkP;
        config.Slot0.kI = HoodConstants.khoodkI;
        config.Slot0.kD = HoodConstants.khoodkD;

        // Motion Magic
        config.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        config.MotionMagic.MotionMagicAcceleration = 40.0;

        config.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive;

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        
        hoodMotor.setPosition(0.0);
        targetAngleDeg = 0.0;
    }

    /** 設定 Hood 角度（單位：度） */
    public void setAngle(double angleDeg) {

        targetAngleDeg = clamp(
                angleDeg,
                HoodConstants.khoodMinAngle,
                HoodConstants.khoodMaxAngle
        );

        double targetRotations =
                degreesToMotorRotations(targetAngleDeg);

        hoodMotor.setControl(
                motionMagic.withPosition(targetRotations)
        );
    }

    /** 相對增加角度 */
    public void addAngle(double deltaDeg) {
        setAngle(targetAngleDeg + deltaDeg);
    }

    /** 取得目前目標角度 */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    /** 取得目前角度（由 encoder 推算） */
    public double getAngle() {
        return motorRotationsToDegrees(
                hoodMotor.getPosition().getValueAsDouble()
        );
    }

    /* ---------- 轉換工具 ---------- */

    private double degreesToMotorRotations(double degrees) {
        return degrees * HoodConstants.kOneMotor;
    }

    private double motorRotationsToDegrees(double rotations) {
        return rotations / HoodConstants.kOneMotor;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}