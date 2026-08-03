package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
 * 射出罩子系統 (HoodSubsystem)
 * 負責控制射擊角度的調整罩 (Hood) 馬達。
 */
public class HoodSubsystem extends SubsystemBase {

    /** 罩子控制馬達 */
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.khoodId);

    /** Motion Magic 電壓控制模式 */
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    /** 目前的目標角度 (度) */
    private double targetAngleDeg = 0.0;
    
    // 預設的角度容許誤差（單位：度）
    private static final double kAngleTolerance = 1.0; 

    /**
     * 射出罩子系統建構子
     * 初始化馬達配置，包含軟體極限、PID 參數及 Motion Magic 設定。
     */
    public HoodSubsystem() {

        TalonFXConfiguration hoodconfig = new TalonFXConfiguration();
        hoodconfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodconfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(HoodConstants.khoodMaxAngle + 1.0);
        hoodconfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodconfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(HoodConstants.khoodMinAngle - 1.0);

        // 設定 PID 參數
        hoodconfig.Slot0.kP = HoodConstants.khoodkP;
        hoodconfig.Slot0.kI = HoodConstants.khoodkI;
        hoodconfig.Slot0.kD = HoodConstants.khoodkD;
        hoodconfig.Slot0.kS = HoodConstants.khoodkS;
        hoodconfig.Slot0.kG = HoodConstants.khoodkG;

        // 設定 Motion Magic 參數 (巡航速度與加速度)
        hoodconfig.MotionMagic.MotionMagicCruiseVelocity = 0.5; // 0.5 RPS = 180 degrees/sec (每秒 180 度)
        hoodconfig.MotionMagic.MotionMagicAcceleration = 1.0; // 1.0 RPS/s = 360 degrees/sec^2 (每秒平方 360 度)
        hoodconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // 應用配置並設定為煞車模式
        hoodMotor.getConfigurator().apply(hoodconfig);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        // 重置位置與目標角度
        hoodMotor.setPosition(0.0);
        targetAngleDeg = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Target Angle", getTargetAngle());
        SmartDashboard.putNumber("Hood Actual Angle", getAngle());
        SmartDashboard.putNumber("Hood Motor Rotations", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Stator Current", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Hood At Angle", isAtAngle());
    }

    /** 設定 Hood 角度（單位：度） */
    public void setAngle(double angleDeg) {

        targetAngleDeg = clamp(
                angleDeg,
                HoodConstants.khoodMinAngle,
                HoodConstants.khoodMaxAngle);

        double targetRotations = degreesToMotorRotations(targetAngleDeg);

        hoodMotor.setControl(
                motionMagic.withPosition(targetRotations));
    }

    /** 相對增加角度 */
    public void addAngle(double deltaDeg) {
        setAngle(targetAngleDeg + deltaDeg);
    }

    /** 取得目前目標角度 */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    /** 取得目前角度 */
    public double getAngle() {
        return motorRotationsToDegrees(
                hoodMotor.getPosition().getValueAsDouble());
    }

    /** 
     * 判斷 Hood 是否已到達目標角度（使用預設容許誤差 kAngleTolerance）
     * 解決 AutoAimAndShoot 中的編譯錯誤
     */
    public boolean isAtAngle() {
        return isAtAngle(kAngleTolerance);
    }

    /** 
     * 判斷 Hood 是否已到達目標角度（可自訂容許誤差）
     * @param toleranceDeg 允許的角度誤差（單位：度）
     */
    public boolean isAtAngle(double toleranceDeg) {
        return Math.abs(getAngle() - targetAngleDeg) <= toleranceDeg;
    }

    /* ---------- 轉換工具 ---------- */

    /**
     * 將角度 (度) 轉換為馬達旋轉圈數
     * 
     * @param degrees 目標角度
     * @return 馬達旋轉圈數
     */
    private double degreesToMotorRotations(double degrees) {
        return degrees * HoodConstants.kOneMotor;
    }

    /**
     * 將馬達旋轉圈數轉換為角度 (度)
     * 
     * @param rotations 馬達旋轉圈數
     * @return 實際角度
     */
    private double motorRotationsToDegrees(double rotations) {
        return rotations / HoodConstants.kOneMotor;
    }

    /**
     * 限制數值在指定的最小值與最大值範圍內
     * 
     * @param value 原始數值
     * @param min 最小值
     * @param max 最大值
     * @return 限制後的數值
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}