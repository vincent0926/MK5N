package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut; // 1. 匯入 NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * 射擊子系統 (ShooterSubsystem)
 * 負責控制四顆射擊馬達以射出物件。
 */
public class ShooterSubsystem extends SubsystemBase {

    /** 左上方射擊馬達 (Master) */
    private final TalonFX LTshooterMotor = new TalonFX(ShooterConstants.kLTShooterId);
    /** 左下方射擊馬達 */
    private final TalonFX LDshooterMotor = new TalonFX(ShooterConstants.kLDShooterId);
    /** 右上方射擊馬達 */
    private final TalonFX RTshooterMotor = new TalonFX(ShooterConstants.kRTShooterId);
    /** 右下方射擊馬達 */
    private final TalonFX RDshooterMotor = new TalonFX(ShooterConstants.kRDShooterId);

    /** 控制物件 */
    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut(); // 2. 宣告 Coast/Neutral 物件

    public ShooterSubsystem() {

        TalonFXConfiguration shooterconfig = new TalonFXConfiguration();

        // 設定 PID 參數
        shooterconfig.Slot0.kP = ShooterConstants.kShooterkP;
        shooterconfig.Slot0.kI = ShooterConstants.kShooterkI;
        shooterconfig.Slot0.kD = ShooterConstants.kShooterkD;
        shooterconfig.Slot0.kS = ShooterConstants.kShooterkS;
        shooterconfig.Slot0.kV = ShooterConstants.kShooterkV;
        shooterconfig.Slot0.kA = ShooterConstants.kShooterkA;

        // 設定中立模式為滑行 (Coast)
        shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // 電流限制
        shooterconfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        shooterconfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;
        shooterconfig.CurrentLimits.StatorCurrentLimit = 60;
        shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // 應用設定到所有射擊馬達
        LTshooterMotor.getConfigurator().apply(shooterconfig);
        LDshooterMotor.getConfigurator().apply(shooterconfig);
        RTshooterMotor.getConfigurator().apply(shooterconfig);
        RDshooterMotor.getConfigurator().apply(shooterconfig);

        // 設定跟隨關係
        LDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        RTshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        RDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * 以預設常數速度運轉射擊馬達
     */
    public void runshooter() {
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(ShooterConstants.kshooterVelocityVoltage));
    }

    /**
     * 設定射擊馬達的目標 RPS（每秒轉數）
     */
    public void setRPS(double targetRPS) {
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(targetRPS));
    }

    /**
     * 檢查射擊馬達是否已達到目標速度
     */
    public boolean isAtSpeed(double targetRPS) {
        double currentRPS = LTshooterMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentRPS - targetRPS) < 2.0;
    }

    /**
     * 停止所有射擊馬達（真正切斷輸出，讓飛輪順暢滑行停止）
     */
    public void stop() {
        // 3. 使用 NeutralOut 讓馬達切斷電源自然滑行
        LTshooterMotor.setControl(neutralRequest);
    }
}