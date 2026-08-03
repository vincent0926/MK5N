package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

/**
 * 射擊子系統 (ShooterSubsystem)
 * 負責控制四顆射擊馬達以射出物件。
 */
public class ShooterSubsystem extends SubsystemBase {

    /** 左上方射擊馬達 */
    private final TalonFX LTshooterMotor = new TalonFX(ShooterConstants.kLTShooterId);
    /** 左下方射擊馬達 */
    private final TalonFX LDshooterMotor = new TalonFX(ShooterConstants.kLDShooterId);
    /** 右上方射擊馬達 */
    private final TalonFX RTshooterMotor = new TalonFX(ShooterConstants.kRTShooterId);
    /** 右下方射擊馬達 */
    private final TalonFX RDshooterMotor = new TalonFX(ShooterConstants.kRDShooterId);

    /** 射擊馬達的速度電壓輸出控制物件 */
    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);
    // private final DutyCycleOut shooterDutyCycleOut = new DutyCycleOut(0);

    /**
     * 射擊子系統建構子
     * 初始化各個射擊馬達的 PID 參數、電流限制以及從動關係。
     */
    public ShooterSubsystem() {

        TalonFXConfiguration shooterconfig = new TalonFXConfiguration();

        // 設定 PID 參數
        shooterconfig.Slot0.kP = ShooterConstants.kShooterkP;
        shooterconfig.Slot0.kI = ShooterConstants.kShooterkI;
        shooterconfig.Slot0.kD = ShooterConstants.kShooterkD;
        shooterconfig.Slot0.kS = ShooterConstants.kShooterkS;
        shooterconfig.Slot0.kV = ShooterConstants.kShooterkV;
        shooterconfig.Slot0.kA = ShooterConstants.kShooterkA;

        // 設定為滑行模式
        shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // 設定供應電流限制
        shooterconfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        shooterconfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;

        // 設定轉子電流限制
        shooterconfig.CurrentLimits.StatorCurrentLimit = 60;
        shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // 應用設定到所有射擊馬達
        LTshooterMotor.getConfigurator().apply(shooterconfig);
        LDshooterMotor.getConfigurator().apply(shooterconfig);
        RTshooterMotor.getConfigurator().apply(shooterconfig);
        RDshooterMotor.getConfigurator().apply(shooterconfig);

        // 設定跟隨主馬達 (LTshooterMotor) 的從動馬達
        LDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        RTshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        RDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    /**
     * 以預設常數電壓速度運轉射擊馬達
     */
    public void runshooter() {
        // 記得修改 PID 數值
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(ShooterConstants.kshooterVelocityVoltage));
    
        // LTshooterMotor.setControl(new DutyCycleOut(0.6));
    }

    /**
     * 設定射擊馬達的目標 RPS（每秒轉數）
     * 
     * @param targetRPS 目標每秒轉數
     */
    public void setRPS(double targetRPS) {
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(targetRPS));
    }

    /**
     * 檢查射擊馬達是否已達到目標速度
     * 
     * @param targetRPS 欲檢查的目標 RPS
     * @return 若當前速度在誤差範圍內則回傳 true，否則回傳 false
     */
    public boolean isAtSpeed(double targetRPS) {
        double currentRPS = LTshooterMotor.getVelocity().getValueAsDouble();
        // 允許 2.0 RPS 的誤差範圍
        return Math.abs(currentRPS - targetRPS) < 2.0;
    }

    /**
     * 停止所有射擊馬達
     */
    public void stop() {
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(0));
    }
}
