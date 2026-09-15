package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.RollerIntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 滾輪進件子系統 (RollerIntakeSubsystem)
 * 負責控制滾輪馬達將物件吸入與吐出。
 */
public class RollerIntakeSubsystem extends SubsystemBase {
    
    /** 前方滾輪馬達 */
    private final TalonFX Frollintake = new TalonFX(RollerIntakeConstants.kFrollerintake);
    
    /** 後方滾輪馬達 */
    private final TalonFX Rrollintake = new TalonFX(RollerIntakeConstants.kRrollerintake);
    
    /** 滾輪馬達的電壓輸出控制物件 */
    private final VoltageOut rollerVoltageOut = new VoltageOut(0);

    /** 目前滾輪目標電壓 */
    private double targetVoltage = 0.0;

    /**
     * 滾輪進件子系統建構子
     * 初始化馬達設定、電流限制以及從動跟隨模式。
     */
    public RollerIntakeSubsystem() {
        // 滾輪馬達設定
        TalonFXConfiguration rollerintakeConfig = new TalonFXConfiguration();
        rollerintakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerintakeConfig.CurrentLimits.SupplyCurrentLimit = RollerIntakeConstants.krollerSupplyCurrentLimit;
        rollerintakeConfig.CurrentLimits.SupplyCurrentLimitEnable = RollerIntakeConstants.krollerSupplyCurrentLimitEnable;
        rollerintakeConfig.CurrentLimits.StatorCurrentLimit = 30;
        rollerintakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerintakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Frollintake.getConfigurator().apply(rollerintakeConfig);
        Rrollintake.getConfigurator().apply(rollerintakeConfig);
        
        // 設定後方滾輪為前方滾輪的反向從動模式
        Rrollintake.setControl(new Follower(Frollintake.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * 啟動滾輪以預設正轉電壓運作 (吸球)
     */
    public void runRollers() {
        System.out.println("[RollerIntakeSubsystem] runRollers() called! Setting target voltage to: " + RollerIntakeConstants.krollerVoltage);
        targetVoltage = RollerIntakeConstants.krollerVoltage;
    }

    /**
     * 【新增】啟動滾輪以負電壓反轉運作 (吐球)
     */
    public void reverseRollers() {
        System.out.println("[RollerIntakeSubsystem] reverseRollers() called! Setting target voltage to: " + (-RollerIntakeConstants.krollerVoltage));
        targetVoltage = -RollerIntakeConstants.krollerVoltage;
    }

    /**
     * 【新增】直接指定目標電壓
     */
    public void setVoltage(double voltage) {
        targetVoltage = voltage;
    }
    
    public boolean isFConnected() { return Frollintake.isConnected(); }
    public boolean isRConnected() { return Rrollintake.isConnected(); }

    /**
     * 停止滾輪運作
     */
    public void stop() {
        System.out.println("[RollerIntakeSubsystem] stop() called! Setting target voltage to 0.0");
        targetVoltage = 0.0;
    }

    /**
     * 每 20ms 持續送出控制指令，避免 Phoenix 6 Motor Safety 超時停止馬達
     */
    @Override
    public void periodic() {
        Frollintake.setControl(rollerVoltageOut.withOutput(targetVoltage));
    }
}