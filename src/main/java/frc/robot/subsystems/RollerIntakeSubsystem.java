package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
 * 負責控制滾輪馬達將物件吸入。
 */
public class RollerIntakeSubsystem extends SubsystemBase {
    
    /** 前方滾輪馬達 */
    private final TalonFX Frollintake = new TalonFX(RollerIntakeConstants.kFrollerintake);
    
    /** 後方滾輪馬達 */
    private final TalonFX Rrollintake = new TalonFX(RollerIntakeConstants.kRrollerintake);
    
    /** 滾輪馬達的電壓輸出控制物件 */
    private final VoltageOut rollerVoltageOut = new VoltageOut(0);

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
        rollerintakeConfig.CurrentLimits.StatorCurrentLimit = 40;
        rollerintakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerintakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Frollintake.getConfigurator().apply(rollerintakeConfig);
        Rrollintake.getConfigurator().apply(rollerintakeConfig);
        
        // 設定後方滾輪為前方滾輪的反向從動模式
        Rrollintake.setControl(new Follower(Frollintake.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * 啟動滾輪以預設電壓運作
     */
    public void runRollers() {
        Frollintake.setControl(rollerVoltageOut.withOutput(RollerIntakeConstants.krollerVoltage));
    }
    
    /**
     * 停止滾輪運作
     */
    public void stop() {
        Frollintake.setControl(rollerVoltageOut.withOutput(0.0)); 
    }
}
