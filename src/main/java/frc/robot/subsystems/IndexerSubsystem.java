package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.kindexerId);
    private final VoltageOut indexerVoltageOut = new VoltageOut(0);

    private double targetIndexerVoltage = 0.0;

    public IndexerSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // 電流限制：關閉 Stator 電流限制，避免馬達啟動扭力不足被限制
        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kIndexerSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.kIndexerSupplyCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // 確保軟體軟極限 (Soft Limit) 關閉，避免硬體暫存舊數據擋住輸出
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        indexerMotor.getConfigurator().apply(config);
    }

    public void runindexer() {
        targetIndexerVoltage = IndexerConstants.kindexerVoltage;
        System.out.println("[Indexer] RUN Command Executed! Target Voltage: " + targetIndexerVoltage);
    }

    public void stop() {
        targetIndexerVoltage = 0.0;
        System.out.println("[Indexer] STOP Command Executed!");
    }

    public boolean isConnected() {
        return indexerMotor.isConnected();
    }

    @Override
    public void periodic() {
        indexerMotor.setControl(indexerVoltageOut.withOutput(targetIndexerVoltage));
        
        SmartDashboard.putNumber("Indexer/Target Voltage", targetIndexerVoltage);
        SmartDashboard.putNumber("Indexer/Stator Current", indexerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Indexer/Connected", isConnected());
    }
}