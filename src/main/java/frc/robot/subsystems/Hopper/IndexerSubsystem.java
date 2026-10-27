package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.kindexerId);
    private final VelocityVoltage indexerVelocityVoltage = new VelocityVoltage(0);
    // 目標轉速 (馬達軸 RPS，沒有設 SensorToMechanismRatio)
    private double targetIndexerRPS = 0.0;

    public IndexerSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Brake：停止時立刻停住，避免球靠慣性繼續被送進飛輪
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = IndexerConstants.kindexerkP;
        config.Slot0.kS = IndexerConstants.kindexerkS;
        config.Slot0.kV = IndexerConstants.kindexerkV;
        // 電流限制：Supply 保護電池；Stator 限制扭力 (太低時速度環會追不上目標)
        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kIndexerSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.kIndexerSupplyCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = IndexerConstants.kIndexerStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        indexerMotor.getConfigurator().apply(config);
    }

    // 指令的 execute() 會每 20 ms 呼叫，所以這裡只設目標、不印訊息；狀態看 SmartDashboard
    public void runindexer() {
        targetIndexerRPS = IndexerConstants.kindexerRPS;
    }

    public void stop() {
        targetIndexerRPS = 0.0;
    }

    public boolean isConnected() {
        return indexerMotor.isConnected();
    }

    @Override
    public void periodic() {
        indexerMotor.setControl(indexerVelocityVoltage.withVelocity(targetIndexerRPS));

        SmartDashboard.putNumber("Indexer/Target RPS", targetIndexerRPS);
        SmartDashboard.putNumber("Indexer/Velocity RPS", indexerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Stator Current", indexerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Indexer/Connected", isConnected());
    }
}
