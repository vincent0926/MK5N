package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OrbitConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX uorbitMotor = new TalonFX(OrbitConstants.kuorbitId);
    private final TalonFX dorbitMotor = new TalonFX(OrbitConstants.kdorbitId);
    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.kindexerId);

    private final VoltageOut indexerVoltageOut = new VoltageOut(0);
    private final VoltageOut orbiVoltageOut = new VoltageOut(0);

    public IndexerSubsystem() {
        //indexer
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        indexerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kIndexerSupplyCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.kIndexerSupplyCurrentLimitEnable;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //orbit
        TalonFXConfiguration orbitConfig = new TalonFXConfiguration();
        orbitConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        orbitConfig.CurrentLimits.SupplyCurrentLimit = OrbitConstants.kOrbitSupplyCurrentLimit;
        orbitConfig.CurrentLimits.SupplyCurrentLimitEnable = OrbitConstants.kOrbitSupplyCurrentLimitEnable;
        orbitConfig.CurrentLimits.StatorCurrentLimit = 40;
        orbitConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        orbitConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        uorbitMotor.getConfigurator().apply(orbitConfig);
        dorbitMotor.getConfigurator().apply(orbitConfig);
        indexerMotor.getConfigurator().apply(indexerConfig);

        dorbitMotor.setControl(new Follower(uorbitMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    }

    public void runindexer() {
        uorbitMotor.setControl(indexerVoltageOut.withOutput(OrbitConstants.korbitVoltage));
        indexerMotor.setControl(orbiVoltageOut.withOutput(IndexerConstants.kindexerVoltage));

    }

    public void stop() {
        uorbitMotor.setControl(indexerVoltageOut.withOutput(0.0));
        indexerMotor.setControl(orbiVoltageOut.withOutput(0.0));
    }

}
