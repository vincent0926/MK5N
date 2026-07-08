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

public class RollerIntakeSubsystem extends SubsystemBase {
    private final TalonFX Frollintake = new TalonFX(RollerIntakeConstants.kFrollerintake);
    private final TalonFX Rrollintake = new TalonFX(RollerIntakeConstants.kRrollerintake);
    private final VoltageOut rollerVoltageOut = new VoltageOut(0);

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
        Rrollintake.setControl(new Follower(Frollintake.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void runRollers() {
        //Frollintake.setControl(rollerVoltageOut.withOutput(RollerIntakeConstants.krollerVoltage));
        Frollintake.setControl(new DutyCycleOut(0.5));
    }
    public void stop() {
    Frollintake.setControl(rollerVoltageOut.withOutput(0.0)); 
}
}
