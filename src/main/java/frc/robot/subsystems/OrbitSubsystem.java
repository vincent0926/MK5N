package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OrbitConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class OrbitSubsystem extends SubsystemBase {

    private final TalonFX uorbitMotor = new TalonFX(OrbitConstants.kuorbitId);
    private final TalonFX dorbitMotor = new TalonFX(OrbitConstants.kdorbitId);

    private final VoltageOut uorbitVoltageOut = new VoltageOut(0);
    private final VoltageOut dorbitVoltageOut = new VoltageOut(0);
    private double targetorbitVoltage = 0.0;

    public OrbitSubsystem() {
        TalonFXConfiguration orbitconfig = new TalonFXConfiguration();
        orbitconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        orbitconfig.CurrentLimits.SupplyCurrentLimit = OrbitConstants.kOrbitSupplyCurrentLimit;
        orbitconfig.CurrentLimits.SupplyCurrentLimitEnable = OrbitConstants.kOrbitSupplyCurrentLimitEnable;
        orbitconfig.CurrentLimits.StatorCurrentLimit = 40.0;
        orbitconfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        orbitconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        uorbitMotor.getConfigurator().apply(orbitconfig);
        dorbitMotor.getConfigurator().apply(orbitconfig);
    }

    public void runorbit() {
        targetorbitVoltage = OrbitConstants.korbitVoltage;
        System.out.println("[Orbit] RUN Command Executed! Target Voltage: " + targetorbitVoltage);
    }
    public void norunorbit() {
        targetorbitVoltage = OrbitConstants.korbitVoltage * -1;
        System.out.println("[Orbit] NORUN Command Executed! Target Voltage: " + targetorbitVoltage);
    }

    public void stop() {
        targetorbitVoltage = 0.0;
        System.out.println("[Orbit] STOP Command Executed!");
    }

    public boolean isUConnected() { return uorbitMotor.isConnected(); }
    public boolean isDConnected() { return dorbitMotor.isConnected(); }

    @Override
    public void periodic() {
        // 直接對兩顆 Orbit 馬達發送電壓指令，不依賴 Follower 模式
        uorbitMotor.setControl(uorbitVoltageOut.withOutput(targetorbitVoltage));
        dorbitMotor.setControl(dorbitVoltageOut.withOutput(targetorbitVoltage));
        
        SmartDashboard.putNumber("Orbit/Target Voltage", targetorbitVoltage);
        SmartDashboard.putNumber("Orbit/U Stator Current", uorbitMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/D Stator Current", dorbitMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Orbit/U Connected", isUConnected());
        SmartDashboard.putBoolean("Orbit/D Connected", isDConnected());
    }

}