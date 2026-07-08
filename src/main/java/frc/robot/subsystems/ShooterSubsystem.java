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

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX LTshooterMotor = new TalonFX(ShooterConstants.kLTShooterId);
    private final TalonFX LDshooterMotor = new TalonFX(ShooterConstants.kLDShooterId);
    private final TalonFX RTshooterMotor = new TalonFX(ShooterConstants.kRTShooterId);
    private final TalonFX RDshooterMotor = new TalonFX(ShooterConstants.kRDShooterId);

    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);
    private final DutyCycleOut shooterDutyCycleOut = new DutyCycleOut(0);

    public ShooterSubsystem() {

        TalonFXConfiguration shooterconfig = new TalonFXConfiguration();

        shooterconfig.Slot0.kP = ShooterConstants.kShooterkP;
        shooterconfig.Slot0.kI = ShooterConstants.kShooterkI;
        shooterconfig.Slot0.kD = ShooterConstants.kShooterkD;
        //shooterconfig.Slot0.kS = ShooterConstants.kShooterkS;
        //shooterconfig.Slot0.kV = ShooterConstants.kShooterkV;
        //shooterconfig.Slot0.kA = ShooterConstants.kShooterkA;

        shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterconfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterSupplyCurrentLimit;
        shooterconfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.kShooterSupplyCurrentLimitEnable;

        shooterconfig.CurrentLimits.StatorCurrentLimit = 60;
        shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;

        LTshooterMotor.getConfigurator().apply(shooterconfig);
        LDshooterMotor.getConfigurator().apply(shooterconfig);
        RTshooterMotor.getConfigurator().apply(shooterconfig);
        RDshooterMotor.getConfigurator().apply(shooterconfig);

        LDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        RTshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        RDshooterMotor.setControl(new Follower(LTshooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    public void runshooter() {
        //記得改PID數值
        LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(ShooterConstants.kshooterVelocityVoltage));
    
        // LTshooterMotor.setControl(new DutyCycleOut(0.6));

    }

    public void stop() {
         LTshooterMotor.setControl(shooterVelocityVoltage.withVelocity(0));
       // LTshooterMotor.setControl(new DutyCycleOut(0.0));
    }
}
