package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OrbitConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class OrbitSubsystem extends SubsystemBase {

    // 下馬達是主馬達，上馬達跟隨
    private final TalonFX uorbitMotor = new TalonFX(OrbitConstants.kuorbitId);
    private final TalonFX dorbitMotor = new TalonFX(OrbitConstants.kdorbitId);

    // Phoenix 6 控制請求預設 EnableFOC = true，需 Pro 授權，所以跟底盤共用同一個開關
    private final VelocityVoltage orbitVelocityVoltage =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(Constants.Swerve.UseFocControl);
    // 停止時直接放開 (Coast)，不用速度環硬煞到 0
    private final NeutralOut orbitNeutral = new NeutralOut();

    // 目標轉速 (馬達軸 RPS，沒有設 SensorToMechanismRatio)
    private double targetOrbitRPS = 0.0;

    public OrbitSubsystem() {
        TalonFXConfiguration orbitconfig = new TalonFXConfiguration();
        orbitconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        orbitconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        orbitconfig.Slot0.kP = OrbitConstants.korbitkP;
        orbitconfig.Slot0.kS = OrbitConstants.korbitkS;
        orbitconfig.Slot0.kV = OrbitConstants.korbitkV;

        orbitconfig.CurrentLimits.SupplyCurrentLimit = OrbitConstants.kOrbitSupplyCurrentLimit;
        orbitconfig.CurrentLimits.SupplyCurrentLimitEnable = OrbitConstants.kOrbitSupplyCurrentLimitEnable;
        orbitconfig.CurrentLimits.StatorCurrentLimit = 40.0;
        orbitconfig.CurrentLimits.StatorCurrentLimitEnable = true;

        uorbitMotor.getConfigurator().apply(orbitconfig);
        dorbitMotor.getConfigurator().apply(orbitconfig);
        uorbitMotor.setControl(new Follower(dorbitMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runorbit() {
        targetOrbitRPS = OrbitConstants.korbitRPS;
    }

    public void norunorbit() {
        targetOrbitRPS = -OrbitConstants.korbitRPS;
    }

    public void stop() {
        targetOrbitRPS = 0.0;
    }

    public boolean isDConnected() {
        return dorbitMotor.isConnected();
    }

    @Override
    public void periodic() {
        if (targetOrbitRPS == 0.0) {
            dorbitMotor.setControl(orbitNeutral);
        } else {
            dorbitMotor.setControl(orbitVelocityVoltage.withVelocity(targetOrbitRPS));
        }

        SmartDashboard.putNumber("Orbit/Target RPS", targetOrbitRPS);
        SmartDashboard.putNumber("Orbit/Velocity RPS", dorbitMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/U Stator Current", uorbitMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Orbit/D Stator Current", dorbitMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Orbit/D Connected", isDConnected());
    }

}
