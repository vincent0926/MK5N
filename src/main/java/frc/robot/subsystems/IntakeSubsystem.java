package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kFintakeId);

    private final PositionVoltage intakePositionVoltage = new PositionVoltage(0);

    // 因為起始位置在機器外（伸出，即 0.3 公尺處），所以預設狀態設為 true
    private boolean isOut = true;

    public IntakeSubsystem() {

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = IntakeConstants.kintakekP;
        intakeConfig.Slot0.kI = IntakeConstants.kintakekI;
        intakeConfig.Slot0.kD = IntakeConstants.kintakekD;
        intakeConfig.Slot0.kS = IntakeConstants.kintakeks;
        intakeConfig.Slot0.kG = IntakeConstants.kintakekg;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kintakeSupplyCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.kintakeSupplyCurrentLimitEnable;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60; // 轉子電流防護
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeConfig.Feedback.SensorToMechanismRatio = 1.0;

        intakeMotor.getConfigurator().apply(intakeConfig);

        double initialRotations = IntakeConstants.kintakeout * IntakeConstants.kRotationsPerMeter;

        intakeMotor.setPosition(initialRotations);
    }


    public void setIntakePositionVoltage(double targetMeters) {    
        // 將公尺換算為馬達目標圈數
        double targetRotations = targetMeters * IntakeConstants.kRotationsPerMeter;
        // 執行 PID 位置控制
        intakeMotor.setControl(intakePositionVoltage.withPosition(targetRotations));
    }

    /**
     * 切換齒條的伸出與收回狀態（單鍵切換邏輯）
     */
    public void togglePosition() {
        if (isOut) {
            // 如果目前是在機器外（0.3m），就下達「收回」指令（走到 0.0m）
            setIntakePositionVoltage(IntakeConstants.kintakein);
        } else {
            // 如果目前是在機器內（0.0m），就下達「伸出」指令（走到 0.3m）
            setIntakePositionVoltage(IntakeConstants.kintakeout);
        }
        // 切換狀態旗標
        isOut = !isOut;
    }

    /**
     * 取得當前齒條是否在機器外（伸出）
     * 
     * @return true 代表在機器外，false 代表已收回
     */
    public boolean isOut() {
        return isOut;
    }

    //@Override
    //public void periodic() {
        // 週期性將數據上傳到 Driver Station 的 SmartDashboard，方便調機與監控
        // 這裡顯示的會是「馬達目前的總旋轉圈數」
       // SmartDashboard.putNumber("Intake/Current Motor Rotations", intakeMotor.getPosition().getValueAsDouble());
        // 換算回公尺顯示，方便直覺檢查數據是否正確
      //  SmartDashboard.putNumber("Intake/Current Position (Meters)",
      //          intakeMotor.getPosition().getValueAsDouble() / IntakeConstants.kRotationsPerMeter);
      //  SmartDashboard.putBoolean("Intake/Is Position Out",isOut);
    //}
}