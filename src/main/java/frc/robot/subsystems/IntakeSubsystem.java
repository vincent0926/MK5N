package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * 進件子系統 (IntakeSubsystem)
 * 負責控制進件齒條的伸出與收回。
 */
public class IntakeSubsystem extends SubsystemBase {

    /** 控制進件機構伸縮的馬達 */
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kFintakeId);

    /** 控制馬達位置的電壓輸出物件 */
    private final PositionVoltage intakePositionVoltage = new PositionVoltage(0);

    // 起始位置為機器內（收回），預設狀態設為 false
    private boolean isOut = false;

    /** 目前的目標位置 (rotations)，由 periodic() 持續送出 */
    private double targetRotations = 0.0;

    /**
     * 進件子系統建構子
     * 初始化馬達設定、PID 參數及電流限制。
     */
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
        intakeConfig.CurrentLimits.StatorCurrentLimit = 30; // 轉子電流防護
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        intakeConfig.Feedback.SensorToMechanismRatio = 1.0;

        intakeMotor.getConfigurator().apply(intakeConfig);

        // 起始位置為收回（0 公尺），告知馬達目前位置為 kintakein
        double initialRotations = IntakeConstants.kintakein * IntakeConstants.kRotationsPerMeter;

        intakeMotor.setPosition(initialRotations);
        targetRotations = initialRotations;
    }


    /**
     * 設定進件機構的目標位置
     * 
     * @param targetMeters 目標位置（單位：公尺）
     */
    public void setIntakePositionVoltage(double targetMeters) {    
        System.out.println("[IntakeSubsystem] setIntakePositionVoltage() called! Target meters=" + targetMeters);
        // 將公尺換算為馬達目標圈數
        targetRotations = targetMeters * IntakeConstants.kRotationsPerMeter;
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

    /**
     * 明確伸出 Intake（供自動階段 Named Command 使用）
     * 不依賴 isOut 旗標，直接下達伸出位置指令
     */
    public void extend() {
        System.out.println("[IntakeSubsystem] extend() called!");
        setIntakePositionVoltage(IntakeConstants.kintakeout);
        isOut = true;
    }

    /**
     * 明確收回 Intake（供自動階段 Named Command 使用）
     * 不依賴 isOut 旗標，直接下達收回位置指令
     */
    public boolean isConnected() { return intakeMotor.isConnected(); }

    /**
     * 不依賴 isOut 旗標，直接下達收回位置指令
     */
    public void retract() {
        System.out.println("[IntakeSubsystem] retract() called!");
        setIntakePositionVoltage(IntakeConstants.kintakein);
        isOut = false;
    }

    /**
     * 週期性更新函式
     * 定期將 Intake 狀態與位置推送到 SmartDashboard 上供觀察。
     */
    @Override
    public void periodic() {
        // 每 20ms 持續送出位置控制指令，避免 Phoenix 6 Motor Safety 超時
        intakeMotor.setControl(intakePositionVoltage.withPosition(targetRotations));

        // 將 Intake 狀態推送到 SmartDashboard
        SmartDashboard.putBoolean("Intake/伸出中", isOut);
        SmartDashboard.putString("Intake/狀態", isOut ? "✅ 伸出" : "🔴 收回");
        SmartDashboard.putNumber("Intake/位置(公尺)",
                intakeMotor.getPosition().getValueAsDouble() / IntakeConstants.kRotationsPerMeter);
    }
}
