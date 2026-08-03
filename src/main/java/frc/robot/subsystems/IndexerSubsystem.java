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

/**
 * 索引器子系統 (IndexerSubsystem)
 * 負責控制軌道 (Orbit) 與索引器 (Indexer) 馬達，用於傳輸與定位物件。
 */
public class IndexerSubsystem extends SubsystemBase {

    /** 上軌道馬達 */
    private final TalonFX uorbitMotor = new TalonFX(OrbitConstants.kuorbitId);
    /** 下軌道馬達 */
    private final TalonFX dorbitMotor = new TalonFX(OrbitConstants.kdorbitId);
    /** 索引器馬達 */
    private final TalonFX indexerMotor = new TalonFX(IndexerConstants.kindexerId);

    /** 索引器電壓輸出控制 */
    private final VoltageOut indexerVoltageOut = new VoltageOut(0);
    /** 軌道電壓輸出控制 */
    private final VoltageOut orbiVoltageOut = new VoltageOut(0);

    /**
     * 索引器子系統建構子
     * 初始化並配置馬達參數 (如中立模式、電流限制與反轉設定)。
     */
    public IndexerSubsystem() {
        // 配置索引器 (Indexer) 馬達參數
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        indexerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kIndexerSupplyCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.kIndexerSupplyCurrentLimitEnable;
        indexerConfig.CurrentLimits.StatorCurrentLimit = 40;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // 配置軌道 (Orbit) 馬達參數
        TalonFXConfiguration orbitConfig = new TalonFXConfiguration();
        orbitConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        orbitConfig.CurrentLimits.SupplyCurrentLimit = OrbitConstants.kOrbitSupplyCurrentLimit;
        orbitConfig.CurrentLimits.SupplyCurrentLimitEnable = OrbitConstants.kOrbitSupplyCurrentLimitEnable;
        orbitConfig.CurrentLimits.StatorCurrentLimit = 40;
        orbitConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        orbitConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // 應用配置至馬達
        uorbitMotor.getConfigurator().apply(orbitConfig);
        dorbitMotor.getConfigurator().apply(orbitConfig);
        indexerMotor.getConfigurator().apply(indexerConfig);

        // 讓下軌道馬達跟隨上軌道馬達並對齊
        dorbitMotor.setControl(new Follower(uorbitMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    }

    /**
     * 啟動索引器與軌道馬達
     * 依照設定的電壓常數給予輸出，以傳輸物件。
     */
    public void runindexer() {
        uorbitMotor.setControl(indexerVoltageOut.withOutput(OrbitConstants.korbitVoltage));
        indexerMotor.setControl(orbiVoltageOut.withOutput(IndexerConstants.kindexerVoltage));

    }

    /**
     * 停止所有索引器與軌道馬達的輸出
     */
    public void stop() {
        uorbitMotor.setControl(indexerVoltageOut.withOutput(0.0));
        indexerMotor.setControl(orbiVoltageOut.withOutput(0.0));
    }

}
