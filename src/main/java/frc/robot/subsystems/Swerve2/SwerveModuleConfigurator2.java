package frc.robot.subsystems.Swerve2;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveConstants;

/**
 * 負責單一 Swerve 模組中各個硬體（驅動馬達、轉向馬達、絕對值編碼器）的初始化設定。
 */
public class SwerveModuleConfigurator2 {

  /**
   * 設定編碼器偏移，並把 Swerve2 的 PID/前饋寫入馬達的 Slot1 (其餘馬達設定沿用 Swerve)。
   *
   * @param driveMotor           驅動馬達
   * @param steerMotor           轉向馬達
   * @param encoder              絕對值編碼器
   * @param steerOffsetRotations 轉向編碼器的偏移量（圈數）
   * @param driveInverted        驅動馬達的旋轉方向設定 (目前沿用 Swerve 的設定，未使用)
   * @return 設定完成後回傳 true
   */
  public static boolean configure(
      TalonFX driveMotor,
      TalonFX steerMotor,
      CANcoder encoder,
      double steerOffsetRotations,
      InvertedValue driveInverted) {
    // CANcoder 絕對值編碼器設定
    var coderConfig = new CANcoderConfiguration();
   
    // 設定感測器方向：逆時針為正
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
   
    // 設定磁性偏移量
    coderConfig.MagnetSensor.MagnetOffset = steerOffsetRotations;
    
    // 將設定套用到編碼器
    encoder.getConfigurator().apply(coderConfig);

    // 馬達和新的 Swerve 共用。Swerve 先建立，已經寫入完整設定 (反轉、齒比、CANcoder 回授、電流限制) 並把增益放在 Slot0。
    // 這裡如果再 apply 整份 TalonFXConfiguration，會把 Swerve 的 Slot0 蓋掉，自動路線就會用錯前饋。
    // 所以 Swerve2 只把自己的 PID/前饋寫進 Slot1 (部分寫入不會動到其他設定)，SwerveModule2 用 Slot1 下指令。
    // 注意：Swerve2 必須在 Swerve 之後建立，否則 Swerve 的整份設定會把 Slot1 清成 0。

    // 驅動馬達的 PIDF 參數
    var driveSlot = new Slot1Configs();
    driveSlot.kP = SwerveConstants.kDrivekP;
    driveSlot.kI = SwerveConstants.kDrivekI;
    driveSlot.kD = SwerveConstants.kDrivekD;
    driveSlot.kS = SwerveConstants.kDrivekS;
    driveSlot.kV = SwerveConstants.kDrivekV;
    driveSlot.kA = SwerveConstants.kDrivekA;

    // 轉向馬達的 PID 參數
    var steerSlot = new Slot1Configs();
    steerSlot.kP = SwerveConstants.kSteerkP;
    steerSlot.kI = SwerveConstants.kSteerkI;
    steerSlot.kD = SwerveConstants.kSteerkD;

    driveMotor.getConfigurator().apply(driveSlot);
    steerMotor.getConfigurator().apply(steerSlot);

    // 延遲一小段時間確保設定生效
    Timer.delay(0.05);

    // 模組設定程序執行完畢，回傳 true 代表流程走完
    return true;
  }

}

