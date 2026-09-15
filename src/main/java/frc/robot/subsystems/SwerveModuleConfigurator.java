package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveConstants;

/**
 * 負責單一 Swerve 模組中各個硬體（驅動馬達、轉向馬達、絕對值編碼器）的初始化設定。
 */
public class SwerveModuleConfigurator {

  /**
   * 設定給定的馬達與編碼器參數，套用 PID、電流限制、與轉向偏移等設定。
   *
   * @param driveMotor           驅動馬達
   * @param steerMotor           轉向馬達
   * @param encoder              絕對值編碼器
   * @param steerOffsetRotations 轉向編碼器的偏移量（圈數）
   * @param driveInverted        驅動馬達的旋轉方向設定
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

    // Drive Motor 驅動馬達設定
    var driveConfig = new TalonFXConfiguration();
    
    // 設定馬達停止時為煞車模式 (Brake)，防止機器人在靜止時滑動
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // 設定驅動馬達是否反轉
    driveConfig.MotorOutput.Inverted = driveInverted;
   
    // 設定驅動馬達的 PIDF 參數
    driveConfig.Slot0.kP = SwerveConstants.kDrivekP;
    driveConfig.Slot0.kI = SwerveConstants.kDrivekI;
    driveConfig.Slot0.kD = SwerveConstants.kDrivekD;
    driveConfig.Slot0.kS = SwerveConstants.kDrivekS;
    driveConfig.Slot0.kV = SwerveConstants.kDrivekV;
    driveConfig.Slot0.kA = SwerveConstants.kDrivekA;

    // --- 【保護電力系統】電源端電流限制 (Supply Current) ---
    // 限制從配電盤 (PDH) 流進控制器的電流。
    // 目的：避免瞬間抽載過大導致保險絲跳脫 (Breaker Tripping) 或電池電壓驟降 (Brownout)。
    driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kDriveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.kDriveSupplyCurrentLimitEnable;

    // --- 【保護馬達與機械】定子端電流限制 (Stator Current) ---
    // 限制真正流進馬達內部線圈的電流 (與馬達產生的扭力直接成正比)。
    // 目的：防止馬達在堵轉 (如推牆) 時過熱燒毀，並限制最大扭力以避免齒輪崩壞或輪胎打滑
    driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.kDriveStatorCurrentLimitEnable;

    // 設定驅動機構的齒輪比
    driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;

    // Steer Motor 轉向馬達設定
    var steerConfig = new TalonFXConfiguration();
    // 設定停止時為煞車模式
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // 設定轉向馬達方向，逆時針為正
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // 設定轉向馬達的 PID 參數
    steerConfig.Slot0.kP = SwerveConstants.kSteerkP;
    steerConfig.Slot0.kI = SwerveConstants.kSteerkI;
    steerConfig.Slot0.kD = SwerveConstants.kSteerkD;

    // 設定轉向馬達的電源端電流限制
    steerConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kSteerSupplyCurrentLimit;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.kSteerSupplyCurrentLimitEnable;
    steerConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kSteerStatorCurrentLimit;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.kSteerStatorCurrentLimitEnable;

    // --- 閉迴路感測器設定 ---
    // 將轉向位置的回饋來源，綁定為外部的 CANcoder，而非馬達內部編碼器 (消除齒輪間隙造成的誤差)
    steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // 設定轉向機構的齒輪比
    steerConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;

    // 啟用連續環繞功能：確保轉向角度從 359 度跨越到 0 度 (或反之) 時，馬達會走最短路徑，而不會瘋狂反轉一大圈
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // 將設定好的 driveConfig 參數透過 CAN Bus 正式寫入驅動馬達硬體中
    driveMotor.getConfigurator().apply(driveConfig);
    steerMotor.getConfigurator().apply(steerConfig);

    // 延遲一小段時間確保設定生效
    Timer.delay(0.05);

    // 模組設定程序執行完畢，回傳 true 代表流程走完
    return true;
  }

}
