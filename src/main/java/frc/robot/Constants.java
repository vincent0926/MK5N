package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Constants 類別提供了機器人各子系統所需的所有常數定義，
 * 包括馬達 ID、PID 參數、機構尺寸以及各種限制等。
 */
public final class Constants {

  // 萬向輪底盤 (Swerve)
  public final class SwerveConstants {

    public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(22.75);// 左到右 距離
    public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(20.75);// 前到後 距輪
    public static final double kDriveGearRatio = 4.71; // L3 驅動齒比
    public static final double kSteerGearRatio = 287.0 / 11.0;// 轉向齒比
    public static final double kMaxSpeed = 4;// 最大速度
    public static final double turnSpeed = 1;
    public static final InvertedValue kFLDriveInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kBLDriveInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kFRDriveInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kBRDriveInverted = InvertedValue.Clockwise_Positive;

    // Swerve 運動學 (SwerveDriveKinematics)
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // FL
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // FR
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // BL
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // BR
    );

    // 右前輪 (FR)
    public static final int kFRDriveId = 2;
    public static final int kFRSteerId = 1;
    public static final int kFREncoderId = 1;
    public static final double kFROffset = 0.289794921875;
    // 右後輪 (BR)
    public static final int kBRDriveId = 4;
    public static final int kBRSteerId = 3;
    public static final int kBREncoderId = 2;
    public static final double kBROffset = 0.19580078125;
    // 左後輪 (BL)
    public static final int kBLDriveId = 8;
    public static final int kBLSteerId = 7;
    public static final int kBLEncoderId = 4;
    public static final double kBLOffset = 0.30126953125;
    // 左前輪 (FL)
    public static final int kFLDriveId = 6;
    public static final int kFLSteerId = 5;
    public static final int kFLEncoderId = 3;
    public static final double kFLOffset = -0.258544921875;
    // 陀螺儀 (Pigeon)
    public static final int kPigeonId = 11;

    // 轉向 (Steer) PID 控制參數
    public static final double kSteerkP = 11.0000 * (2 * Math.PI);
    public static final double kSteerkI = 0.0000 * (2 * Math.PI);
    public static final double kSteerkD = 0.0000 * (2 * Math.PI);
    public static final double kSteerSupplyCurrentLimit = 40.0;// 電流限制
    public static final boolean kSteerSupplyCurrentLimitEnable = true;// 打開電流限制
    // 驅動 (Drive) PID 與前饋 (Feedforward) 控制參數
    public static final double kWheelRadius = edu.wpi.first.math.util.Units.inchesToMeters(2.0);// 輪子半徑2inch
    public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;// 輪子周長
    public static final double kDrivekP = 0.0021 * kWheelCircumference;
    public static final double kDrivekI = 0.0;
    public static final double kDrivekD = 0.0;
    public static final double kDrivekS = 0.18; // 克服靜摩擦力
    public static final double kDrivekV = 2.35; // 速度常數
    public static final double kDrivekA = 0.05; // 加速度常數

    public static final double kDriveSupplyCurrentLimit = 60.0;// 電流限制
    public static final boolean kDriveSupplyCurrentLimitEnable = true;// 打開電流限制

  }

  // 射出機構 (Shooter) 參數
  public final class ShooterConstants {
    public static final int kLTShooterId = 9;
    public static final int kLDShooterId = 10;
    public static final int kRTShooterId = 11;
    public static final int kRDShooterId = 12;

    public static final double kShooterkP = 0.05;
    public static final double kShooterkI = 0.0;
    public static final double kShooterkD = 0;
    public static final double kShooterkS = 0.45;
    public static final double kShooterkV = 0.12;
    public static final double kShooterkA = 0;

    public static final double kshooterVelocityVoltage = 30.0;

    public static final double kShooterGearRatio = 1.0;

    public static final double kShooterSupplyCurrentLimit = 100;
    public static final boolean kShooterSupplyCurrentLimitEnable = true;

  }
  // 仰角機構 (Hood) 參數
  public static final class HoodConstants {
    public static final int khoodId = 15;

    // 這裡調高 PID 參數，讓馬達有力氣對抗重力（直驅無齒輪箱改用 180.0）
    // 更新：因馬達震盪發熱，將 kP 暫時降至 50.0
    public static final double khoodkP = 70.0;
    public static final double khoodkI = 0.0;
    public static final double khoodkD = 0.0;
    public static final double khoodkS = 0.5;
    public static final double khoodkG = 1.5;

    public static final double khoodMinAngle = 0.0;
    public static final double khoodMaxAngle = 45.0;

    public static final double kOneMotor = 10.0/360.0;

    public static final double kHoodSupplyCurrentLimit = 40;
    public static final boolean kHoodSupplyCurrentLimitEnable = true;
  }

  // 輸彈機構 (Indexer) 參數
  public static class IndexerConstants {

    public static final int kindexerId = 0;
    public static final double kindexerVoltage = 4.0;
    public static final double kIndexerSupplyCurrentLimit = 40.0;// 電流限制
    public static final boolean kIndexerSupplyCurrentLimitEnable = true;// 打開電流限制
  }

  // 軌道機構 (Orbit) 參數
  public static class OrbitConstants {

    public static final int kuorbitId = 14;
    public static final int kdorbitId = 13;
    public static final double korbitVoltage = 4.0;
    public static final double kOrbitSupplyCurrentLimit = 40.0;// 電流限制
    public static final boolean kOrbitSupplyCurrentLimitEnable = true;// 打開電流限制

  }

  // 進件機構 (Intake) 參數
  public static class IntakeConstants {

    public static final int kFintakeId = 30;

    public static final double kintakeGearRatio = 12;
    public static final double kintakePitChcircleDiameter = edu.wpi.first.math.util.Units.inchesToMeters(3);
    public static final double kintakeMetersPerRotorRotation = (1.0 / kintakeGearRatio)
        * (kintakePitChcircleDiameter * Math.PI);// 齒比*節圓值周長
    public static final double kRotationsPerMeter = 1.0 / kintakeMetersPerRotorRotation;// 齒條走 1 公尺，馬達要轉幾圈

    public static final double kintakein = 0.1;// 收0cm
    public static final double kintakeout = 0.38;// 伸長30cm

    public static final double kintakekP = 0.6;
    public static final double kintakekI = 0.0;
    public static final double kintakekD = 0.0;
    public static final double kintakeks = 0.07;
    public static final double kintakekg = 0.04;

    public static final double kintakeSupplyCurrentLimit = 40;
    public static final boolean kintakeSupplyCurrentLimitEnable = true;

  }

  // 滾輪進件機構 (Roller Intake) 參數
  public static class RollerIntakeConstants {
    public static final int kFrollerintake = 45;
    public static final int kRrollerintake = 41;
    public static final double krollerVoltage = 7;
    public static final double krollerSupplyCurrentLimit = 40.0;// 電流限制
    public static final boolean krollerSupplyCurrentLimitEnable = true;// 打開電流限制

  }

  // 視覺系統 (Vision) 參數
  public static class VisionConstants {
    public static final String kLimelightFront = "limelight-up";
    public static final String kLimelightLeft = "limelight-left";
    public static final String kLimelightRight = "limelight-right";
  }

  public static class AimConstants {
    
     // 預設的仰角增加度數 (補償值，可依需求調整)
    public static final double kElevationOffset = 0.0;
    // 距離 (公尺) 對應 仰角 (度)
    public static final double[][] kDistanceToAngleMap = {
        {1.0, 5.0},
        {2.0, 15.0},
        {3.0, 25.0},
        {4.0, 35.0}
    };

    // 距離 (公尺) 對應 轉速 (RPS, TalonFX 單位)
    public static final double[][] kDistanceToRPSMap = {
        {1.0, 40.0},
        {2.0, 50.0},
        {3.0, 60.0},
        {4.0, 70.0}
    };
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }
}
