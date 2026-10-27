package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Constants 類別提供了機器人各子系統所需的所有常數定義，
 * 包括馬達 ID、PID 參數、機構尺寸以及各種限制等。
 */
public final class Constants {

  // 主迴圈週期（TimedRobot 預設 20 ms）
  public static final float CycleTimeSec = 0.02f;

  // 萬向輪底盤 (Swerve)
  public final class SwerveConstants {

    public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(22.75);// 左到右 距離
    public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(20.75);// 前到後 距輪
    public static final double kDriveGearRatio = 5.27; // L3 驅動齒比
    public static final double kSteerGearRatio = 287.0 / 11.0;// 轉向齒比
    public static final double turnSpeed = 3;
    // 以下四個給 Swerve2 用 (舊式 WPILib 底盤)
    public static final double kMaxSpeed = 6.06; // 模組最高速度 (m/s)，desaturateWheelSpeeds 用
    public static final double kMaxAccel = Swerve.MaxForwardAccelMps2; // 遙控平移加速度上限 (m/s²)
    public static final double kHeadingHoldP = Swerve.HeadingHoldP; // 定向保持 P (rad/s per rad)
    public static final double kHeadingHoldOmegaDeadband = Swerve.HeadingHoldOmegaDeadband; // rad/s
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
    public static final int kFRSteerId = 46;
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
    public static final double kSteerkP = 2.0000 * (2 * Math.PI);
    public static final double kSteerkI = 0.0000 * (2 * Math.PI);
    public static final double kSteerkD = 0.0000 * (2 * Math.PI);

    // 驅動 (Drive) PID 與前饋 (Feedforward) 控制參數
    public static final double kWheelRadius = edu.wpi.first.math.util.Units.inchesToMeters(2.0);// 輪子半徑2inch
    public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;// 輪子周長
    public static final double kDrivekP = 0.0021 * kWheelCircumference;
    public static final double kDrivekI = 0.0;
    public static final double kDrivekD = 0.0;
    public static final double kDrivekS = 0.18; // 克服靜摩擦力
    public static final double kDrivekV = 2.35; // 速度常數
    public static final double kDrivekA = 0.05; // 加速度常數

    public static final double kDriveSupplyCurrentLimit = 50.0;// 電流限制
    public static final boolean kDriveSupplyCurrentLimitEnable = true;// 打開電流限制
    public static final double kDriveStatorCurrentLimit = 50.0;// 定子電流限制
    public static final boolean kDriveStatorCurrentLimitEnable = true;// 打開定子電流限制

    public static final double kSteerSupplyCurrentLimit = 40.0;// 電流限制
    public static final boolean kSteerSupplyCurrentLimitEnable = true;// 打開電流限制
    public static final double kSteerStatorCurrentLimit = 40.0;// 定子電流限制
    public static final boolean kSteerStatorCurrentLimitEnable = true;// 打開定子電流限制

  }

  // 射出機構 (Shooter) 參數
  public final class ShooterConstants {
    public static final int kLTShooterId = 9;
    public static final int kLDShooterId = 10;
    public static final int kRTShooterId = 11;
    public static final int kRDShooterId = 12;

    public static final double kShooterkP = 0.95;
    public static final double kShooterkI = 0.0;
    public static final double kShooterkD = 0;
    public static final double kShooterkS = 0.055;
    public static final double kShooterkV = 0.28;
    public static final double kShooterkA = 0.010;

    public static final double kshooterVelocityVoltage = 80.0;

    public static final double kShooterGearRatio = 1.0;

    public static final double kShooterSupplyCurrentLimit = 30.0;
    public static final boolean kShooterSupplyCurrentLimitEnable = true;

  }

  // 仰角機構 (Hood) 參數
  public final class HoodConstants {
    public static final int khoodId = 15;

    public static final double khoodkP = 65.0;
    public static final double khoodkI = 0.0;
    public static final double khoodkD = 0.0;
    public static final double khoodkS = 0.6;
    public static final double khoodkG = 2.0;

    // 角度都是「相對起始位置」：0 度 = 開機時的位置
    public static final double khoodMinAngle = 0;
    public static final double khoodMaxAngle = 50.0;

    // 起始位置的實際仰角 (度)，只用來顯示實際角度
    public static final double khoodStartAngle = 12.5;

    // 齒比 馬達:hood = 59:4，hood 轉 1 圈馬達轉 14.75 圈
    public static final double kHoodGearRatio = 59.0 / 4.0;

    // Motion Magic (馬達端單位)
    public static final double kHoodCruiseVelocity = 2.0; // 馬達 RPS，約 hood 48.8 度/秒
    public static final double kHoodAcceleration = 4.0; // 馬達 RPS/s，約 hood 97.6 度/秒²
    public static final double kHoodJerk = 20.0; // 馬達 RPS/s²

    public static final double kHoodSupplyCurrentLimit = 40;
    public static final boolean kHoodSupplyCurrentLimitEnable = true;
  }

  // 輸彈機構 (Indexer) 參數
  public static class IndexerConstants {

    public static final int kindexerId = 31;
    public static final double kIndexerSupplyCurrentLimit = 30.0;// 電流限制
    public static final boolean kIndexerSupplyCurrentLimitEnable = true;// 打開電流限制
    public static final double kIndexerStatorCurrentLimit = 20.0;// Stator 電流限制 (A)
    public static final double kindexerRPS = 40.0;// 馬達軸 RPS
    public static final double kindexerkP = 0.5;// V / (rps 誤差)
    public static final double kindexerkS = 0.5;// V
    public static final double kindexerkV = 0.12;// V/rps，Kraken/Falcon 理論值約 12 V / 100 rps (原 4.8 會讓輸出一直貼 12 V)
  }

  // 軌道機構 (Orbit) 參數
  public static class OrbitConstants {

    public static final int kuorbitId = 14;
    public static final int kdorbitId = 13;
    public static final double kOrbitSupplyCurrentLimit = 30.0;// 電流限制
    public static final boolean kOrbitSupplyCurrentLimitEnable = true;// 打開電流限制
    public static final double korbitRPS = 40.0;// 馬達軸 RPS (原本 10 V 空載約 82 rps)
    public static final double korbitkP = 0.7;// V / (rps 誤差)
    public static final double korbitkS = 0.3;// V
    public static final double korbitkV = 0.12;// V/rps，理論值約 12 V / 100 rps (原 4.8 會讓輸出一直貼 12 V)

  }

  // 進件機構 (Intake) 參數
  public static class IntakeConstants {

    public static final int kFintakeId = 30;

    public static final double kintakeGearRatio = 12;
    public static final double kintakePitChcircleDiameter = edu.wpi.first.math.util.Units.inchesToMeters(3);
    public static final double kintakeMetersPerRotorRotation = (1.0 / kintakeGearRatio)
        * (kintakePitChcircleDiameter * Math.PI);// 齒比*節圓值周長
    public static final double kRotationsPerMeter = 1.0 / kintakeMetersPerRotorRotation;// 齒條走 1 公尺，馬達要轉幾圈

    public static final double kintakein = 0.0;//
    public static final double kintakeout = 0.28;//

    public static final double kintakekP = 0.8;
    public static final double kintakekI = 0.0;
    public static final double kintakekD = 0.0;
    public static final double kintakeks = 0.07;
    public static final double kintakekg = 0.04;

    public static final double kintakeSupplyCurrentLimit = 30;
    public static final boolean kintakeSupplyCurrentLimitEnable = true;

  }

  // 滾輪進件機構 (Roller Intake) 參數
  public static class RollerIntakeConstants {
    public static final int kFrollerintake = 45;
    public static final int kRrollerintake = 41;
    public static final double krollerVoltage = 5.5;
    public static final double krollerSupplyCurrentLimit = 30.0;// 電流限制
    public static final boolean krollerSupplyCurrentLimitEnable = true;// 打開電流限制

  }

  // 視覺系統 (Vision) 參數
  public static class VisionConstants {
    public static final String kLimelightFront = "limelight-up";
    public static final String kLimelightLeft = "limelight-left";
    public static final String kLimelightRight = "limelight-right";

    // 2026 REBUILT 場地 HUB 中心座標 (公尺) - 以 WPI Blue 原點為基準
    // Blue Alliance Hub: X = 4.626 m, Y = 4.035 m
    public static final Translation2d kBlueHubCenter = new Translation2d(4.626, 4.035);
    // Red Alliance Hub: X = 11.915 m, Y = 4.035 m
    public static final Translation2d kRedHubCenter = new Translation2d(11.915, 4.035);

    // 2026 REBUILT 場地 HUB AprilTag ID 清單 (四個面全部標籤)
    public static final int[] kRedHubTagIds = { 2, 3, 4, 5, 8, 9, 10, 11 };
    public static final int[] kBlueHubTagIds = { 18, 19, 20, 21, 24, 25, 26, 27 };

    public static final int[] kBlueReturnTags = { 17, 12 };

    // 視覺位姿過濾 (MegaTag1)：單一 tag 太遠時位置會跳；朝向只相信多 tag 的結果
    public static final double kSingleTagMaxDistM = 3.0;
    public static final int kHeadingMinTagCount = 2;
    public static final double kHeadingMaxTagDistM = 4.0;
  }

  public static class AimConstants {

    // 預設的仰角增加度數 (補償值，可依需求調整)
    public static final double kElevationOffset = 0.0;
    // 距離 (公尺) 對應 仰角 (度)
    public static final double[][] kDistanceToAngleMap = {
        { 1.0, 2.0 },
        { 2.0, 5.0 },
        { 3.0, 15.0 },
        { 4.0, 32.5 }
    };

    // 距離 (公尺) 對應 轉速 (RPS, TalonFX 單位)
    public static final double[][] kDistanceToRPSMap = {
        { 1.0, 22.4 },
        { 2.0, 23.5 },
        { 3.0, 30.0 },
        { 4.0, 33.3 }
    };
  }

  // 中場回傳球的查表
  public static class ReturnConstants {

    // 中場回傳球的仰角
    public static final double[][] kReturnBalltohood = {
        { 1.0, 10 },
        { 2.0, 15 },
        { 3.0, 20 }
    };

    // 中場回傳球的RPS
    public static final double[][] kReturnBalltorps = {
        { 1.0, 20 },
        { 2.0, 20 },
        { 3.0, 20 }
    };
  }

  // 鳥蛋的高級swerve參數
  public static final class Swerve {
    public static final float InchesToMeters = 0.0254f;
    public static final float TrackWidthM = 22.75f * InchesToMeters;
    public static final float WheelBaseM = 20.75f * InchesToMeters;
    public static final float HalfTrackWidthM = TrackWidthM * 0.5f;
    public static final float HalfWheelBaseM = WheelBaseM * 0.5f;
    public static final float WheelRadiusM = 2.0f * InchesToMeters;
    public static final float WheelCircumferenceM = 2.0f * 3.1415927f * WheelRadiusM;
    public static final float RobotRadiusM = (float) Math
        .sqrt(HalfTrackWidthM * HalfTrackWidthM + HalfWheelBaseM * HalfWheelBaseM);
    public static final float RobotMassKg = 111.5f * 0.453592f;
    public static final float CoefficientOfFriction = 0.90f;
    public static final float MaxFrictionForceN = RobotMassKg * 9.81f * CoefficientOfFriction;
    public static final float MaxDriveAccelMps2 = MaxFrictionForceN / RobotMassKg;
    public static final float AccelerationSafetyFactor = 0.70f;
    public static final float MaxForwardAccelMps2 = MaxDriveAccelMps2 * AccelerationSafetyFactor;
    public static final float MaxFrontAccelMps2 = MaxForwardAccelMps2;
    public static final float MaxSideAccelMps2 = MaxForwardAccelMps2;
    public static final float MaxSkidAccelMps2 = MaxForwardAccelMps2;
    public static final float MaxDriveVelocityMps = 4.0f;
    public static final float MaxAngularVelocityRadPerSec = 10.0f;
    public static final float MaxTeleopAngularVelocityRadPerSec = 2.0f;
    public static final float DriveGearRatio = 5.27f; // SDS MK5n L3
    public static final float SteerGearRatio = 287.0f / 11.0f;
    public static final float DrivePositionCoefficient = 1.0f;
    public static final float SignalTimeoutSec = 0.018f;
    public static final double StatusSignalHz = 100.0;
    public static final double AbsoluteSignalHz = 4.0;
    public static final double UnusedSignalHz = 0.0;
    public static final double ControlRequestHz = 0.0;
    public static final float NormalCompensationK = 0.0f;
    public static final float HeadingHoldDelaySec = 0.35f;
    public static final float HeadingHoldP = 4.0f;
    public static final float HeadingHoldOmegaDeadband = 0.0001f;
    public static final float ModuleHoldSpeedMps = 0.02f;
    // 視覺融合：每次修正往視覺位姿靠近的比例，以及超過這個距離 (m) 的視覺資料視為離群值丟掉
    public static final float VisionBlendGain = 0.1f;
    public static final float VisionMaxJumpM = 2.0f;
    // 視覺修正朝向：每次往視覺朝向靠近的比例；誤差超過 15° 視為離群值 (起點擺歪不會超過這麼多)
    public static final float VisionHeadingGain = 0.1f;
    public static final float VisionHeadingMaxErrorRad = (float) java.lang.Math.toRadians(15.0);
    // 位置融合：轉太快時畫面糊掉、MegaTag1 會跳，超過這個角速度不用
    public static final double VisionMaxYawRateDegPerSec = 60.0;
    // 朝向修正只在車子停住時使用 (朝向沒有延遲補償，轉動中會不準)
    public static final float VisionStationarySpeedMps = 0.05f;
    public static final float VisionStationaryYawRateRadPerSec = 0.05f;
    // FOC 需要 Phoenix Pro 授權；舊版底盤沒有開 FOC，沒確認每顆馬達都有授權前維持 false
    public static final boolean UseFocControl = false;

    public static final int FrontLeftDriveId = 6;
    public static final int FrontLeftSteerId = 5;
    public static final int FrontLeftEncoderId = 3;

    public static final int FrontRightDriveId = 2;
    public static final int FrontRightSteerId = 46;
    public static final int FrontRightEncoderId = 1;

    public static final int BackLeftDriveId = 8;
    public static final int BackLeftSteerId = 7;
    public static final int BackLeftEncoderId = 4;

    public static final int BackRightDriveId = 4;
    public static final int BackRightSteerId = 3;
    public static final int BackRightEncoderId = 2;

    public static final int PigeonId = 11;
    public static final float PigeonMountPoseYawDeg = 0.0f;
    public static final float PigeonMountPosePitchDeg = 0.0f;
    public static final float PigeonMountPoseRollDeg = 0.0f;
    public static final float PigeonGyroScalarZ = 1.0f;
    public static final float FrontLeftMagnetOffset = -0.258544921875f;
    public static final float FrontRightMagnetOffset = 0.289794921875f;
    public static final float BackLeftMagnetOffset = 0.30126953125f;
    public static final float BackRightMagnetOffset = 0.19580078125f;
    public static final InvertedValue FrontLeftDriveInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue FrontRightDriveInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BackLeftDriveInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue BackRightDriveInverted = InvertedValue.Clockwise_Positive;
    public static final float DriveP = 0.0021f * WheelCircumferenceM;
    public static final float DriveI = 0.0f;
    public static final float DriveD = 0.0f;
    public static final float DriveS = 0.18f;
    public static final float DriveV = 2.35f * WheelCircumferenceM;
    public static final float DriveA = 0.05f * WheelCircumferenceM;
    public static final float SteerP = 11.0f * 2.0f * 3.1415927f;
    public static final float SteerI = 0.0f;
    public static final float SteerD = 0.0f;
    public static final float DriveSupplyCurrentLimitA = 60.0f;
    public static final float DriveStatorCurrentLimitA = 60.0f;
    public static final float SteerSupplyCurrentLimitA = 40.0f;
    public static final float SteerStatorCurrentLimitA = 60.0f;
    public static final float ClosedLoopRampPeriodSec = 0.020f;

    private Swerve() {
    }
  }

}