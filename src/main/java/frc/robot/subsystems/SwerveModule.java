package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final int ModuleIndex;
    private final TalonFX DriveMotor;
    private final TalonFX SteerMotor;
    private final CANcoder AbsoluteEncoder;

    // 建立驅動馬達的控制指令物件：使用「速度與電壓」閉迴路控制 (預設目標為 0)
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    // 建立轉向馬達的控制指令物件：使用「位置與電壓」閉迴路控制 (預設目標為 0)
    private final PositionVoltage steerPosition = new PositionVoltage(0);

    // 紀錄最後一次的目標角度，避免機器人靜止時，搖桿回正導致輪子自動轉回 0 度而產生抖動
    private Rotation2d mLastAngle;

    public SwerveModule(int index, int driveId, int steerId, int encoderId, double offsetRotations, InvertedValue driveInverted) {

        // 將傳入的模組編號存入類別變數
        this.ModuleIndex = index;
        // 根據傳入的 CAN ID 實例化驅動馬達
        this.DriveMotor = new TalonFX(driveId);
        // 根據傳入的 CAN ID 實例化轉向馬達
        this.SteerMotor = new TalonFX(steerId);
        // 根據傳入的 CAN ID 實例化編碼器
        this.AbsoluteEncoder = new CANcoder(encoderId);
        // 呼叫我們之前寫好的 Configurator 類別，把上面實例化的硬體傳進去進行 PID 與電流限制等基礎配置
        SwerveModuleConfigurator.configure(DriveMotor, SteerMotor, AbsoluteEncoder, offsetRotations,driveInverted );
        // 初始化最後一次的角度：讀取轉向馬達當下的位置 (以圈數 Rotations 為單位) 並轉換為 Rotation2d 物件
        this.mLastAngle = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // --- 防抖動設計 (Anti-Jitter / Deadband) ---
        // 檢查目標速度是否小於 0.02 公尺/秒 (代表搖桿幾乎沒有推，機器人應該要靜止)
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.02) {
            DriveMotor.setControl(driveVelocity.withVelocity(0));
            SteerMotor.setControl(steerPosition.withPosition(mLastAngle.getRotations()));
            return;
        }

        // --- 讀取當前角度 ---
        // 從轉向馬達讀取現在真實的角度 (因為前面 Config 時有綁定 RemoteCANcoder，所以這裡抓到的是精準數值)
        Rotation2d currentAngle = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());

        // --- Swerve 狀態優化 (Swerve Optimize) 核心邏輯 ---
        // 這是 WPILib 提供的重要功能：避免輪胎旋轉超過 90 度！
        // 如果現在輪胎朝前(0度)，但目標要向後走(180度)，與其讓轉向馬達辛苦轉 180 度，
        // optimize 會聰明地讓輪胎保持 0 度，但把驅動馬達的「速度反轉」，達成一樣的移動效果！
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);

        // --- 單位轉換 (從 WPILib 的公制單位 -> TalonFX 的圈數單位) ---
        // 將優化後的「目標公尺/秒」除以「輪胎圓周長 (Constants.kWheelCircumference)」，算出每秒需要轉幾圈 (Rotations
        // Per Second, RPS)
        double wheelRotperSec = optimizedState.speedMetersPerSecond / SwerveConstants.kWheelCircumference;

        // --- 發送控制指令給馬達 ---
        // 將換算好的 RPS 目標速度發送給驅動馬達
        DriveMotor.setControl(driveVelocity.withVelocity(wheelRotperSec));

        // 將優化後的目標角度 (以圈數為單位) 發送給轉向馬達
        SteerMotor.setControl(steerPosition.withPosition(optimizedState.angle.getRotations()));

        // 更新最後一次的角度紀錄，供下一次靜止時使用
        this.mLastAngle = optimizedState.angle;
    }

    /**
     * 獲取模組目前的「位置資訊」 (Odometry 哩程計專用)
     * 用來告訴系統：這顆輪子總共往前滾了多遠？現在指著什麼方向？
     */
    public SwerveModulePosition getPosition() {
        // 讀取驅動馬達總共轉了幾圈，乘上輪胎圓周長，算出實際在地毯上行駛了幾公尺
        double distanceMeters = DriveMotor.getPosition().getValueAsDouble() * SwerveConstants.kWheelCircumference;
        // 讀取轉向馬達當前的角度
        Rotation2d angle = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());
        // 將行駛距離與角度包裝成 SwerveModulePosition 物件並回傳
        return new SwerveModulePosition(distanceMeters, angle);
    }

    /**
     * 獲取模組目前的「狀態資訊」 (Kinematics 運動學專用)
     * 用來告訴系統：這顆輪子現在當下的「瞬時速度」是多少？指著什麼方向？
     */
    public SwerveModuleState getState() {
        // 讀取驅動馬達當前的轉速 (RPS)，乘上輪胎圓周長，算出目前的瞬時速度 (公尺/秒)
        double velocityMetersPerSec = DriveMotor.getVelocity().getValueAsDouble() * SwerveConstants.kWheelCircumference;
        // 讀取轉向馬達當前的角度
        Rotation2d angle = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());
        // 將瞬時速度與角度包裝成 SwerveModuleState 物件並回傳
        return new SwerveModuleState(velocityMetersPerSec, angle);

    }

}
