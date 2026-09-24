package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.Optional;

/**
 * 視覺辨識子系統 (VisionSubsystem)。
 *
 * <h2>功能概述</h2>
 * <p>此子系統負責管理機器人上搭載的三組 Limelight 相機（前方、左側、右側），
 * 透過 AprilTag 視覺辨識實現以下核心功能：</p>
 * <ul>
 *     <li>估算機器人在場地上的二維姿態 (Pose2d)，供 {@code DriveSubsystem} 的姿態估測器 (Pose Estimator) 融合使用</li>
 *     <li>計算機器人與目標 AprilTag 之間的水平距離 (公尺)</li>
 *     <li>計算機器人與 HUB（射擊目標）中心的距離與水平角度誤差，供自動瞄準邏輯使用</li>
 *     <li>將即時視覺資訊發布至 SmartDashboard，便於駕駛員與除錯人員監控</li>
 * </ul>
 *
 * <h2>硬體配置</h2>
 * <ul>
 *     <li><b>前方 Limelight ({@code limelight-up})</b>：主相機，用於射擊瞄準與 HUB 偵測</li>
 *     <li><b>左側 Limelight ({@code limelight-left})</b>：輔助相機，擴展可視範圍</li>
 *     <li><b>右側 Limelight ({@code limelight-right})</b>：輔助相機，擴展可視範圍</li>
 * </ul>
 *
 * <h2>定位演算法</h2>
 * <p>本子系統支援 Limelight 的 <b>MegaTag 2</b> 演算法。MegaTag 2 需要外部提供機器人的
 * 陀螺儀方向資訊（yaw 與 yaw rate），以提高多標籤融合定位的精度與穩定性。
 * 因此，{@code DriveSubsystem} 必須在其 {@code periodic()} 方法中定期呼叫
 * {@link #updateMegaTag2(double, double)} 來同步陀螺儀資料。</p>
 *
 * <h2>座標系統</h2>
 * <p>所有姿態資料皆使用 <b>WPI Blue 原點座標系</b>（場地左下角為原點，X 軸朝向紅方，Y 軸朝向場地上方）。
 * 當機器人位於紅方聯盟時，會透過 {@code botpose_wpired} 讀取紅方座標系下的姿態。</p>
 *
 * @see frc.robot.Constants.VisionConstants 視覺相關常數定義（Limelight 名稱、HUB 座標、AprilTag ID 等）
 * @see DriveSubsystem 底盤子系統，負責呼叫 {@link #updateMegaTag2(double, double)} 並融合視覺估測姿態
 */
public class VisionSubsystem extends SubsystemBase {

    /**
     * 前方 Limelight 的 NetworkTable 參照。
     * <p>對應裝設於機器人前方（朝向射擊方向）的 Limelight 相機，
     * 其 NetworkTable 名稱由 {@link VisionConstants#kLimelightFront} 定義。
     * 此相機為主要瞄準相機，用於偵測 HUB AprilTag 並計算射擊角度。</p>
     */
    private final NetworkTable frontLimelight = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.kLimelightFront);

    /**
     * 左側 Limelight 的 NetworkTable 參照。
     * <p>對應裝設於機器人左側的 Limelight 相機，
     * 其 NetworkTable 名稱由 {@link VisionConstants#kLimelightLeft} 定義。
     * 此相機作為輔助視角，擴展機器人的 AprilTag 可視範圍，
     * 提升定位覆蓋率與姿態估測的連續性。</p>
     */
     private final NetworkTable leftLimelight = NetworkTableInstance.getDefault()
             .getTable(VisionConstants.kLimelightLeft);

    /**
     * 右側 Limelight 的 NetworkTable 參照。
     * <p>對應裝設於機器人右側的 Limelight 相機，
     * 其 NetworkTable 名稱由 {@link VisionConstants#kLimelightRight} 定義。
     * 功能與左側 Limelight 對稱，共同確保機器人在旋轉或側向移動時
     * 仍能持續偵測到場地上的 AprilTag。</p>
     */
     private final NetworkTable rightLimelight = NetworkTableInstance.getDefault()
             .getTable(VisionConstants.kLimelightRight);

    /**
     * AprilTag 場地佈局資料。
     * <p>載入自 WPILib 提供的官方場地佈局檔案，包含所有 AprilTag 的 ID 與其在場地上的
     * 精確三維位置 ({@link Pose3d})。用於 {@link #getDistanceFromLimelight(NetworkTable)} 中，
     * 將 Limelight 回報的目標 AprilTag ID 映射至其真實世界座標，
     * 從而計算機器人與該標籤之間的精確平面距離。</p>
     * <p>若場地佈局載入失敗，此欄位將為 {@code null}，相關距離計算方法會回傳 {@code -1.0}。</p>
     */
    private AprilTagFieldLayout fieldLayout;

    /**
     * 建構視覺子系統實例。
     * <p>在建構過程中，會嘗試載入 WPILib 提供的預設 AprilTag 場地佈局
     * ({@link AprilTagFields#kDefaultField})。此佈局檔案包含當前賽季所有
     * AprilTag 的 ID 與三維座標資訊，為後續的目標距離計算提供基礎。</p>
     * <p>若載入過程中發生例外（例如檔案遺失或格式不正確），將透過
     * {@link DriverStation#reportError(String, StackTraceElement[])} 回報錯誤訊息，
     * 但不會中斷機器人啟動流程。此時 {@link #fieldLayout} 將保持為 {@code null}，
     * 部分依賴場地佈局的功能（如 {@link #getDistanceFromLimelight(NetworkTable)}）
     * 將無法正常運作。</p>
     */
    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout (載入 AprilTag 場地佈局失敗)", e.getStackTrace());
        }
    }

    /**
     * 更新 MegaTag 2 演算法所需的機器人陀螺儀方向資訊。
     *
     * <p>Limelight 的 MegaTag 2 多標籤融合定位演算法需要外部提供機器人的即時方向資訊，
     * 以便將多個 AprilTag 的觀測結果更準確地融合為單一姿態估測。此方法將陀螺儀的
     * yaw 角度與角速度封裝為六元素陣列，並透過 NetworkTable 的
     * {@code robot_orientation_set} 條目同步傳送至所有三組 Limelight。</p>
     *
     * <p><b>方向陣列格式：</b> {@code [yaw, yawRate, pitch, pitchRate, roll, rollRate]}</p>
     * <p>其中 pitch 與 roll 相關欄位固定為 0，因為 FRC 機器人在正常運作下
     * 不會有明顯的俯仰或側傾。</p>
     *
     * <p><b>呼叫時機：</b> 此方法應由 {@code DriveSubsystem} 在其 {@code periodic()} 方法中
     * 每個控制迴圈週期（預設 20ms / 50Hz）呼叫一次，以確保 Limelight 持續獲得
     * 最新的方向資料。</p>
     *
     * @param yaw     機器人當前的 Yaw 角度（單位：度），以場地座標系為基準，
     *                逆時針為正方向
     * @param yawRate 機器人當前的 Yaw 角速度（單位：度/秒），表示旋轉速率，
     *                逆時針為正方向
     *
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">
     *      Limelight MegaTag 2 官方文件</a>
     */
    public void updateMegaTag2(double yaw, double yawRate) {
        double[] orientation = new double[] { yaw, yawRate, 0, 0, 0, 0 };
        frontLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
        leftLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
        rightLimelight.getEntry("robot_orientation_set").setDoubleArray(orientation);
    }

    /**
     * 從所有 Limelight 中取得距離最近的 AprilTag 目標之水平距離。
     *
     * <p>此方法依序查詢前方、左側、右側三組 Limelight，透過各自的 {@code botpose_wpiblue}
     * 資料計算機器人與其偵測到的 AprilTag 之間的平面距離（忽略高度差）。
     * 最終回傳三者中距離最短且有效（大於 0）的數值。</p>
     *
     * <p><b>選擇邏輯：</b></p>
     * <ol>
     *     <li>各 Limelight 的距離由 {@link #getDistanceFromLimelight(NetworkTable)} 計算</li>
     *     <li>僅考慮回傳值大於 0 的有效結果（{@code -1.0} 表示該相機未偵測到目標）</li>
     *     <li>從所有有效距離中取最小值作為「最佳」距離</li>
     * </ol>
     *
     * <p><b>副作用：</b> 此方法會將最佳距離值發布至 SmartDashboard 的
     * {@code "Vision Best Target Distance"} 條目，供駕駛員或除錯人員即時監控。</p>
     *
     * @return 距離最近的目標之水平距離（單位：公尺）。
     *         若所有 Limelight 均未偵測到有效目標，則回傳 {@code -1.0}。
     *
     * @see #getDistanceFromLimelight(NetworkTable) 單一 Limelight 的距離計算實作
     */
    public double getBestTargetDistance() {
        double frontDist = getDistanceFromLimelight(frontLimelight);
        double leftDist = getDistanceFromLimelight(leftLimelight);
        double rightDist = getDistanceFromLimelight(rightLimelight);

        // 從三組 Limelight 的距離中挑選最小的有效值（> 0）。
        // bestDist 初始為 -1（無效），遇到第一個有效值時直接採用，
        // 之後若有更小的有效值則取代之。
        double bestDist = -1;
        if (frontDist > 0)
            bestDist = frontDist;
        if (leftDist > 0 && (bestDist == -1 || leftDist < bestDist))
            bestDist = leftDist;
        if (rightDist > 0 && (bestDist == -1 || rightDist < bestDist))
            bestDist = rightDist;

        SmartDashboard.putNumber("Vision Best Target Distance", bestDist);
        return bestDist;
    }

    /**
     * 判斷指定的 AprilTag ID 是否屬於當前聯盟的 HUB（射擊目標）標籤。
     *
     * <p>2026 REBUILT 賽季的場地上，每個聯盟的 HUB 共有多面，每面貼有不同的 AprilTag。
     * 紅方 HUB 的標籤 ID 定義在 {@link VisionConstants#kRedHubTagIds}，
     * 藍方 HUB 的標籤 ID 定義在 {@link VisionConstants#kBlueHubTagIds}。
     * 此方法透過線性搜尋判斷傳入的 ID 是否存在於對應聯盟的標籤清單中。</p>
     *
     * @param tid   欲檢查的 AprilTag ID（由 Limelight 的 {@code tid} 條目取得）
     * @param isRed 是否為紅方聯盟。{@code true} 表示查詢紅方 HUB 標籤，
     *              {@code false} 表示查詢藍方 HUB 標籤
     * @return 若該 AprilTag ID 屬於指定聯盟的 HUB 則回傳 {@code true}，否則回傳 {@code false}
     *
     * @see VisionConstants#kRedHubTagIds  紅方 HUB AprilTag ID 清單
     * @see VisionConstants#kBlueHubTagIds 藍方 HUB AprilTag ID 清單
     */
    public boolean isHubTag(int tid, boolean isRed) {
        int[] hubTags = isRed ? VisionConstants.kRedHubTagIds : VisionConstants.kBlueHubTagIds;
        for (int tagId : hubTags) {
            if (tagId == tid) {
                return true;
            }
        }
        return false;
    }

    /**
     * 根據機器人的估測姿態，計算機器人到所屬聯盟 HUB 中心的水平距離。
     *
     * <p>此方法不依賴 Limelight 的即時偵測結果，而是直接使用傳入的機器人姿態
     * （通常來自姿態估測器融合輪速里程計與視覺定位的結果）與 HUB 中心的已知場地座標
     * 進行幾何距離計算。即使當前所有 Limelight 都未看到 HUB，只要姿態估測器的
     * 累積誤差在可接受範圍內，此方法仍能提供合理的距離估算。</p>
     *
     * <p><b>聯盟判斷：</b> 透過 {@link DriverStation#getAlliance()} 自動判斷當前聯盟，
     * 並選取對應的 HUB 中心座標（{@link VisionConstants#kRedHubCenter} 或
     * {@link VisionConstants#kBlueHubCenter}）。</p>
     *
     * @param robotPose 機器人當前的估測姿態（{@link Pose2d}），應來自姿態估測器
     *                  融合輪速里程計與視覺定位後的結果
     * @return 機器人到 HUB 中心的水平距離（單位：公尺），此值永遠大於等於 0
     *
     * @see #getHubDistance() 不需傳入姿態的版本，直接使用 Limelight 回報的 botpose
     */
    public double getHubDistance(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;
        return robotPose.getTranslation().getDistance(hubCenter);
    }

    /**
     * 透過 Limelight 的即時偵測結果，取得機器人到所屬聯盟 HUB 中心的水平距離。
     *
     * <p>此方法依序檢查前方、左側、右側三組 Limelight，尋找第一個正在偵測
     * 對應聯盟 HUB AprilTag 的相機，並利用該相機回報的 {@code botpose_wpiblue}
     * 姿態資料計算機器人到 HUB 中心的平面距離。</p>
     *
     * <p><b>與 {@link #getHubDistance(Pose2d)} 的差異：</b></p>
     * <ul>
     *     <li>{@link #getHubDistance(Pose2d)} 使用外部傳入的融合姿態，不需要 Limelight 即時看到 HUB</li>
     *     <li>此方法依賴 Limelight 的即時偵測，只有在至少一組相機看到 HUB AprilTag 時才有效</li>
     * </ul>
     *
     * <p><b>查詢優先順序：</b> 前方 → 左側 → 右側。一旦某組 Limelight 提供有效結果
     * （距離 &gt; 0），即立即回傳該結果，不再查詢後續相機。</p>
     *
     * @return 機器人到 HUB 中心的水平距離（單位：公尺）。
     *         若所有 Limelight 均未偵測到對應聯盟的 HUB AprilTag，則回傳 {@code -1.0}。
     *
     * @see #getDistanceToHubFromLimelight(NetworkTable, Translation2d, boolean) 單一 Limelight 的 HUB 距離計算
     * @see #getHubDistance(Pose2d) 基於融合姿態的版本
     */
    public double getHubDistance() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;

        // 依優先順序查詢：前方 → 左側 → 右側
        // 一旦取得有效距離（> 0）即立即回傳，避免不必要的 NetworkTable 查詢
        double dist = getDistanceToHubFromLimelight(frontLimelight, hubCenter, isRed);
        if (dist > 0)
            return dist;
        dist = getDistanceToHubFromLimelight(leftLimelight, hubCenter, isRed);
        if (dist > 0)
            return dist;
        dist = getDistanceToHubFromLimelight(rightLimelight, hubCenter, isRed);
        return dist;
    }

    /**
     * 檢查指定 Limelight 是否偵測到對應聯盟的 HUB AprilTag，若有則計算到 HUB 中心的距離。
     *
     * <p>此方法為 {@link #getHubDistance()} 的內部輔助方法，執行以下步驟：</p>
     * <ol>
     *     <li>檢查 {@code tv}（Target Valid）是否為 1.0，確認該 Limelight 目前有偵測到目標</li>
     *     <li>讀取 {@code tid}（Target ID），確認偵測到的 AprilTag 是否屬於當前聯盟的 HUB</li>
     *     <li>若為 HUB 標籤，讀取 {@code botpose_wpiblue}（機器人在 WPI Blue 座標系下的姿態），
     *         取出 X、Y 座標並計算與 HUB 中心的平面距離</li>
     * </ol>
     *
     * <p><b>無效情況回傳 -1.0 的條件：</b></p>
     * <ul>
     *     <li>Limelight 未偵測到任何目標（{@code tv} ≠ 1.0）</li>
     *     <li>偵測到的 AprilTag 不屬於當前聯盟的 HUB</li>
     *     <li>{@code botpose_wpiblue} 陣列長度不足或座標皆為 0（表示資料無效）</li>
     * </ul>
     *
     * @param table     欲查詢的 Limelight 的 {@link NetworkTable} 參照
     * @param hubCenter 當前聯盟 HUB 的中心座標（{@link Translation2d}），
     *                  來自 {@link VisionConstants#kRedHubCenter} 或 {@link VisionConstants#kBlueHubCenter}
     * @param isRed     是否為紅方聯盟，用於判斷偵測到的 AprilTag 是否屬於正確的 HUB
     * @return 機器人到 HUB 中心的水平距離（單位：公尺）。若無法取得有效資料，回傳 {@code -1.0}。
     *
     * @see #isHubTag(int, boolean) 判斷 AprilTag 是否屬於 HUB
     */
    private double getDistanceToHubFromLimelight(NetworkTable table, Translation2d hubCenter, boolean isRed) {
        // 檢查目標有效性：tv (Target Valid) = 1.0 表示 Limelight 目前有鎖定到一個 AprilTag
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return -1.0;
        }

        // 取得目前鎖定的 AprilTag ID，確認其是否為當前聯盟的 HUB 標籤
        long tid = (long) table.getEntry("tid").getInteger(0);
        if (isHubTag((int) tid, isRed)) {
            // 讀取 botpose_wpiblue：機器人在 WPI Blue 座標系下的六軸姿態
            // 陣列格式為 [X, Y, Z, Roll, Pitch, Yaw]，此處僅使用 X (index 0) 與 Y (index 1)
            double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            // 確認陣列長度足夠且座標非零（全零通常代表 Limelight 尚未產生有效定位結果）
            if (botpose.length >= 2 && (botpose[0] != 0.0 || botpose[1] != 0.0)) {
                Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                return robotTrans.getDistance(hubCenter);
            }
        }
        return -1.0;
    }

    /**
     * 根據機器人的估測姿態，計算車頭對準 HUB 中心所需的水平角度誤差。
     *
     * <p>此方法將機器人當前朝向與「機器人指向 HUB 中心的理想方向」之間的角度差轉換為
     * 符合 Limelight {@code tx} 慣例的水平誤差值，可直接供 PID 控制器或瞄準邏輯使用。</p>
     *
     * <h3>計算步驟</h3>
     * <ol>
     *     <li>以 {@link Math#atan2(double, double)} 計算機器人位置指向 HUB 中心的場地絕對角度</li>
     *     <li>取機器人當前朝向角度（來自姿態估測器）</li>
     *     <li>計算兩者差值，並以 {@link MathUtil#inputModulus(double, double, double)} 限制在 [-180°, 180°] 範圍內</li>
     *     <li>取反號以符合 Limelight tx 慣例（目標在畫面右側為正值 CW，左側為負值 CCW）</li>
     * </ol>
     *
     * <p><b>注意：</b> 若前方 Limelight 目前未偵測到 HUB AprilTag（{@link #hasHubTarget()} 為 false），
     * 此方法會直接回傳 {@code 0.0}，避免在無目標時產生不正確的瞄準修正。</p>
     *
     * @param robotPose 機器人當前的估測姿態（{@link Pose2d}），應來自姿態估測器
     *                  融合輪速里程計與視覺定位後的結果
     * @return 水平角度誤差（單位：度）。正值表示 HUB 在機器人車頭右方（需順時針旋轉），
     *         負值表示在左方（需逆時針旋轉）。若未偵測到 HUB 目標，回傳 {@code 0.0}。
     *
     * @see #getHubTx() 不需傳入姿態的版本，使用 Limelight 回報的 botpose
     * @see #hasHubTarget() 檢查前方 Limelight 是否正在偵測 HUB
     */
    public double getHubTx(Pose2d robotPose) {
        if (!hasHubTarget()) {
            return 0.0;
        }

        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Translation2d hubCenter = isRed ? VisionConstants.kRedHubCenter : VisionConstants.kBlueHubCenter;

        Translation2d robotTrans = robotPose.getTranslation();
        double dx = hubCenter.getX() - robotTrans.getX();
        double dy = hubCenter.getY() - robotTrans.getY();

        // 計算場地座標系中，機器人位置指向 HUB 中心的絕對角度（度）。
        // atan2 回傳值範圍為 (-π, π]，轉換為度後為 (-180°, 180°]。
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // 取得機器人當前車頭朝向的場地絕對角度（度），範圍 (-180°, 180°]
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        // 計算角度差：目標方向 - 車頭朝向。
        // 正值表示目標在車頭逆時針方向 (CCW)，負值表示在順時針方向 (CW)。
        // 使用 inputModulus 將結果限制在 [-180°, 180°] 範圍，避免跨越 ±180° 邊界時的跳變。
        double angleErrorDeg = MathUtil.inputModulus(targetAngleDeg - robotHeadingDeg, -180.0, 180.0);

        // 轉換為 Limelight tx 慣例：
        //   - Limelight tx > 0 → 目標在畫面右側（機器人需順時針旋轉 CW）
        //   - Limelight tx < 0 → 目標在畫面左側（機器人需逆時針旋轉 CCW）
        // 而 angleErrorDeg > 0 表示目標在 CCW 方向，故需取反號
        return -angleErrorDeg;
    }

    /**
     * 透過前方 Limelight 的即時偵測結果，計算對準 HUB 中心的水平角度誤差。
     *
     * <p>此方法為 {@link #getHubTx(Pose2d)} 的簡化版本，不需要外部傳入姿態。
     * 它直接從前方 Limelight 的 {@code botpose_wpiblue} 條目讀取機器人姿態，
     * 再委託 {@link #getHubTx(Pose2d)} 進行角度計算。</p>
     *
     * <p><b>備援機制：</b> 若 {@code botpose_wpiblue} 資料無效（陣列長度不足或座標皆為零），
     * 則直接回傳前方 Limelight 的原始 {@code tx} 值作為備援。此值為 Limelight 自身
     * 計算的目標在畫面中的水平偏移角度，精度較 botpose 計算版本低，但仍可提供
     * 基本的瞄準參考。</p>
     *
     * @return 水平角度誤差（單位：度）。正值表示 HUB 在車頭右方，負值表示在左方。
     *         若前方 Limelight 未偵測到 HUB AprilTag，回傳 {@code 0.0}。
     *
     * @see #getHubTx(Pose2d) 使用外部傳入姿態的完整版本
     * @see #hasHubTarget() 檢查前方 Limelight 是否正在偵測 HUB
     */
    public double getHubTx() {
        if (!hasHubTarget()) {
            return 0.0;
        }

        // 嘗試從前方 Limelight 的 botpose_wpiblue 取得機器人姿態
        // 陣列格式：[X, Y, Z, Roll, Pitch, Yaw]
        double[] botpose = frontLimelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length >= 6 && (botpose[0] != 0.0 || botpose[1] != 0.0)) {
            // 成功取得有效姿態，建構 Pose2d 並委託完整版方法計算
            Pose2d robotPose = new Pose2d(botpose[0], botpose[1],
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5]));
            return getHubTx(robotPose);
        }

        // 備援方案：botpose 無效時，直接使用 Limelight 原始的 tx 值。
        // 此值為 AprilTag 中心在相機畫面中相對於十字準心的水平角度偏移，
        // 精度不如幾何計算但仍可提供基本瞄準參考。
        return frontLimelight.getEntry("tx").getDouble(0.0);
    }

    /**
     * 檢查前方 Limelight 是否正在偵測對應聯盟的 HUB AprilTag。
     *
     * <p>此方法進行兩階段驗證：</p>
     * <ol>
     *     <li><b>目標存在性檢查：</b> 前方 Limelight 的 {@code tv} 是否為 1.0（有偵測到目標）</li>
     *     <li><b>目標合法性檢查：</b> 偵測到的 AprilTag ID（{@code tid}）是否屬於
     *         當前聯盟的 HUB 標籤（透過 {@link #isHubTag(int, boolean)} 驗證）</li>
     * </ol>
     *
     * <p><b>注意：</b> 此方法僅檢查前方 Limelight（{@link #frontLimelight}），
     * 因為前方相機是主要的射擊瞄準相機。左側與右側 Limelight 不參與此檢查。</p>
     *
     * @return 若前方 Limelight 偵測到合法的 HUB AprilTag 則回傳 {@code true}，
     *         否則回傳 {@code false}
     *
     * @see #isHubTag(int, boolean) AprilTag ID 合法性判斷
     * @see #getHubTx() 使用此方法作為前置條件
     * @see #getHubTx(Pose2d) 使用此方法作為前置條件
     */
    public boolean hasHubTarget() {
        // 第一階段：確認前方 Limelight 目前有鎖定到任何 AprilTag
        if (frontLimelight.getEntry("tv").getDouble(0) != 1.0) {
            return false;
        }

        // 第二階段：根據 DriverStation 判斷聯盟顏色，並驗證偵測到的 Tag ID
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        long tid = (long) frontLimelight.getEntry("tid").getInteger(0);
        return isHubTag((int) tid, isRed);
    }

    /**
     * 從所有 Limelight 中取得第一個偵測到目標的相機的水平角度偏移 (tx)。
     *
     * <p>依序查詢前方、左側、右側三組 Limelight 的 {@code tv}（Target Valid）狀態，
     * 回傳第一個有目標（{@code tv} = 1.0）的相機之 {@code tx} 值。</p>
     *
     * <p><b>{@code tx} 的含義：</b> 目標（AprilTag）中心相對於相機畫面十字準心的
     * 水平角度偏移（單位：度）。正值表示目標在畫面右側，負值表示在左側。</p>
     *
     * <p><b>查詢優先順序：</b> 前方 → 左側 → 右側。此順序確保前方相機（主瞄準相機）
     * 的結果優先被採用。</p>
     *
     * <p><b>注意：</b> 此方法不區分 AprilTag 的種類，無論偵測到的是 HUB 標籤還是
     * 其他場地標籤，只要有目標就會回傳其 tx。若需要僅針對 HUB 標籤的角度誤差，
     * 請使用 {@link #getHubTx()} 或 {@link #getHubTx(Pose2d)}。</p>
     *
     * @return 第一個有偵測到目標的 Limelight 之 tx 值（單位：度）。
     *         若所有 Limelight 均未偵測到目標，回傳 {@code 0.0}。
     *
     * @see #getHubTx() 僅針對 HUB AprilTag 的角度誤差計算
     */
    public double getBestTargetTx() {
        // 依優先順序檢查各 Limelight：前方 → 左側 → 右側
        if (frontLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return frontLimelight.getEntry("tx").getDouble(0.0);
        }
        if (leftLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return leftLimelight.getEntry("tx").getDouble(0.0);
        }
        if (rightLimelight.getEntry("tv").getDouble(0) == 1.0) {
            return rightLimelight.getEntry("tx").getDouble(0.0);
        }
        // 三組 Limelight 均未偵測到任何目標，回傳 0.0 作為「無偏移」的預設值
        return 0.0;
    }

    /**
     * 計算指定 Limelight 偵測到的 AprilTag 與機器人之間的水平距離。
     *
     * <p>此方法結合 Limelight 的 MegaTag 2 定位結果與 WPILib 的 AprilTag 場地佈局資料，
     * 計算機器人到其偵測目標的精確平面距離（忽略高度差）。計算步驟如下：</p>
     * <ol>
     *     <li>驗證 Limelight 的 {@code tv} = 1.0，確認有偵測到目標</li>
     *     <li>讀取 {@code botpose_wpiblue} 取得機器人的場地座標 (X, Y)</li>
     *     <li>讀取 {@code tid} 取得目標 AprilTag 的 ID</li>
     *     <li>從 {@link #fieldLayout} 查詢該 AprilTag 的精確三維位置</li>
     *     <li>計算機器人座標與 AprilTag 座標之間的二維平面距離</li>
     * </ol>
     *
     * <p><b>回傳 -1.0 的條件（任一滿足即回傳）：</b></p>
     * <ul>
     *     <li>Limelight 未偵測到目標（{@code tv} ≠ 1.0）</li>
     *     <li>{@code botpose_wpiblue} 陣列長度不足 6</li>
     *     <li>目標 AprilTag ID 無效（≤ 0）</li>
     *     <li>{@link #fieldLayout} 為 {@code null}（場地佈局載入失敗）</li>
     *     <li>場地佈局中不存在該 AprilTag ID 的座標資料</li>
     * </ul>
     *
     * @param table 欲查詢的 Limelight 的 {@link NetworkTable} 參照
     * @return 機器人到目標 AprilTag 的水平距離（單位：公尺）。若無法取得有效資料，回傳 {@code -1.0}。
     *
     * @see #getBestTargetDistance() 從所有 Limelight 中取得最佳距離
     */
    private double getDistanceFromLimelight(NetworkTable table) {
        // 前置條件：確認該 Limelight 目前有鎖定到 AprilTag
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return -1.0;
        }

        // 讀取 botpose_wpiblue：機器人在 WPI Blue 座標系下的六軸姿態
        // 陣列格式：[X(m), Y(m), Z(m), Roll(°), Pitch(°), Yaw(°)]
        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length >= 6) {

            // 從 Limelight 讀取目前鎖定的 AprilTag ID
            long tid = (long) table.getEntry("tid").getInteger(0);
            if (tid > 0 && fieldLayout != null) {
                // 利用場地佈局查詢該 AprilTag 的三維位置 (Pose3d)
                Optional<Pose3d> tagPose = fieldLayout.getTagPose((int) tid);
                if (tagPose.isPresent()) {
                    // 將機器人座標與 AprilTag 座標分別投影至二維平面 (X, Y)，
                    // 計算兩者之間的歐幾里得距離，忽略高度差 (Z)
                    Translation2d robotTrans = new Translation2d(botpose[0], botpose[1]);
                    Translation2d tagTrans = tagPose.get().getTranslation().toTranslation2d();
                    return robotTrans.getDistance(tagTrans);
                }
            }
        }
        return -1.0;
    }

    /**
     * 封裝視覺估算機器人姿態與對應時間戳記的不可變資料類別。
     *
     * <p>此類別用於將 Limelight 回報的機器人場地姿態與精確的 FPGA 時間戳記
     * 打包成一個結構化物件，便於傳遞給 {@code DriveSubsystem} 的姿態估測器
     * （如 {@code SwerveDrivePoseEstimator}）進行視覺量測融合。</p>
     *
     * <p><b>為何需要時間戳記：</b> 視覺姿態估測存在延遲（包含影像擷取延遲 + 管線處理延遲），
     * 姿態估測器需要知道該視覺量測對應的精確時間點，才能在時間軸上正確地將
     * 視覺資料與輪速里程計資料進行內插融合，避免因延遲導致的定位誤差。</p>
     *
     * @see #getEstimatedPose(NetworkTable) 產生此物件的方法
     * @see #getEstimatedPoses() 批量取得所有 Limelight 的估測結果
     */
    public static class EstimatedRobotPose {
        /**
         * 視覺估算的機器人場地姿態。
         * <p>包含機器人在場地上的 X、Y 座標（公尺）與旋轉角度（Rotation2d），
         * 座標系依據聯盟顏色使用 WPI Blue 或 WPI Red 原點。</p>
         */
        public final Pose2d estimatedPose;

        /**
         * 此姿態估算對應的 FPGA 時間戳記（單位：秒）。
         * <p>此值為 {@code Timer.getFPGATimestamp()} 減去 Limelight 回報的
         * 管線延遲 ({@code tl}) 與擷取延遲 ({@code cl})，代表影像實際被擷取的精確時間點。
         * 姿態估測器會利用此時間戳記，在時間軸上找到正確的內插位置進行融合。</p>
         */
        public final double timestampSeconds;

        /**
         * 建構一個估算姿態實例。
         *
         * @param estimatedPose    視覺估算的機器人姿態（{@link Pose2d}），
         *                         包含場地上的二維位置與旋轉角度
         * @param timestampSeconds 此估算對應的 FPGA 時間戳記（單位：秒），
         *                         應為影像擷取時間（已扣除管線與擷取延遲）
         */
        public EstimatedRobotPose(Pose2d estimatedPose, double timestampSeconds) {
            this.estimatedPose = estimatedPose;
            this.timestampSeconds = timestampSeconds;
        }
    }

    /**
     * 從指定 Limelight 取得視覺估算的機器人姿態與時間戳記。
     *
     * <p>此方法讀取 Limelight 的 MegaTag 2 定位結果，將其轉換為
     * {@link EstimatedRobotPose} 物件。估算過程包含以下步驟：</p>
     * <ol>
     *     <li>檢查 {@code tv} = 1.0，確認有目標可用於定位</li>
     *     <li>根據當前聯盟顏色選擇讀取 {@code botpose_wpired} 或 {@code botpose_wpiblue}，
     *         以取得對應座標系下的姿態資料</li>
     *     <li>從六軸姿態陣列中提取 X、Y 座標與 Yaw 角度，建構 {@link Pose2d}</li>
     *     <li>計算精確的 FPGA 時間戳記：{@code Timer.getFPGATimestamp() - tl/1000 - cl/1000}，
     *         其中 {@code tl} 為管線處理延遲（ms），{@code cl} 為影像擷取延遲（ms）</li>
     * </ol>
     *
     * <p><b>座標系選擇邏輯：</b></p>
     * <ul>
     *     <li>紅方聯盟 → 讀取 {@code botpose_wpired}（以場地右下角為原點）</li>
     *     <li>藍方聯盟或聯盟未知 → 讀取 {@code botpose_wpiblue}（以場地左下角為原點）</li>
     * </ul>
     *
     * @param table 欲查詢的 Limelight 的 {@link NetworkTable} 參照
     * @return 包含估算姿態與時間戳記的 {@link Optional}。若 Limelight 未偵測到目標或
     *         姿態資料無效，回傳 {@link Optional#empty()}。
     *
     * @see EstimatedRobotPose 回傳的資料結構
     * @see #getEstimatedPoses() 批量取得所有 Limelight 的估測結果
     */
    private Optional<EstimatedRobotPose> getEstimatedPose(NetworkTable table) {
        // 前置條件：該 Limelight 必須目前有偵測到至少一個 AprilTag
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return Optional.empty();
        }

        // 根據聯盟顏色選擇對應座標系的 botpose 條目名稱
        // 紅方使用 wpired 座標系，藍方（或聯盟未知時）使用 wpiblue 座標系
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        String entryName = isRed ? "botpose_wpired" : "botpose_wpiblue";

        // 讀取六軸姿態陣列：[X(m), Y(m), Z(m), Roll(°), Pitch(°), Yaw(°)]
        double[] botpose = table.getEntry(entryName).getDoubleArray(new double[6]);
        if (botpose.length < 6)
            return Optional.empty();

        // 從陣列中提取 X (index 0)、Y (index 1)、Yaw (index 5)，建構二維姿態物件
        Pose2d pose = new Pose2d(botpose[0], botpose[1],
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(botpose[5]));

        // 計算影像實際擷取時間的 FPGA 時間戳記：
        //   tl = pipeline latency（管線處理延遲，單位：毫秒）
        //   cl = capture latency（影像擷取延遲，單位：毫秒）
        //   timestamp = 當前 FPGA 時間 - (tl + cl) / 1000
        // 此時間戳記代表影像被擷取的精確時間點，而非定位結果產出的時間點，
        // 確保姿態估測器在時間軸上正確融合視覺量測與里程計資料。
        double tl = table.getEntry("tl").getDouble(0.0);
        double cl = table.getEntry("cl").getDouble(0.0);
        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);

        return Optional.of(new EstimatedRobotPose(pose, timestamp));
    }

    /**
     * 批量取得所有 Limelight 的視覺估算姿態。
     *
     * <p>此方法依序查詢前方、左側、右側三組 Limelight，收集所有目前有偵測到
     * AprilTag 並成功產生定位結果的相機之估算姿態。回傳的列表可能包含 0~3 個元素，
     * 取決於各相機的即時偵測狀態。</p>
     *
     * <p><b>使用場景：</b> {@code DriveSubsystem} 在每個控制迴圈週期呼叫此方法，
     * 將所有有效的視覺量測逐一送入姿態估測器（如 {@code SwerveDrivePoseEstimator}），
     * 實現多相機融合定位。多組相機同時提供量測時，姿態估測器會根據各量測的信賴度
     * （通常由標準差矩陣控制）自動進行加權融合。</p>
     *
     * @return 包含所有有效視覺估算姿態的列表（{@code List<EstimatedRobotPose>}）。
     *         列表大小為 0（所有相機均無目標）至 3（所有相機皆有有效估測）。
     *         列表順序固定為：前方、左側、右側（僅包含有效結果的相機）。
     *
     * @see #getEstimatedPose(NetworkTable) 單一 Limelight 的估測實作
     * @see EstimatedRobotPose 列表中元素的資料結構
     */
    public java.util.List<EstimatedRobotPose> getEstimatedPoses() {
        java.util.List<EstimatedRobotPose> poses = new java.util.ArrayList<>();
        // 依序收集各 Limelight 的有效姿態估測結果
        // ifPresent 確保僅將非空的估測結果加入列表
        getEstimatedPose(frontLimelight).ifPresent(poses::add);
        getEstimatedPose(leftLimelight).ifPresent(poses::add);
        getEstimatedPose(rightLimelight).ifPresent(poses::add);
        return poses;
    }

    /**
     * 子系統的週期性執行方法，由 {@code CommandScheduler} 每個控制迴圈週期自動呼叫。
     *
     * <p>目前的實作中，此方法呼叫 {@link #getBestTargetDistance()} 以確保
     * SmartDashboard 上的 {@code "Vision Best Target Distance"} 數值每個週期都會更新，
     * 讓駕駛員與除錯人員能即時監控視覺系統的偵測狀態與目標距離。</p>
     *
     * <p><b>擴展建議：</b> 未來可在此方法中加入額外的週期性任務，例如：</p>
     * <ul>
     *     <li>發布各 Limelight 的連線狀態至 SmartDashboard</li>
     *     <li>記錄視覺定位的延遲統計資料</li>
     *     <li>根據比賽階段自動切換 Limelight 管線</li>
     * </ul>
     */
    @Override
    public void periodic() {
        getBestTargetDistance();
    }
}