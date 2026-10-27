package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * 整份程式的煙霧測試：建立 Robot，依序跑 Disabled → 自動 15 秒 → 遙控 (推搖桿、每顆按鍵、十字鍵、右搖桿)，
 * 藍方紅方各一次。排程器裡任何指令丟例外，或 Robot 建構失敗，測試就會失敗。
 *
 * 限制：Swerve 沒有模擬物理 (馬達 SimState 沒更新)，模擬中機器人不會真的移動，
 * 所以只檢查自動開始時位置被設成 Blue3-1 起點 (紅方要鏡像)，不檢查路線有沒有開對。
 */
class RobotSmokeTest {
    private static final double DT = 0.02;
    private static final double FIELD_LENGTH_M = 16.541;
    private static final double FIELD_WIDTH_M = 8.069;
    // Blue3-1.path 第一個點
    private static final double BLUE3_START_X = 4.4767;
    private static final double BLUE3_START_Y = 0.8455;

    private static Robot robot;
    private static XboxControllerSim driver;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        driver = new XboxControllerSim(0);
        robot = new Robot();
    }

    @AfterAll
    static void cleanup() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
        SimHooks.resumeTiming();
    }

    /** Swerve 發布在 Dashboard "Field" 的機器人位置 [x, y, 角度]。 */
    private static double[] fieldPose() {
        return NetworkTableInstance.getDefault().getTable("SmartDashboard/Field")
                .getEntry("Robot").getDoubleArray(new double[3]);
    }

    private interface ModePeriodic {
        void run();
    }

    private static void step(double seconds, ModePeriodic modePeriodic) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();
            modePeriodic.run();
            robot.simulationPeriodic();
            SimHooks.stepTimingAsync(DT); // stepTiming 會等 Notifier 執行緒，Phoenix 6 模擬下會卡住
        }
    }

    private static void runMatch(AllianceStationID station, double expectedStartX, double expectedStartY) {
        DriverStationSim.setAllianceStationId(station);

        // Disabled
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        robot.disabledInit();
        step(1.0, robot::disabledPeriodic);

        // 自動 15 秒
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        robot.autonomousInit();
        step(0.1, robot::autonomousPeriodic);
        double[] start = fieldPose();
        assertEquals(expectedStartX, start[0], 0.01, "自動起點 x");
        assertEquals(expectedStartY, start[1], 0.01, "自動起點 y");
        step(14.9, robot::autonomousPeriodic);

        // 遙控
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        robot.teleopInit();
        step(0.5, robot::teleopPeriodic);

        // 往前開、往左平移、右扳機轉向
        driver.setLeftY(-0.6);
        driver.setLeftX(-0.3);
        driver.setRightTriggerAxis(0.5);
        driver.notifyNewData();
        step(2.0, robot::teleopPeriodic);
        driver.setLeftY(0);
        driver.setLeftX(0);
        driver.setRightTriggerAxis(0);
        driver.notifyNewData();
        step(0.5, robot::teleopPeriodic);

        // 每顆按鍵各按兩下 (toggle 的指令第二下關掉)
        for (int button = 1; button <= 10; button++) {
            for (int press = 0; press < 2; press++) {
                driver.setRawButton(button, true);
                driver.notifyNewData();
                step(0.1, robot::teleopPeriodic);
                driver.setRawButton(button, false);
                driver.notifyNewData();
                step(1.0, robot::teleopPeriodic);
            }
        }

        // 十字鍵左右 (hood) 與右搖桿 (orbit)
        for (int pov : new int[] { 90, 270 }) {
            driver.setPOV(pov);
            driver.notifyNewData();
            step(0.5, robot::teleopPeriodic);
            driver.setPOV(-1);
            driver.notifyNewData();
            step(0.2, robot::teleopPeriodic);
        }
        driver.setRightY(-0.8);
        driver.notifyNewData();
        step(0.5, robot::teleopPeriodic);
        driver.setRightY(0);
        driver.notifyNewData();
        step(0.5, robot::teleopPeriodic);

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        robot.disabledInit();
        step(0.5, robot::disabledPeriodic);
    }

    @Test
    void 藍方跑完整場不會當掉() {
        runMatch(AllianceStationID.Blue1, BLUE3_START_X, BLUE3_START_Y);
    }

    @Test
    void 紅方跑完整場不會當掉且起點是鏡像() {
        runMatch(AllianceStationID.Red1, FIELD_LENGTH_M - BLUE3_START_X, FIELD_WIDTH_M - BLUE3_START_Y);
    }
}
