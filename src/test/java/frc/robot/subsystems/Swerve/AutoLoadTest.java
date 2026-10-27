package frc.robot.subsystems.Swerve;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** 確認 deploy/pathplanner 裡的自動路線都能被 PathPlanner 載入 (檔案格式、路徑名稱、RobotConfig)。 */
class AutoLoadTest {
    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
        if (!AutoBuilder.isConfigured()) {
            new Swerve(false);
        }
    }

    @Test
    void AutoBuilder已設定() {
        assertTrue(AutoBuilder.isConfigured());
    }

    @Test
    void Blue1可以載入() {
        assertDoesNotThrow(() -> new PathPlannerAuto("Blue1"));
    }

    @Test
    void Blue3可以載入() {
        assertDoesNotThrow(() -> new PathPlannerAuto("Blue3"));
    }
}
