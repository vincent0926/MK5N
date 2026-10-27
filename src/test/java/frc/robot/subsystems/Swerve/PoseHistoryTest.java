package frc.robot.subsystems.Swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector2f;
import org.junit.jupiter.api.Test;

class PoseHistoryTest {
    private static final float EPSILON = 1e-5f;

    @Test
    void 兩筆之間線性內插() {
        PoseHistory history = new PoseHistory(10);
        history.add(1.00, 0.0f, 0.0f);
        history.add(1.02, 0.08f, 0.0f);
        Vector2f out = new Vector2f();

        assertTrue(history.sample(1.01, out));
        assertEquals(0.04f, out.x, EPSILON);
    }

    @Test
    void 超出範圍回傳false並能分辨太舊() {
        PoseHistory history = new PoseHistory(10);
        history.add(1.0, 0.0f, 0.0f);
        history.add(2.0, 1.0f, 0.0f);
        Vector2f out = new Vector2f();

        assertFalse(history.sample(0.5, out));
        assertTrue(history.isOlderThanOldest(0.5));
        assertFalse(history.sample(2.5, out));
        assertFalse(history.isOlderThanOldest(2.5));
    }

    @Test
    void 滿了蓋掉最舊的() {
        PoseHistory history = new PoseHistory(3);
        for (int i = 0; i < 5; i++) {
            history.add(i, i, 0.0f);
        }
        Vector2f out = new Vector2f();

        assertTrue(history.isOlderThanOldest(1.5));
        assertTrue(history.sample(3.5, out));
        assertEquals(3.5f, out.x, EPSILON);
    }

    @Test
    void 平移所有紀錄() {
        PoseHistory history = new PoseHistory(10);
        history.add(1.0, 1.0f, 2.0f);
        history.shift(0.5f, -1.0f);
        Vector2f out = new Vector2f();

        assertTrue(history.sample(1.0, out));
        assertEquals(1.5f, out.x, EPSILON);
        assertEquals(1.0f, out.y, EPSILON);
    }
}
