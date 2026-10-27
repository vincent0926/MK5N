package frc.robot.subsystems.Math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector2f;
import org.junit.jupiter.api.Test;

class SwerveMathTest {
    // 容許誤差：浮點數不能用 == 比較 (float 約 7 位有效數字)
    private static final float EPSILON = 1e-4f;

    @Test
    void wrapRad_把角度收斂到負pi到pi() {
        assertEquals(0.0f, SwerveMath.wrapRad(0.0f), EPSILON);
        assertEquals(1.0f, SwerveMath.wrapRad(1.0f + SwerveMath.Tau), EPSILON);
        assertEquals(-1.0f, SwerveMath.wrapRad(-1.0f - 3 * SwerveMath.Tau), EPSILON);
        // 剛好 pi 要落在 -pi (區間是 [-pi, pi))
        assertEquals(-SwerveMath.Pi, SwerveMath.wrapRad(SwerveMath.Pi), EPSILON);
    }

    @Test
    void fieldToRobot_與_robotToField_互為反運算() {
        Vector2f robot = new Vector2f();
        Vector2f back = new Vector2f();
        float heading = 0.7f;

        SwerveMath.fieldToRobot(2.0f, -1.5f, heading, robot);
        SwerveMath.robotToField(robot.x, robot.y, heading, back);

        assertEquals(2.0f, back.x, EPSILON);
        assertEquals(-1.5f, back.y, EPSILON);
    }

    @Test
    void fieldToRobot_車頭朝場地Y軸時場地X變成車身右側() {
        Vector2f out = new Vector2f();

        // 車頭朝 +90 度 (場地 +Y)：場地 +Y 是車頭前方，場地 +X 是車身右側 (機器人座標 -Y)
        SwerveMath.fieldToRobot(0.0f, 1.0f, SwerveMath.HalfPi, out);
        assertEquals(1.0f, out.x, EPSILON);
        assertEquals(0.0f, out.y, EPSILON);

        SwerveMath.fieldToRobot(1.0f, 0.0f, SwerveMath.HalfPi, out);
        assertEquals(0.0f, out.x, EPSILON);
        assertEquals(-1.0f, out.y, EPSILON);
    }

    @Test
    void arcDisplacement_角度不變時是直線() {
        Vector2f out = new Vector2f();

        SwerveMath.arcDisplacement(0.0f, 0.0f, 2.0f, out);

        assertEquals(2.0f, out.x, EPSILON);
        assertEquals(0.0f, out.y, EPSILON);
    }

    @Test
    void arcDisplacement_四分之一圓() {
        Vector2f out = new Vector2f();
        float radius = 1.0f;

        // 朝 +X 出發、向左轉 90 度，走過 1/4 圓周，位移應為 (r, r)
        SwerveMath.arcDisplacement(0.0f, SwerveMath.HalfPi, radius * SwerveMath.HalfPi, out);

        assertEquals(radius, out.x, EPSILON);
        assertEquals(radius, out.y, EPSILON);
    }

    @Test
    void desaturate_超速時等比例縮小() {
        float[] speeds = {6.0f, -3.0f, 2.0f, 1.0f};

        float scale = SwerveMath.desaturate(speeds, 4.0f);

        assertEquals(4.0f / 6.0f, scale, EPSILON);
        assertEquals(4.0f, speeds[0], EPSILON);
        assertEquals(-2.0f, speeds[1], EPSILON);
    }

    @Test
    void desaturate_未超速時不變() {
        float[] speeds = {1.0f, -2.0f};

        assertEquals(1.0f, SwerveMath.desaturate(speeds, 4.0f), EPSILON);
        assertEquals(-2.0f, speeds[1], EPSILON);
    }

    @Test
    void clampMagnitude_只縮短不改方向() {
        Vector2f v = new Vector2f(3.0f, 4.0f);

        SwerveMath.clampMagnitude(v, 2.5f);

        assertEquals(1.5f, v.x, EPSILON);
        assertEquals(2.0f, v.y, EPSILON);
    }

    @Test
    void headingHoldOmega_誤差越大越快但不超過上限() {
        assertEquals(0.4f, SwerveMath.headingHoldOmega(0.1f, 0.0f, 4.0f, 2.0f), EPSILON);
        assertEquals(2.0f, SwerveMath.headingHoldOmega(1.5f, 0.0f, 4.0f, 2.0f), EPSILON);
        assertEquals(-2.0f, SwerveMath.headingHoldOmega(-1.5f, 0.0f, 4.0f, 2.0f), EPSILON);
    }

    @Test
    void headingHoldOmega_走最短路徑跨越正負pi() {
        // 目前 3.0 rad，目標 -3.0 rad：最短路徑是再往正方向轉約 0.28 rad
        float omega = SwerveMath.headingHoldOmega(-3.0f, 3.0f, 1.0f, 2.0f);

        assertTrue(omega > 0.0f);
        assertEquals(SwerveMath.Tau - 6.0f, omega, EPSILON);
    }

    @Test
    void limitVelocity_一個週期內的加速度不超過上限() {
        Vector2f wanted = new Vector2f(4.0f, 0.0f);
        Vector2f current = new Vector2f();
        Vector2f accel = new Vector2f();
        float dt = 0.02f;
        float maxAccel = 5.0f;

        SwerveMath.limitVelocity(wanted, current, dt, 4.0f, maxAccel, maxAccel, maxAccel, maxAccel, accel);

        // 從靜止開始，一個週期最多加速 maxAccel * dt
        assertEquals(maxAccel * dt, current.x, EPSILON);
        assertEquals(0.0f, current.y, EPSILON);
    }

    @Test
    void limitVelocity_最終會到達目標速度而不超過() {
        Vector2f wanted = new Vector2f(3.0f, 0.0f);
        Vector2f current = new Vector2f();
        Vector2f accel = new Vector2f();

        for (int i = 0; i < 500; i++) {
            SwerveMath.limitVelocity(wanted, current, 0.02f, 4.0f, 5.0f, 5.0f, 5.0f, 5.0f, accel);
            assertTrue(current.x <= 3.0f + EPSILON);
        }

        assertEquals(3.0f, current.x, 1e-2f);
    }

    @Test
    void nearestModule_選出最近的模組() {
        Vector2f[] modules = {
            new Vector2f(1.0f, 1.0f),
            new Vector2f(1.0f, -1.0f),
            new Vector2f(-1.0f, 1.0f),
            new Vector2f(-1.0f, -1.0f)
        };

        assertEquals(1, SwerveMath.nearestModule(modules, 2.0f, -0.5f));
        assertEquals(2, SwerveMath.nearestModule(modules, -2.0f, 0.5f));
    }

    @Test
    void calculateModuleTargets_純旋轉時各輪速度相同且與半徑垂直() {
        Vector2f[] modules = {
            new Vector2f(1.0f, 1.0f),
            new Vector2f(1.0f, -1.0f),
            new Vector2f(-1.0f, 1.0f),
            new Vector2f(-1.0f, -1.0f)
        };
        float[] speeds = new float[4];
        float[] angles = new float[4];

        SwerveMath.calculateModuleTargets(0.0f, 0.0f, 1.0f, new Vector2f(), modules, speeds, angles);

        for (int i = 0; i < 4; i++) {
            assertEquals((float) Math.sqrt(2.0), speeds[i], EPSILON);
        }
        // 左前輪 (1,1) 逆時針旋轉時往左後方 (-1, +1) 移動，角度 135 度
        assertEquals(3.0f * SwerveMath.Pi / 4.0f, angles[0], EPSILON);
    }
}
