package frc.robot.subsystems.Swerve;

import org.joml.Vector2f;

/**
 * 最近一段時間的里程計位置 (環形緩衝區)，給視覺融合補償相機延遲用：
 * 鏡頭畫面是幾十毫秒前拍的，要跟「拍攝當下」的里程計位置比，不能跟現在的位置比。
 */
public class PoseHistory {
  private final double[] times;
  private final float[] xs;
  private final float[] ys;
  private int start;
  private int size;

  public PoseHistory(int capacity) {
    times = new double[capacity];
    xs = new float[capacity];
    ys = new float[capacity];
  }

  public void clear() {
    start = 0;
    size = 0;
  }

  public boolean isEmpty() {
    return size == 0;
  }

  /** 加入一筆位置；時間要遞增，滿了就蓋掉最舊的。 */
  public void add(double timeSec, float x, float y) {
    if (size > 0 && timeSec <= times[index(size - 1)]) {
      return;
    }

    if (size < times.length) {
      size++;
    } else {
      start = (start + 1) % times.length;
    }

    int i = index(size - 1);
    times[i] = timeSec;
    xs[i] = x;
    ys[i] = y;
  }

  /** 時間比最舊的一筆還早 (緩衝區裡找不到拍攝當下的位置)。 */
  public boolean isOlderThanOldest(double timeSec) {
    return size > 0 && timeSec < times[index(0)];
  }

  /**
   * 取出某個時間的位置 (兩筆之間線性內插)。
   *
   * @return 時間落在緩衝區範圍內才回傳 true
   */
  public boolean sample(double timeSec, Vector2f out) {
    if (size == 0 || timeSec < times[index(0)] || timeSec > times[index(size - 1)]) {
      return false;
    }

    for (int k = size - 1; k > 0; k--) {
      int newer = index(k);
      int older = index(k - 1);

      if (timeSec >= times[older]) {
        double span = times[newer] - times[older];
        float t = span > 0.0 ? (float) ((timeSec - times[older]) / span) : 1.0f;
        out.set(xs[older] + (xs[newer] - xs[older]) * t, ys[older] + (ys[newer] - ys[older]) * t);
        return true;
      }
    }

    int first = index(0);
    out.set(xs[first], ys[first]);
    return true;
  }

  /** 視覺修正了目前位置後，舊紀錄也要一起平移，下一張畫面才不會重複修正同一段偏差。 */
  public void shift(float dx, float dy) {
    for (int k = 0; k < size; k++) {
      int i = index(k);
      xs[i] += dx;
      ys[i] += dy;
    }
  }

  private int index(int k) {
    return (start + k) % times.length;
  }
}
