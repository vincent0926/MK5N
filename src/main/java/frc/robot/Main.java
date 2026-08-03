// 版權所有 (c) FIRST 與其他 WPILib 貢獻者。
// 開源軟體；你可以根據本專案根目錄下的 WPILib BSD 授權條款進行修改與/或分享。

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * 請勿在此類別中新增任何靜態變數，或進行任何初始化動作。
 * 除非你清楚自己在做什麼，否則請勿修改此檔案，除非是為了更改 startRobot 呼叫的參數類別。
 */
public final class Main {
  private Main() {}

  /**
   * 主要初始化函式。請勿在此執行任何初始化。
   *
   * <p>如果你更改了主要的 robot 類別，請更改參數類型。
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
