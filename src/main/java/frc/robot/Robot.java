// 版權所有 (c) FIRST 與其他 WPILib 貢獻者。
// 開源軟體；你可以根據本專案根目錄下的 WPILib BSD 授權條款進行修改與/或分享。

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * 此類別中的方法會根據各個模式自動被呼叫，如同 TimedRobot 文件中所述。
 * 如果你在建立此專案後更改了這個類別的名稱或套件，你必須同時更新專案中的 Main.java 檔案。
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * 此函式會在機器人首次啟動時執行，應該用於任何初始化程式碼。
   */
  public Robot() {
    // 啟動 WPILib 內建的資料紀錄器，這會將 NetworkTables 數據存成 .wpilog 檔案
    // 這些檔案會存在 RoboRIO 的隨身碟中，可以下載後用 AdvantageScope 開啟分析
    DataLogManager.start();

    // 實例化我們的 RobotContainer。這將會執行所有的按鈕綁定，並將
    // 自動模式選擇器放到儀表板 (Dashboard) 上。
    m_robotContainer = new RobotContainer();
  }

  /**
   * 無論在哪個模式下，此函式每 20 毫秒會被呼叫一次。將其用於你想在禁用、自動、遙控和測試模式下
   * 執行的項目，例如診斷功能。
   *
   * <p>此函式會在各模式特定的 periodic (週期性) 函式之後執行，但在 LiveWindow 與
   * SmartDashboard 整合更新之前執行。
   */
  @Override
  public void robotPeriodic() {
    // 執行排程器 (Scheduler)。這負責輪詢按鈕、加入新排程的指令、
    // 執行已排程的指令、移除已完成或被中斷的指令，以及執行子系統的 periodic() 方法。
    // 這必須在機器人的 periodic 區塊中被呼叫，以便讓基於 Command 的框架能正常運作。
    CommandScheduler.getInstance().run();

    // 更新 RobotContainer 裡的定期檢查 (例如全場視覺定位校正)
    m_robotContainer.periodic();
  }

  /** 每次機器人進入禁用 (Disabled) 模式時，此函式會被呼叫一次。 */
  @Override
  public void disabledInit() {}

  /** 機器人在禁用模式期間會週期性地呼叫此函式。 */
  @Override
  public void disabledPeriodic() {}

  /** 此自動模式會執行由你的 {@link RobotContainer} 類別所選擇的自動指令。 */
  @Override
  public void autonomousInit() {
    System.out.println("[Robot] autonomousInit() called!");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand == null) {
        System.out.println("[Robot] WARNING: m_autonomousCommand is NULL!");
    } else {
        System.out.println("[Robot] Scheduling autonomous command: " + m_autonomousCommand.getName());
    }

    // 確保自動階段一開始 Intake 在機器內 (收回)
    m_robotContainer.retractIntake();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** 此函式會在自動模式期間週期性地被呼叫。 */
  @Override
  public void autonomousPeriodic() {}

  /** 此函式在每次進入遙控模式時被呼叫一次。 */
  @Override
  public void teleopInit() {
    // 這能確保當遙控模式開始執行時，自動模式會停止。
    // 如果你希望自動模式繼續執行直到被另一個指令中斷，
    // 請移除或註解掉這一行。
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // 確保手動階段一開始 Intake 在機器內 (收回)
    m_robotContainer.retractIntake();
  }

  /** 此函式會在操作員控制 (遙控) 期間週期性地被呼叫。 */
  @Override
  public void teleopPeriodic() {}

  /** 每次進入測試模式時，此函式會被呼叫一次。 */
  @Override
  public void testInit() {
    // 在測試模式開始時取消所有正在執行的指令。
    CommandScheduler.getInstance().cancelAll();
  }

  /** 此函式會在測試模式期間週期性地被呼叫。 */
  @Override
  public void testPeriodic() {}

  /** 此函式會在機器人首次進入模擬時被呼叫一次。 */
  @Override
  public void simulationInit() {}

  /** 此函式會在模擬期間週期性地被呼叫。 */
  @Override
  public void simulationPeriodic() {}
}
