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
    // 啟動 WPILib 內建的資料紀錄器 (.wpilog)
    DataLogManager.start();

    // 實例化我們的 RobotContainer
    m_robotContainer = new RobotContainer();
  }

  /**
   * 無論在哪個模式下，此函式每 20 毫秒會被呼叫一次。
   */
  @Override
  public void robotPeriodic() {
    // 執行 CommandScheduler 核心排程器
    CommandScheduler.getInstance().run();

    // 注意：請確認你的 RobotContainer.java 中確實有定義 periodic() 方法，否則請註解掉下方一行
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
    System.out.println("[Robot] ====== autonomousInit() 開始 ======");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // 確保自動階段一開始安全收回 Intake
    try {
      m_robotContainer.retractIntake();
    } catch (Exception e) {
      System.err.println("[Robot] retractIntake() 執行失敗: " + e.getMessage());
    }

    if (m_autonomousCommand != null) {
      System.out.println("[Robot] 準備執行自動指令: " + m_autonomousCommand.getName());
      System.out.println("[Robot] AutoBuilder.isConfigured() = " + com.pathplanner.lib.auto.AutoBuilder.isConfigured());
      
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      System.out.println("[Robot] 自動指令已成功排程");
    } else {
      System.err.println("[Robot] 警告：m_autonomousCommand 為 NULL！未選擇自動路線或 PathPlanner 載入失敗。");
    }
  }

  /** 此函式會在自動模式期間週期性地被呼叫。 */
  @Override
  public void autonomousPeriodic() {}

  /** 此函式在每次進入遙控模式時被呼叫一次。 */
  @Override
  public void teleopInit() {
    // 切換至遙控模式時，停止自動模式指令
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // 確保手動階段一開始安全收回 Intake
    try {
      m_robotContainer.retractIntake();
    } catch (Exception e) {
      System.err.println("[Robot] retractIntake() 執行失敗: " + e.getMessage());
    }
  }

  /** 此函式會在操作員控制 (遙控) 期間週期性地被呼叫。 */
  @Override
  public void teleopPeriodic() {}

  /** 每次進入測試模式時，此函式會被呼叫一次。 */
  @Override
  public void testInit() {
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