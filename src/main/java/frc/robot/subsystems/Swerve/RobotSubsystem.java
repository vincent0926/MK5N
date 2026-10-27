package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 每個週期先 update()（讀感測器），再 operate()（送出馬達指令）。
 * 指令在 periodic() 之後才執行，所以指令設定的目標會在下一個週期送出。
 */
public abstract class RobotSubsystem extends SubsystemBase {
  public abstract void reset();

  public abstract void update();

  public abstract void operate();

  @Override
  public void periodic() {
    update();
    operate();
  }
}
