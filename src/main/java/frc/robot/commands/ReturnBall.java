package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight4Subsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
// 根據您的需求，您可以加入需要連動的子系統，此處先提供基本架構

public class ReturnBall extends Command {
    private final Limelight4Subsystem limelight4Subsystem;
    
    // 如果回傳球需要啟動射擊或進件，可在此加入對應的子系統
    // private final ShooterSubsystem shooterSubsystem;
    // private final IndexerSubsystem indexerSubsystem;

    public ReturnBall(Limelight4Subsystem limelight4Subsystem) {
        this.limelight4Subsystem = limelight4Subsystem;
        addRequirements(limelight4Subsystem); // 如果有傳入其他子系統，也需要 addRequirements(...)
    }

    @Override
    public void initialize() {
        System.out.println("ReturnBall Command Started");
        // 啟動回傳球機構 (例如：設定 Shooter 轉速、啟動 Indexer 等)
    }

    @Override
    public void execute() {
        // 持續執行的邏輯
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReturnBall Command Ended");
        // 停止回傳球機構
    }

    @Override
    public boolean isFinished() {
        return false; // 如果回傳球是一個持續性的動作，回傳 false，讓綁定的按鍵來控制結束
    }
}
