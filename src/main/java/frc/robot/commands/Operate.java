package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.IndexerSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.OrbitSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

// 固定參數射擊：hood 30°、飛輪 15 RPS，開始後超過 1.0 s 送球。不使用視覺、不轉底盤。
// X 鍵直接使用，也是 ShootSelector 在沒有其他條件成立時的預設模式。
// 參數寫死在 execute()，不在 Constants。從不自行結束（沿用 Command 預設 isFinished）。
public class Operate extends Command {

    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final OrbitSubsystem orbit;

    // 從第一次 execute() 起算；不像其他兩個射擊指令會因失去目標而重置。
    private final Timer timer = new Timer();

    private double targetRPS = 0.0;
    private double hoodangle = 0.0; // deg
    public Operate(HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer, OrbitSubsystem orbit) {
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.orbit = orbit;
        addRequirements(hood, shooter, indexer, orbit);

    }

    @Override
    public void initialize() {
        targetRPS = 0.0;
        hoodangle = 0.0;
        timer.stop();
        timer.reset();
    }

    // hood 與飛輪每週期重送同一目標；超過 1.0 s 後才開 Indexer/Orbit。
    @Override
    public void execute() {
        targetRPS = 20.0;
        hoodangle = 30.0;
        // Timer.start() 已在跑時不會重置
        timer.start();

        // 這裡用 >，另外兩個射擊指令用 >=；差一個週期，無實際影響
        if(timer.get() > 1.0){
            indexer.runindexer();
            orbit.runorbit();
            hood.setAngle(hoodangle);
            shooter.setRPS(targetRPS);
        }
        else{
            indexer.stop();
            orbit.stop();
            hood.setAngle(hoodangle);
            shooter.setRPS(targetRPS);
        }
    }

    // 停送球、hood 歸零；飛輪用 setRPS(0) 而非 stop()。
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        orbit.stop();
        hood.setAngle(0.0);
        shooter.setRPS(0.0);
        timer.stop();
        timer.reset();
    }


}