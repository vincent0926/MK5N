package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// A 鍵 (toggle) 的射擊模式選擇器：每週期依條件在三個射擊指令間切換。
// 優先順序：
// 1. returnBallCondition 成立 → ReturnBall
// 2. autoAimAndShootCondition 成立且 operateCondition 不成立 → AutoAimAndShoot
// 3. 其他 → Operate
// 子指令不交給排程器，而是由本指令直接呼叫其 initialize/execute/end，
// 所以本指令要合併三者的 requirements（含 Swerve），子指令的 isFinished() 也不會被檢查。
// 視覺條件有防抖動：看到目標立刻成立，要連續看不到 kTargetLostDelaySec 才不成立，
// 避免 Limelight 掉幀時退回 Operate（hood 跳 30°、送球計時重來，永遠射不出去）。
// 駕駛操作條件不防抖動，推搖桿立刻讓出底盤。
// 每次切換都會 end 舊模式並 initialize 新模式（仰角鎖定與送球計時重新開始）。
public class ShootSelector extends Command {

    private final BooleanSupplier returnBallCondition;
    private final BooleanSupplier autoAimAndShootCondition;
    // 駕駛正在操作（搖桿/扳機）時為 true，用來讓出 AutoAimAndShoot 對 Swerve 的控制。
    private final BooleanSupplier operateCondition;

    // 失去目標後要等多久才退回 Operate (s)，上機看 ShootSelector/Mode 調整
    private static final double kTargetLostDelaySec = 2;
    private final Debouncer returnBallDebouncer = new Debouncer(kTargetLostDelaySec, DebounceType.kFalling);
    private final Debouncer autoAimDebouncer = new Debouncer(kTargetLostDelaySec, DebounceType.kFalling);

    private final AutoAimAndShoot autoAimAndShoot;
    private final ReturnBall returnBall;
    private final Operate operate;

    // 現在正在跑哪個模式
    private Command current = null;

    public ShootSelector(AutoAimAndShoot autoAimAndShoot, ReturnBall returnBall, Operate operate,
            BooleanSupplier returnBallCondition, BooleanSupplier autoAimAndShootCondition,
            BooleanSupplier operateCondition) {
        this.autoAimAndShoot = autoAimAndShoot;
        this.returnBall = returnBall;
        this.operate = operate;

        this.returnBallCondition = returnBallCondition;
        this.autoAimAndShootCondition = autoAimAndShootCondition;
        this.operateCondition = operateCondition;

        // 子指令由本指令直接執行，需由本指令佔用它們所有的子系統
        addRequirements(autoAimAndShoot.getRequirements());
        addRequirements(returnBall.getRequirements());
        addRequirements(operate.getRequirements());

    }

    @Override
    public void initialize() {
        current = null;
    }

    @Override
    public void execute() {
        Command want;//這個週期應該跑哪個模式
        // 兩個 debouncer 每週期都要更新，不能放進 if 條件裡短路掉
        boolean seeReturnTag = returnBallDebouncer.calculate(returnBallCondition.getAsBoolean());
        boolean seeHub = autoAimDebouncer.calculate(autoAimAndShootCondition.getAsBoolean());

        if (seeReturnTag) {
            want = returnBall;
        } else if (seeHub && !operateCondition.getAsBoolean()) {
            want = autoAimAndShoot;
        } else {
            want = operate;
        }

        // 模式改變時：舊模式以 interrupted 結束，新模式從頭初始化
        if (want != current) {
            if (current != null) { 
                current.end(true); //結束舊的模式
            }
            current = want; //切換到新的模式
            current.initialize(); //新的模式初始化
            SmartDashboard.putString("ShootSelector/Mode", current.getName());
        }

        current.execute(); //執行新模式

    }

    // 結束目前的子指令；不論本指令是否被中斷，都以 interrupted=true 傳下去。
    @Override
    public void end(boolean interrupted) {

        if (current != null) {
            current.end(true);
        }
        SmartDashboard.putString("ShootSelector/Mode", "Off");
    }

}
