package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight4Subsystem extends SubsystemBase {

    private final NetworkTable limelight4Table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight4Subsystem() {
    }

    /**
     * 判斷是否滿足回傳球條件：
     * - 藍方聯盟且看到 AprilTag 17 或 22
     * - 紅方聯盟且看到 AprilTag 6 或 1
     */
    public boolean canReturnBall() {
        if (limelight4Table.getEntry("tv").getDouble(0.0) != 1.0) {
            return false;
        }

        long tid = (long) limelight4Table.getEntry("tid").getInteger(0);
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        if (isRed) {
            return tid == 6 || tid == 1;
        } else {
            return tid == 17 || tid == 22;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight4 Can Return Ball", canReturnBall());
        SmartDashboard.putNumber("Limelight4 Target ID", limelight4Table.getEntry("tid").getInteger(0));
    }
}
