package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.util.log.Log;

public class SwerveConsts {

    String robotSerialNumber;

    public enum Robots {
        REBUILT_ROBOT,
        UNKNOWN
    };

    private String REBUILT_ROBOT = "03260ABB";

    public Robots getRobot() {
        robotSerialNumber = RobotController.getSerialNumber();
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT SERIAL NUMBER", "Serial Number: " + robotSerialNumber);
        }
        if (robotSerialNumber.equals(REBUILT_ROBOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "REBUILT_ROBOT");
            }
            return Robots.REBUILT_ROBOT;
        } else if (Robot.isSimulation()) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "SIMULATED REBUILT ROBOT");
            return Robots.REBUILT_ROBOT;
        } else {
            DriverStation.reportError(
                    "THE CODE IS DEPLOYED ON A UNKNOWN ROBOT THE SERIAL NUMBER IS NOT IN"
                            + " SwerveConsts.java fix ts",
                    true);
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.REBUILT_ROBOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts");
            }
            return new DarkKnightConsts();
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts (default)");
            }
            return new DarkKnightConsts();
        }
    }
}
