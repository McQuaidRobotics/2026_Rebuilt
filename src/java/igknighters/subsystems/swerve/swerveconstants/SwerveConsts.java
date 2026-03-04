package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.RobotController;
import igknighters.util.log.Log;

public class SwerveConsts {

    String robotSerialNumber;

    public enum Robots {
        DEMO_BOT,
        GEMINKNIGHT,
        UNKNOWN
    };

    private String DEMO_BOT_SERIAL_NUMBER = "TBD";
    private String GEMINKNIGHT_SERIAL_NUMBER = "03260AF0";

    public Robots getRobot() {
        robotSerialNumber = RobotController.getSerialNumber();
        Log.log("ROBOT/ROBOT_INFO/ROBOT SERIAL NUMBER", "Serial Number: " + robotSerialNumber);
        if (robotSerialNumber.equals(DEMO_BOT_SERIAL_NUMBER)) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "DEMO_BOT");
            return Robots.DEMO_BOT;
        } else if (robotSerialNumber.equals(GEMINKNIGHT_SERIAL_NUMBER)) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "GEMINKNIGHT");
            return Robots.GEMINKNIGHT;
        } else {
            Log.log(
                    "ROBOT_INFO/ROBOT TYPE",
                    "UNKNOWN: geminknight is: " + GEMINKNIGHT_SERIAL_NUMBER);
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.DEMO_BOT)) {
            Log.log("ROBOT/SWERVE CONSTS", "Using DemoBotConsts");
            return new DemoBotConsts();
        } else if (robot.equals(Robots.GEMINKNIGHT)) {
            Log.log("ROBOT/SWERVE CONSTS", "Using GeminiConsts");
            return new GeminiConsts();
        } else {
            Log.log("ROBOT/SWERVE CONSTS", "Using DemoBotConsts (default)");
            return new DemoBotConsts();
        }
    }
}
