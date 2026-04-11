package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.util.log.Log;

public class SwerveConsts {

    String robotSerialNumber;

    public enum Robots {
        DEMO_BOT,
        GEMINKNIGHT,
        SECOND_BOT,
        UNKNOWN
    };

    private String DEMO_BOT_SERIAL_NUMBER = "TBD";
    private String GEMINKNIGHT_SERIAL_NUMBER = "03260AF0";
    private String SECOND_BOT_SERIAL_NUMBER = "03260ABB";

    public Robots getRobot() {
        robotSerialNumber = RobotController.getSerialNumber();
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT SERIAL NUMBER", "Serial Number: " + robotSerialNumber);
        }
        if (robotSerialNumber.equals(DEMO_BOT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "DEMO_BOT");
            }
            return Robots.DEMO_BOT;
        } else if (robotSerialNumber.equals(GEMINKNIGHT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "GEMINKNIGHT");
            }
            return Robots.GEMINKNIGHT;
        } else if (robotSerialNumber.equals(SECOND_BOT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "SECOND_BOT");
            }
            return Robots.SECOND_BOT;
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log(
                        "ROBOT/ROBOT_INFO/ROBOT TYPE",
                        "UNKNOWN: geminknight is: " + GEMINKNIGHT_SERIAL_NUMBER);
            }
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.DEMO_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts");
            }
            return new DemoBotConsts();
        } else if (robot.equals(Robots.GEMINKNIGHT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using GeminiConsts");
            }
            return new GeminiConsts();
        } else if (robot.equals(Robots.SECOND_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using SecondBotConsts");
            }
            return new DarkKnightConsts();
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts (default)");
            }
            return new GeminiConsts();
        }
    }
}
