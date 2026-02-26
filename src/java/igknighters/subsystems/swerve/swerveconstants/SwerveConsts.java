package igknighters.subsystems.swerve.swerveconstants;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotController;

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
        DogLog.log("ROBOT_INFO/ROBOT SERIAL NUMBER", "Serial Number: " + robotSerialNumber);
        if (robotSerialNumber.equals(DEMO_BOT_SERIAL_NUMBER)) {
            DogLog.log("ROBOT_INFO/ROBOT TYPE", "DEMO_BOT");
            return Robots.DEMO_BOT;
        } else if (robotSerialNumber.equals(GEMINKNIGHT_SERIAL_NUMBER)) {
            DogLog.log("ROBOT_INFO/ROBOT TYPE", "GEMINKNIGHT");
            return Robots.GEMINKNIGHT;
        } else {
            DogLog.log(
                    "ROBOT_INFO/ROBOT TYPE",
                    "UNKNOWN: geminknight is: " + GEMINKNIGHT_SERIAL_NUMBER);
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.DEMO_BOT)) {
            DogLog.log("SWERVE CONSTS", "Using DemoBotConsts");
            return new DemoBotConsts();
        } else if (robot.equals(Robots.GEMINKNIGHT)) {
            DogLog.log("SWERVE CONSTS", "Using GemiknightConsts");
            return new GemiknightConsts();
        } else {
            DogLog.log("SWERVE CONSTS", "Using DemoBotConsts (default)");
            return new DemoBotConsts();
        }
    }
}
