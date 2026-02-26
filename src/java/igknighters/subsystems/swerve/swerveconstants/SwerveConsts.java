package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.RobotController;

public class SwerveConsts {

    String robotSerialNumber;

    public enum Robots {
        DEMO_BOT,
        GEMINKNIGHT,
        UNKNOWN
    };

    private String DEMO_BOT_SERIAL_NUMBER = "TBD";
    private String GEMINKNIGHT_SERIAL_NUMBER = "IT DOESNT EXIST YET";

    public Robots getRobot() {
        robotSerialNumber = RobotController.getSerialNumber();
        if (robotSerialNumber == DEMO_BOT_SERIAL_NUMBER) {
            return Robots.DEMO_BOT;
        } else if (robotSerialNumber == GEMINKNIGHT_SERIAL_NUMBER) {
            return Robots.GEMINKNIGHT;
        } else {
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.DEMO_BOT)) {
            return new DemoBotConsts();
        } else if (robot.equals(Robots.GEMINKNIGHT)) {
            return new GemiknightConsts();
        } else {
            return new DemoBotConsts();
        }
    }
}
