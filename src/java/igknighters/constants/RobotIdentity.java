package igknighters.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;

public class RobotIdentity {

    public enum Robots {
        GEMINKNIGHT,
        DEMO_BOT,
        SECOND_BOT
    }

    private static final String GEMINKNIGHT_SERIAL_NUMBER = "032B4B20";
    private static final String SECOND_BOT_SERIAL_NUMBER = "03260ABB"; // To be filled in later

    private static Robots robot = null;

    public static Robots getRobot() {
        if (robot == null) {
            String serialNumber = RobotController.getSerialNumber();
            if (serialNumber.equals(GEMINKNIGHT_SERIAL_NUMBER)) {
                robot = Robots.GEMINKNIGHT;
            } else if (serialNumber.equals(SECOND_BOT_SERIAL_NUMBER)) {
                robot = Robots.SECOND_BOT;
            } else if (serialNumber.equals("TBD")) { // Placeholder for Demo Bot if different
                robot = Robots.GEMINKNIGHT;
            } else if (Robot.isSimulation()) {
                robot = Robots.SECOND_BOT; // Default to second bot for simulation, can be changed
                // if needed

            } else {
                DriverStation.reportError(
                        "THE SERIAL NUMBER OF THE ROBOT THAT THIS CODE IS DEPLOYED ON IS NOT IN"
                                + " RobotIdentity.java",
                        true);
                robot = Robots.SECOND_BOT; // will default to THIS ONE
            }
        }
        return robot;
    }

    public static boolean isGemini() {
        return getRobot() == Robots.GEMINKNIGHT;
    }

    public static boolean isSecondBot() {
        return getRobot() == Robots.SECOND_BOT;
    }
}
