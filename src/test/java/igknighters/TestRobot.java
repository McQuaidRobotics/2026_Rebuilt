package igknighters;

import edu.wpi.first.hal.HAL;

public class TestRobot {
    private static Robot robot;

    public static Robot get() {
        if (robot == null) {
            try {
                HAL.initialize(500, 0);
            } catch (Exception e) {
            }
            robot = new Robot(false);
            robot.robotInit();
        }
        return robot;
    }
}
