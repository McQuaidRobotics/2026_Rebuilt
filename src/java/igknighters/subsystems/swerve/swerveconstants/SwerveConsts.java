package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.DriverStation;
import igknighters.Robot;
import igknighters.constants.RobotIdentity;
import igknighters.util.log.Log;

public class SwerveConsts {

    public CommonSwerveConsts getSwerveConsts() {
        RobotIdentity.Robots robot = RobotIdentity.getRobot();
        if (robot.equals(RobotIdentity.Robots.DEMO_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts");
            }
            return new DemoBotConsts();
        } else if (robot.equals(RobotIdentity.Robots.GEMINKNIGHT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using GeminiConsts");
            }
            return new GeminiConsts();
        } else if (robot.equals(RobotIdentity.Robots.SECOND_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using SecondBotConsts");
            }
            return new DarkKnightConsts();
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts (default)");
            }

            DriverStation.reportError(
                    "THE SERIAL NUMBER OF THE ROBOT THAT THIS CODE IS DEPLOYED ON IS NOT IN"
                            + " RobotIdentity.java",
                    null);
            return new GeminiConsts();
        }
    }
}
