package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.swerve.Swerve;
import java.util.Set;
import vroom.Field;
import vroom.PathFollower;
import vroom.PathPlanner;

public class Wayfinder {
    static PathPlanner pathPlanner = new PathPlanner(new Field(), 4);
    static PathFollower pathFollower = new PathFollower(1.0, 0, 0);

    public static Command driveToTarget(Swerve swerve, Pose2d target) {

        return Commands.defer(
                () -> {
                    return pathFollower.createFollowPathCommand(
                            swerve,
                            pathPlanner.generateTimestampedPath(swerve.getState().Pose, target),
                            pathPlanner);
                },
                Set.of(swerve));
    }
}
