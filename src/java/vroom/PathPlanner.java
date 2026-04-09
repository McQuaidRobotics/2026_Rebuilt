package vroom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class PathPlanner {
    private final Field field;
    private final double maxVelocity; // meters per second
    private final double lookaheadDistance = 0.1; // step size in meters
    private final double obstacleInfluenceRange = 1.5; // distance where obstacles start pushing

    public PathPlanner(Field field, double maxVelocity) {
        this.field = field;
        this.maxVelocity = maxVelocity;
    }

    public Pose2d[] generatePath(Pose2d current, Pose2d target) {
        Pose2d cursor = current;
        int maxSteps = 500;
        // Use an ArrayList temporarily so we don't have to deal with nulls
        java.util.ArrayList<Pose2d> pathList = new java.util.ArrayList<>();
        ArrayList<Pose2d> posesToDisplay = new ArrayList<>();
        System.out.println(
                "DISTANCE: " + cursor.getTranslation().getDistance(target.getTranslation()));
        while (cursor.getTranslation().getDistance(target.getTranslation()) > 0.1
                && pathList.size() < maxSteps) {
            Translation2d force =
                    calculateTotalForce(cursor.getTranslation(), target.getTranslation());

            // Move cursor in direction of force by lookahead distance
            Translation2d nextStep =
                    cursor.getTranslation()
                            .plus(new Translation2d(lookaheadDistance, force.getAngle()));

            cursor = new Pose2d(nextStep, force.getAngle());
            pathList.add(cursor);
            if (pathList.size() % 10 == 0 || pathList.size() == maxSteps || pathList.size() == 1) {
                posesToDisplay.add(cursor);
            }
        }

        // Convert to a clean array with NO null values
        Pose2d[] pathArray = pathList.toArray(new Pose2d[0]);

        Pose2d[] posesToDisplayArray = posesToDisplay.toArray(new Pose2d[0]);

        // Log the clean array
        // Logger.recordOutput("PATH", pathArray);
        Logger.recordOutput("PATH_DISPLAY", posesToDisplayArray);

        return pathArray;
    }

    private Translation2d calculateTotalForce(Translation2d current, Translation2d target) {
        Translation2d attractive = target.minus(current);
        double distToTarget = attractive.getNorm();
        Translation2d unitAttractive =
                (distToTarget > 0) ? attractive.div(distToTarget) : new Translation2d();

        // Higher attraction weight helps "pull" the robot through narrow gaps
        Translation2d finalAttractive = unitAttractive.times(5);

        Translation2d totalRepulsive = new Translation2d();
        double robotRadius = 0.5;

        for (vroom.Obstacles.Obstacle obs : field.getObstacles()) {
            totalRepulsive = totalRepulsive.plus(obs.calculateForce(current, target, robotRadius));
        }
        return finalAttractive.plus(totalRepulsive);
    }

    /**
     * Estimates the required pose at a specific time.
     *
     * @param path The generated path.
     * @param seconds Seconds since start of path.
     * @param lookaheadTime The time to look ahead when estimating the pose.
     */
    public Pose2d getPoseAtTime(Pose2d[] path, double seconds, double lookaheadTime) {
        if (path.length == 0) return new Pose2d();

        double distanceToTravel = (seconds + lookaheadTime) * maxVelocity;
        int index = (int) Math.round(distanceToTravel / lookaheadDistance);

        if (index >= path.length) return path[path.length - 1];

        Logger.recordOutput("PATH_INDEX", index);
        Logger.recordOutput("POSITION LOOKED UP", path[Math.max(0, index)]);
        return path[Math.max(0, index)];
    }
}
