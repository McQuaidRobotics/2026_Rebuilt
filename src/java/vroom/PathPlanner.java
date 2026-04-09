package vroom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class PathPlanner {
    private final Field field;
    private final double maxVelocity; // meters per second
    private final double lookaheadDistance = 0.1; // step size in meters
    private final double obstacleInfluenceRange = 1.5; // distance where obstacles start pushing

    public PathPlanner(Field field, double maxVelocity) {
        this.field = field;
        this.maxVelocity = maxVelocity;
    }

    /**
     * Generates a path from current to target.
     *
     * @return A list of Poses representing the path.
     */
    public List<Pose2d> generatePath(Pose2d current, Pose2d target) {
        List<Pose2d> path = new ArrayList<>();
        Pose2d cursor = current;
        int maxSteps = 500; // Safety break to prevent infinite loops

        while (cursor.getTranslation().getDistance(target.getTranslation()) > 0.1 && maxSteps > 0) {
            Translation2d force =
                    calculateTotalForce(cursor.getTranslation(), target.getTranslation());

            // Move cursor in direction of force by lookahead distance
            Translation2d nextStep =
                    cursor.getTranslation()
                            .plus(new Translation2d(lookaheadDistance, force.getAngle()));

            cursor = new Pose2d(nextStep, force.getAngle());
            path.add(cursor);
            maxSteps--;
        }
        return path;
    }

    private Translation2d calculateTotalForce(Translation2d current, Translation2d target) {
        // 1. Attractive force (towards target)
        Translation2d attractive = target.minus(current);
        double distToTarget = attractive.getNorm();
        Translation2d unitAttractive = attractive.div(distToTarget);

        // 2. Repulsive force (away from obstacles)
        Translation2d totalRepulsive = new Translation2d();
        for (Field.obstacle obs : field.getObstacles()) {
            Translation2d obsPos = new Translation2d(obs.x(), obs.y());
            double distToObs = current.getDistance(obsPos);

            if (distToObs < obstacleInfluenceRange) {
                // Formula: strength * (1/dist - 1/range) * (1/dist^2)
                double magnitude =
                        obs.strength()
                                * (1.0 / Math.max(0.1, distToObs) - 1.0 / obstacleInfluenceRange);
                Translation2d pushDir = current.minus(obsPos).div(distToObs);
                totalRepulsive = totalRepulsive.plus(pushDir.times(magnitude));
            }
        }

        return unitAttractive.plus(totalRepulsive);
    }

    /**
     * Estimates the required pose at a specific time.
     *
     * @param path The generated path.
     * @param seconds Seconds since start of path.
     */
    public Pose2d getPoseAtTime(List<Pose2d> path, double seconds) {
        if (path.isEmpty()) return new Pose2d();

        double distanceToTravel = seconds * maxVelocity;
        int index = (int) Math.round(distanceToTravel / lookaheadDistance);

        if (index >= path.size()) return path.get(path.size() - 1);
        return path.get(Math.max(0, index));
    }
}
