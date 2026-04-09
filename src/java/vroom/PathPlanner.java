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
        Translation2d finalAttractive = unitAttractive.times(1.2);

        Translation2d totalRepulsive = new Translation2d();
        double robotRadius = 0.35;

        for (Field.obstacle obs : field.getObstacles()) {
            if (obs.type() == Field.obstacleType.CIRCLE) {
                // --- CIRCLE: x,y is center, width is radius ---
                double dist = current.getDistance(new Translation2d(obs.x(), obs.y()));
                double edgeDist = dist - obs.width();
                if (edgeDist < obstacleInfluenceRange) {
                    double magnitude =
                            obs.strength() * (1.0 / Math.pow(Math.max(0.1, edgeDist), 2));
                    Translation2d pushDir =
                            current.minus(new Translation2d(obs.x(), obs.y())).div(dist);
                    totalRepulsive = totalRepulsive.plus(pushDir.times(magnitude));
                }

            } else if (obs.type() == Field.obstacleType.WALL) {
                // --- WALL: x,y is start, width/height is TOTAL length ---
                boolean isHorizontal = obs.height() == 0;
                double dx = 0, dy = 0, distance = Double.MAX_VALUE;

                if (isHorizontal) {
                    // Check if robot is within the X-span: [x, x + width]
                    if (current.getX() >= obs.x() && current.getX() <= obs.x() + obs.width()) {
                        dy = current.getY() - obs.y();
                        distance = Math.abs(dy);
                        dx = 0;
                    }
                } else {
                    // Check if robot is within the Y-span: [y, y + height]
                    if (current.getY() >= obs.y() && current.getY() <= obs.y() + obs.height()) {
                        dx = current.getX() - obs.x();
                        distance = Math.abs(dx);
                        dy = 0;
                    }
                }

                // Influence range for walls is smaller to allow tight trench driving
                if (distance < 0.7) {
                    double effectiveDist = Math.max(0.01, distance - robotRadius);
                    double magnitude = obs.strength() * (0.6 / Math.pow(effectiveDist, 2));
                    magnitude = Math.min(magnitude, 1.0); // Never fully block attraction

                    Translation2d pushDir =
                            new Translation2d(
                                    distance == 0 ? 0 : dx / distance,
                                    distance == 0 ? 0 : dy / distance);
                    totalRepulsive = totalRepulsive.plus(pushDir.times(magnitude));
                }

            } else if (obs.type() == Field.obstacleType.RECTANGLE) {
                // --- RECTANGLE: x,y is center, width/height is HALF-LENGTH ---
                double minX = obs.x() - obs.width();
                double maxX = obs.x() + obs.width();
                double minY = obs.y() - obs.height();
                double maxY = obs.y() + obs.height();

                // Only apply force if aligned with one of the faces (prevents corner traps)
                boolean withinWidth = current.getX() >= minX && current.getX() <= maxX;
                boolean withinHeight = current.getY() >= minY && current.getY() <= maxY;

                if (withinWidth || withinHeight) {
                    double closestX = Math.max(minX, Math.min(current.getX(), maxX));
                    double closestY = Math.max(minY, Math.min(current.getY(), maxY));

                    double rdx = current.getX() - closestX;
                    double rdy = current.getY() - closestY;
                    double rDistance = Math.sqrt(rdx * rdx + rdy * rdy);

                    if (rDistance < obstacleInfluenceRange) {
                        double effectiveDist = Math.max(0.01, rDistance - robotRadius);
                        double magnitude = obs.strength() * (1.2 / Math.pow(effectiveDist, 2));

                        Translation2d awayForce =
                                new Translation2d(
                                        rDistance == 0 ? 0 : rdx / rDistance,
                                        rDistance == 0 ? 0 : rdy / rDistance);

                        // Sliding logic
                        Translation2d tangent;
                        if (withinWidth) {
                            // Robot is on top/bottom face: slide along X
                            tangent =
                                    new Translation2d(
                                            Math.signum(target.getX() - current.getX()), 0);
                        } else {
                            // Robot is on side faces: slide along Y
                            tangent =
                                    new Translation2d(
                                            0, Math.signum(target.getY() - current.getY()));
                        }

                        totalRepulsive =
                                totalRepulsive.plus(
                                        awayForce.plus(tangent.times(0.8)).times(magnitude));
                    }
                }
            }
        }
        return finalAttractive.plus(totalRepulsive);
    }

    /**
     * Estimates the required pose at a specific time.
     *
     * @param path The generated path.
     * @param seconds Seconds since start of path.
     */
    public Pose2d getPoseAtTime(Pose2d[] path, double seconds) {
        if (path.length == 0) return new Pose2d();

        double distanceToTravel = seconds * maxVelocity;
        int index = (int) Math.round(distanceToTravel / lookaheadDistance);

        if (index >= path.length) return path[path.length - 1];

        Logger.recordOutput("PATH_INDEX", index);
        Logger.recordOutput("POSITION LOOKED UP", path[Math.max(0, index)]);
        return path[Math.max(0, index)];
    }
}
