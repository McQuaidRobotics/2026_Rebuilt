package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class StuddedRectangle implements Obstacle {
    private final double x, y, halfWidth, halfHeight, strength, obstacleInfluenceRange;

    public StuddedRectangle(
            double x,
            double y,
            double halfWidth,
            double halfHeight,
            double strength,
            double obstacleInfluenceRange) {
        this.x = x;
        this.y = y;
        this.halfWidth = halfWidth;
        this.halfHeight = halfHeight;
        this.strength = strength;
        this.obstacleInfluenceRange = obstacleInfluenceRange;
    }

    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {
        double minX = x - halfWidth;
        double maxX = x + halfWidth;
        double minY = y - halfHeight;
        double maxY = y + halfHeight;

        Translation2d totalForce = new Translation2d();

        // 1. STANDARD FACE REPULSION (Slideways)
        boolean withinWidth = robotPos.getX() >= minX && robotPos.getX() <= maxX;
        boolean withinHeight = robotPos.getY() >= minY && robotPos.getY() <= maxY;

        if (withinWidth || withinHeight) {
            double closestX = Math.max(minX, Math.min(robotPos.getX(), maxX));
            double closestY = Math.max(minY, Math.min(robotPos.getY(), maxY));
            double rDist = robotPos.getDistance(new Translation2d(closestX, closestY));

            if (rDist < obstacleInfluenceRange) {
                // Safety check for distance
                double safeDist = Math.max(0.01, rDist);
                double effectiveDist = Math.max(0.01, rDist - robotRadius);
                double magnitude = strength * (1.0 / Math.pow(effectiveDist, 2));

                Translation2d awayForce =
                        robotPos.minus(new Translation2d(closestX, closestY)).div(safeDist);
                Translation2d tangent =
                        (withinWidth)
                                ? new Translation2d(
                                        Math.signum(targetPos.getX() - robotPos.getX()), 0)
                                : new Translation2d(
                                        0, Math.signum(targetPos.getY() - robotPos.getY()));

                // Combine and add to total
                totalForce = totalForce.plus(awayForce.plus(tangent.times(0.8)).times(magnitude));
            }
        }

        // 2. CORNER STUDS (Radial Kicker)
        // We check corners REGARDLESS of the faces to ensure a smooth transition
        double[][] corners = {{minX, minY}, {minX, maxY}, {maxX, minY}, {maxX, maxY}};
        double studInfluence = 1.0;

        for (double[] corner : corners) {
            Translation2d cornerPos = new Translation2d(corner[0], corner[1]);
            double distToCorner = robotPos.getDistance(cornerPos);

            if (distToCorner < studInfluence) {
                double effectiveDist = Math.max(0.01, distToCorner - robotRadius);
                double studMagnitude = strength * (2.0 / Math.pow(effectiveDist, 2));
                studMagnitude = Math.min(studMagnitude, 2.5);

                Translation2d kickDir = robotPos.minus(cornerPos).div(Math.max(0.01, distToCorner));
                totalForce = totalForce.plus(kickDir.times(studMagnitude));
            }
        }

        return totalForce;
    }
}
