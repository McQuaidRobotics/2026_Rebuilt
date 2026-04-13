package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class Rectangle implements Obstacle {
    private final double x, y, width, height, strength;

    public Rectangle(double x, double y, double width, double height, double strength) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.strength = strength;
    }

    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {
        Translation2d totalRepulsive = new Translation2d();

        double minX = x - width;
        double maxX = x + width;
        double minY = y - height;
        double maxY = y + height;

        // Only apply force if aligned with one of the faces (prevents corner traps)
        boolean withinWidth = robotPos.getX() >= minX && robotPos.getX() <= maxX;
        boolean withinHeight = robotPos.getY() >= minY && robotPos.getY() <= maxY;

        if (withinWidth || withinHeight) {
            double closestX = Math.max(minX, Math.min(robotPos.getX(), maxX));
            double closestY = Math.max(minY, Math.min(robotPos.getY(), maxY));

            double rdx = robotPos.getX() - closestX;
            double rdy = robotPos.getY() - closestY;
            double rDistance = Math.sqrt(rdx * rdx + rdy * rdy);

            if (rDistance < 1.5) { // ignore if too far
                double effectiveDist = Math.max(0.01, rDistance - robotRadius);
                double magnitude = strength * (1.2 / Math.pow(effectiveDist, 2));

                Translation2d awayForce =
                        new Translation2d(
                                rDistance == 0 ? 0 : rdx / rDistance,
                                rDistance == 0 ? 0 : rdy / rDistance);

                // Sliding logic
                Translation2d tangent;
                if (withinWidth) {
                    // Robot is on top/bottom face: slide along X
                    tangent = new Translation2d(Math.signum(targetPos.getX() - robotPos.getX()), 0);
                } else {
                    // Robot is on side faces: slide along Y
                    tangent = new Translation2d(0, Math.signum(targetPos.getY() - robotPos.getY()));
                }

                totalRepulsive =
                        totalRepulsive.plus(awayForce.plus(tangent.times(0.8)).times(magnitude));
            }
        }
        return totalRepulsive;
    }
}
