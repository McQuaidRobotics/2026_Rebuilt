package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class WallObstacle implements Obstacle {
    private final double x1, y1, width, height, strength;

    public WallObstacle(double x1, double y1, double width, double height, double strength) {
        this.x1 = x1;
        this.y1 = y1;
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
        boolean isHorizontal = height == 0;
        double dx = 0, dy = 0, distance = Double.MAX_VALUE;

        if (isHorizontal) {
            // Check if robot is within the X-span: [x, x + width]
            if (robotPos.getX() >= x1 && robotPos.getX() <= x1 + width) {
                dy = robotPos.getY() - y1;
                distance = Math.abs(dy);
                dx = 0;
            }
        } else {
            // Check if robot is within the Y-span: [y, y + height]
            if (robotPos.getY() >= y1 && robotPos.getY() <= y1 + height) {
                dx = robotPos.getX() - x1;
                distance = Math.abs(dx);
                dy = 0;
            }
        }

        // Influence range for walls is smaller to allow tight trench driving
        if (distance < 0.7) {
            double effectiveDist = Math.max(0.01, distance - robotRadius);
            double magnitude = strength * (0.6 / Math.pow(effectiveDist, 2));
            magnitude = Math.min(magnitude, 1.0); // Never fully block attraction

            Translation2d pushDir =
                    new Translation2d(
                            distance == 0 ? 0 : dx / distance, distance == 0 ? 0 : dy / distance);
            totalRepulsive = totalRepulsive.plus(pushDir.times(magnitude));
        }
        return totalRepulsive;
    }
}
