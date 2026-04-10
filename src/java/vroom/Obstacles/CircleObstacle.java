package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class CircleObstacle implements Obstacle {
    private final double x, y, radius, strength;

    public CircleObstacle(double x, double y, double radius, double strength) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.strength = strength;
    }

    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {
        Translation2d totalRepulsive = new Translation2d();

        // --- CIRCLE: x,y is center, width is radius ---
        double dist = robotPos.getDistance(new Translation2d(x, y));
        double edgeDist = dist - radius;
        if (edgeDist < 1.5) {
            double magnitude = strength * (1.0 / Math.pow(Math.max(0.1, edgeDist), 2));
            Translation2d pushDir = robotPos.minus(new Translation2d(x, y)).div(dist);
            totalRepulsive = totalRepulsive.plus(pushDir.times(magnitude));
        }
        return totalRepulsive;
    }
}
