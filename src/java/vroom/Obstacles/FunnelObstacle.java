package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class FunnelObstacle implements Obstacle {
    private final double x, y, halfWidth, halfHeight, strength, obstacleInfluenceRange;
    private final double protrusionDepth; // How far the triangles stick out

    public FunnelObstacle(
            double x,
            double y,
            double halfWidth,
            double halfHeight,
            double protrusionDepth,
            double strength,
            double obstacleInfluenceRange) {
        this.x = x;
        this.y = y;
        this.halfWidth = halfWidth;
        this.halfHeight = halfHeight;
        this.protrusionDepth = protrusionDepth;
        this.strength = strength;
        this.obstacleInfluenceRange = obstacleInfluenceRange;
    }

    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {

        // 1. Identify if we are dealing with the rectangle core or the triangles
        double dx = robotPos.getX() - x;
        double dy = robotPos.getY() - y;

        Translation2d closestPoint;

        // Check if we are in the "Triangle Zones" (protruding from the long edges)
        // Assuming 'width' is the long edge, triangles are on top and bottom (Y-axis)
        if (Math.abs(dx) <= halfWidth && Math.abs(dy) > halfHeight) {
            // Logic for the triangular "roof" or "floor"
            double sideSign = Math.signum(dy);
            double peakY = (halfHeight + protrusionDepth) * sideSign;

            // The triangle edges go from (+-halfWidth, sideSign * halfHeight) to (0, peakY)
            // We find the closest point on the nearest slanted edge of the triangle
            double slope = protrusionDepth / halfWidth;
            double targetY = sideSign * (halfHeight + protrusionDepth - (slope * Math.abs(dx)));

            closestPoint = new Translation2d(dx + x, targetY + y);
        } else {
            // Standard Rectangle Logic for the rest
            double closestX = Math.max(x - halfWidth, Math.min(robotPos.getX(), x + halfWidth));
            double closestY = Math.max(y - halfHeight, Math.min(robotPos.getY(), y + halfHeight));
            closestPoint = new Translation2d(closestX, closestY);
        }

        double dist = robotPos.getDistance(closestPoint);
        Translation2d totalForce = new Translation2d();

        if (dist < obstacleInfluenceRange) {
            double safeDist = Math.max(0.01, dist);
            double effectiveDist = Math.max(0.01, dist - robotRadius);
            double magnitude = strength * (1.0 / Math.pow(effectiveDist, 2));

            // Push away from the closest point
            Translation2d awayForce = robotPos.minus(closestPoint).div(safeDist);

            // Funneling logic: Push toward the nearest "exit" (corner)
            // If we are on the slanted part, the tangent guides us away from the center X
            Translation2d tangent = new Translation2d(Math.signum(dx), 0);

            // If the robot is already moving toward target, bias the tangent that way
            if (Math.signum(targetPos.getX() - robotPos.getX()) == Math.signum(dx)) {
                tangent = tangent.times(1.2); // Extra boost to clear the obstacle
            }

            totalForce = awayForce.plus(tangent.times(0.5)).times(magnitude);
        }

        return totalForce;
    }
}
