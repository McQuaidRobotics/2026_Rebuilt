package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class BUMPOBSTACLE implements Obstacle {
    private final double x, y, a, b, strength;

    public BUMPOBSTACLE(double x, double y, double a, double b, double strength) {
        this.x = x;
        this.y = y;
        this.a = a; // Horizontal Radius
        this.b = b; // Vertical Radius
        this.strength = strength;
    }

    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {

        // --- 1. HORIZONTAL PASS CHECK ---
        // Determine if the target is on the other side of the obstacle relative to the robot
        boolean robotIsLeft = robotPos.getX() < x;
        boolean targetIsRight = targetPos.getX() > x;
        boolean targetIsLeft = targetPos.getX() < x;
        boolean robotIsRight = robotPos.getX() > x;

        // If I am on the left and target is on the left, I don't need to pass the bump.
        // If I am on the right and target is on the right, I don't need to pass the bump.
        if ((robotIsLeft && targetIsLeft) || (robotIsRight && targetIsRight)) {
            // Only apply force if we are physically overlapping the oval's X-span
            // otherwise, we don't care about it.
            if (Math.abs(robotPos.getX() - x) > a) {
                return new Translation2d();
            }
        }

        // If we have already physically cleared the X-boundary of the oval
        // in the direction of the target, stop caring.
        if (targetIsRight && robotPos.getX() > x + a) return new Translation2d();
        if (targetIsLeft && robotPos.getX() < x - a) return new Translation2d();

        // --- 2. OVAL POTENTIAL FIELD ---
        double dx = robotPos.getX() - x;
        double dy = robotPos.getY() - y;

        // Normalized distance within the ellipse
        double relX = dx / a;
        double relY = dy / b;
        double combinedDist = Math.sqrt(relX * relX + relY * relY);

        // Influence range (e.g., 2.0x the size of the oval)
        if (combinedDist < 5) {
            double rDistance = Math.sqrt(dx * dx + dy * dy);
            double effectiveDist = Math.max(0.01, rDistance - robotRadius);

            // Potential Field Magnitude
            double magnitude = strength * (3 / Math.pow(effectiveDist, 2));

            // Directional components (Standard Elliptical Gradient)
            // This naturally pushes outward from the center
            Translation2d awayForce = new Translation2d(relX / combinedDist, relY / combinedDist);

            // --- 3. THE TRENCH SHOVE ---
            // Force the robot toward the Y-edge of the oval to "clear the peak"
            // This ensures the robot slides UP or DOWN into the trench
            Translation2d sideShove = new Translation2d(0, Math.signum(dy) * 2.5);

            return awayForce.plus(sideShove).times(magnitude);
        }

        return new Translation2d();
    }
}
