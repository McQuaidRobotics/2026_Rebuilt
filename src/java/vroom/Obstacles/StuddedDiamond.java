package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class StuddedDiamond implements Obstacle {
    private final double x, y, halfWidth, halfHeight, strength, obstacleInfluenceRange;

    public StuddedDiamond(
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

        // 1. Relativize coordinates (Center at 0,0)
        double dx = robotPos.getX() - x;
        double dy = robotPos.getY() - y;

        // 2. Map to the first quadrant to simplify edge calculation (Symmetry)
        double absX = Math.abs(dx);
        double absY = Math.abs(dy);

        // The diamond edge is defined by (x / halfWidth) + (y / halfHeight) = 1
        // We find the closest point on this line segment in the first quadrant
        // Normalizing coordinates to a "unit diamond" makes the projection easier
        double normX = absX / halfWidth;
        double normY = absY / halfHeight;

        // If the robot is inside or near the boundary
        // Projection logic to find the closest point on the slanted edge
        double sum = normX + normY;

        // Closest point calculation
        double closestX, closestY;
        if (sum <= 0) { // Safety for center
            closestX = 0;
            closestY = 0;
        } else {
            // Projecting the point onto the line: (x/W) + (y/H) = 1
            // We use a simple clamped projection
            double t = Math.max(0, Math.min(1, (normX - normY + 1) / 2.0));
            closestX = t * halfWidth;
            closestY = (1 - t) * halfHeight;
        }

        // Re-apply signs to get the actual closest point in the original quadrant
        closestX *= Math.signum(dx == 0 ? 1 : dx);
        closestY *= Math.signum(dy == 0 ? 1 : dy);

        Translation2d closestPoint = new Translation2d(closestX + x, closestY + y);
        double distToEdge = robotPos.getDistance(closestPoint);

        Translation2d totalForce = new Translation2d();

        if (distToEdge < obstacleInfluenceRange) {
            double safeDist = Math.max(0.01, distToEdge);
            double effectiveDist = Math.max(0.01, distToEdge - robotRadius);

            // Standard inverse-square repulsion
            double magnitude = strength * (1.0 / Math.pow(effectiveDist, 2));

            // Away vector (Normal to the diamond face)
            Translation2d awayForce = robotPos.minus(closestPoint).div(safeDist);

            // Funneling vector (Tangent to the diamond face)
            // We determine the direction of the edge and pick the one that goes toward target
            Translation2d edgeVector =
                    new Translation2d(halfWidth * Math.signum(-dx), halfHeight * Math.signum(dy));

            // Rotate edge vector based on target location to "slide" towards target
            Translation2d toTarget = targetPos.minus(robotPos);
            if (edgeVector.getDistance(toTarget) > edgeVector.unaryMinus().getDistance(toTarget)) {
                edgeVector = edgeVector.unaryMinus();
            }

            // Normalize tangent
            Translation2d tangent =
                    (edgeVector.getNorm() > 0)
                            ? edgeVector.div(edgeVector.getNorm())
                            : new Translation2d();

            // Combine: 100% away force + 80% tangential slide
            totalForce = awayForce.plus(tangent.times(0.8)).times(magnitude);
        }

        // 3. TIP STUDS (The 4 points of the diamond)
        // These act as extra "kickers" to prevent the robot from getting stuck at the points
        double[][] tips = {
            {x + halfWidth, y}, {x - halfWidth, y}, {x, y + halfHeight}, {x, y - halfHeight}
        };
        for (double[] tip : tips) {
            Translation2d tipPos = new Translation2d(tip[0], tip[1]);
            double distToTip = robotPos.getDistance(tipPos);

            if (distToTip < 0.5) { // Smaller influence for sharp tips
                double effectiveDist = Math.max(0.01, distToTip - robotRadius);
                double tipMagnitude = Math.min(strength * (1.5 / Math.pow(effectiveDist, 2)), 3.0);

                Translation2d kickDir = robotPos.minus(tipPos).div(Math.max(0.01, distToTip));
                totalForce = totalForce.plus(kickDir.times(tipMagnitude));
            }
        }

        return totalForce;
    }
}
