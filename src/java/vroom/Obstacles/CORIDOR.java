package vroom.Obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class CORIDOR implements Obstacle {
    private final double x, y, width, height;
    private final boolean isHorizontal;
    private final double strength;

    public CORIDOR(
            double x,
            double y,
            double width,
            double height,
            double strength,
            boolean isHorizontal) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.isHorizontal = isHorizontal;
        this.strength = strength;
    }

    // will be used to delete all forces that conflict with direction of corridor eg horizontal
    // corridors will delete vertical forces
    @Override
    public Translation2d calculateForce(
            Translation2d robotPos,
            Translation2d targetPos,
            double robotRadius,
            Translation2d forceSoFar) {
        double currentForceX = forceSoFar.getX();
        double currentForceY = forceSoFar.getY();
        double x1 = x - width;
        double y1 = y - height;
        double x2 = x + width;
        double y2 = y + height;

        if (robotPos.getX() >= x1
                && robotPos.getX() <= x2
                && robotPos.getY() >= y1
                && robotPos.getY() <= y2) {

            if (isHorizontal) {
                // Delete vertical forces
                double sign = Math.signum(targetPos.getX() - robotPos.getX());
                return new Translation2d(-currentForceX + sign * strength, -currentForceY);
            } else {
                // Delete horizontal forces and push robot down
                double sign = Math.signum(targetPos.getY() - robotPos.getY());
                return new Translation2d(
                        -currentForceX,
                        -currentForceY + sign * strength); // over ride the force to ensure a push
            }
        }
        return new Translation2d();
    }
}
