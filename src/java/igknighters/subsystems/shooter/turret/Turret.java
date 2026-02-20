package igknighters.subsystems.shooter.turret;

import igknighters.constants.SubsystemConstants.kShooter.kTurret;

public abstract class Turret {
    public double wrapAngleDegrees(double angleDegrees) {
        double angle = angleDegrees;
        if (angle > kTurret.MAX_ANGLE_DEGREES) {
            angle -= 360.0;
        } else if (angle < kTurret.MIN_ANGLE_DEGREES) {
            angle += 360.0;
        }
        return angle;
    }

    

    protected double degrees;
    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(double angleDegrees);
}
