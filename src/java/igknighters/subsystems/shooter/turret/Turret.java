package igknighters.subsystems.shooter.turret;

import edu.wpi.first.units.measure.Angle;
import igknighters.Robot;

public abstract class Turret {
    public double wrapAngleDegrees(double angleDegrees) {
        double angle = angleDegrees;
        if (angle > Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES()) {
        if (angle > Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES()) {
            angle -= 360.0;
        } else if (angle < Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES()) {
            angle += 360.0;
        }
        return angle;
    }

    protected double degrees;

    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngle(Angle angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(Angle angleDegrees);
}
