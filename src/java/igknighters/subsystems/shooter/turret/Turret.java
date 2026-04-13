package igknighters.subsystems.shooter.turret;

import edu.wpi.first.units.measure.Angle;
import igknighters.Robot;
import igknighters.util.log.Log;

public abstract class Turret {
    public double wrapAngleDegrees(double angleDegrees) {
        Log.log("ROBOT/Subsystems/Shooter/Turret/pre wrapped angle", angleDegrees);
        double angle = angleDegrees;
        if (angle > Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES()) {
            angle -= 360.0;
        } else if (angle < Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES()) {
            angle += 360.0;
        }
        return angle;
    }

    public boolean isLegalPositionWrapped(double angleDegrees) {
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees);
        return isLegalPosition(wrappedAngleDegrees);
        // test
    }

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES()
                && angleDegrees <= Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES();
    }

    protected double degrees;

    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngle(Angle angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(Angle angleDegrees);
}
