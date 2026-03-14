package igknighters.subsystems.shooter.hood;

import edu.wpi.first.units.measure.Angle;

public abstract class Hood {
    protected double targetDegrees = 0.0;

    public abstract void setAngle(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void periodic();

    public abstract boolean isSensorHit();

    public abstract void goToAngle(Angle angle);

    public abstract void setVoltage(double voltage);
}
