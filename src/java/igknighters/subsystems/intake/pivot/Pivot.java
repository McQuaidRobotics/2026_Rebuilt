package igknighters.subsystems.intake.pivot;

import edu.wpi.first.units.measure.Angle;

public abstract class Pivot {

    protected double degrees;
    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngle(Angle angle);

    public abstract Angle getAngle();

    public abstract void goToAngle(Angle angle);

    public abstract void stop();
}
