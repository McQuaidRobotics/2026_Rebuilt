package igknighters.subsystems.intake.pivot;

import edu.wpi.first.units.measure.Angle;

public abstract class Pivot {

    protected double degrees;
    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngleDegrees(Angle angle);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(Angle angle);

    public abstract void stop();
}
