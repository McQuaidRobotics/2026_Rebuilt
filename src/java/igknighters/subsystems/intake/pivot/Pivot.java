package igknighters.subsystems.intake.pivot;

public abstract class Pivot {
    public abstract void goToAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void stop();

    public abstract void periodic();

    public abstract void setAngleDegrees(double angleDegrees);
}
