package igknighters.subsystems.intake.pivot;

public abstract class Pivot {

    protected double degrees;
    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(double angleDegrees);

    public abstract void stop();
}
