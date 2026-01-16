package igknighters.subsystems.shooter.hood;

public abstract class Hood {
    protected double targetDegrees = 0.0;

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void periodic();

    public abstract void goToAngleDegrees(double angleDegrees);
}
