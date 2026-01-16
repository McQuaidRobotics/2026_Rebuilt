package igknighters.subsystems.shooter.turret;

public abstract class Turret {

    protected double degrees;
    protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(double angleDegrees);
}
