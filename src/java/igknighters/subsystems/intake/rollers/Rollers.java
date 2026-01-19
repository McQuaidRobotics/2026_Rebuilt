package igknighters.subsystems.intake.rollers;

public abstract class Rollers {

    protected final double speed = 0.0;
    protected final boolean isAtSpeed = false;
    protected final double targetSpeed = 0.0;

    public abstract void goToSpeedRPM(double speed);

    public abstract double getSpeedRPM();

    public abstract void stop();

    public abstract void periodic();
}
