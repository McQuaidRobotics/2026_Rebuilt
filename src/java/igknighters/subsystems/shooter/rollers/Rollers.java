package igknighters.subsystems.shooter.rollers;

public abstract class Rollers {

    public abstract void setSpeed(double speedMetersPerSecond);

    public abstract void setVoltage(double voltage);

    public abstract void periodic();
}
