package igknighters.subsystems.shooter.flywheel;

public abstract class Flywheel {

    public abstract void setSpeedRPM(double speedRPM);

    public abstract void setVoltage(double voltage);

    public abstract void periodic();

    public abstract double getSpeedRPM();
}
