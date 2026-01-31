package igknighters.subsystems.indexer.launcherRollers;

public abstract class ExitRollers {
    public abstract void setSpeedRPM(double rpm);

    public abstract void setVoltage(double voltage);

    public abstract double getSpeedRPM();

    public abstract boolean isAtSpeed(double targetRPM, double toleranceRPM);

    public abstract void periodic();
}
