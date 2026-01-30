package igknighters.subsystems.indexer.launcherRollers;

public abstract class ExitRollers {
    abstract void setSpeedRPM(double rpm);

    abstract void setVoltage(double voltage);

    abstract double getSpeedRPM();

    abstract boolean isAtSpeed(double targetRPM, double toleranceRPM);

    abstract void periodic();
}
