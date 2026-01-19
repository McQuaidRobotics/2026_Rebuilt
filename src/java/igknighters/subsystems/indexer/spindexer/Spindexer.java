package igknighters.subsystems.indexer.spindexer;

public abstract class Spindexer {
    public abstract void goToRPM(double RPM);

    public abstract void stop();

    public abstract double getRPM();

    public abstract void periodic();
}
