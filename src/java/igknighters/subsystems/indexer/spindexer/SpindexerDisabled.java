package igknighters.subsystems.indexer.spindexer;

public class SpindexerDisabled extends Spindexer {

    @Override
    public void goToRPM(double RPM) {}

    @Override
    public double getRPM() {
        return 0.0;
    }

    @Override
    public void stop() {}

    @Override
    public void periodic() {}
}
