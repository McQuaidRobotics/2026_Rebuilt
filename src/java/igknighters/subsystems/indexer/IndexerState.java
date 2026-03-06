package igknighters.subsystems.indexer;

public enum IndexerState {
    STOP(0.0, 0.0),
    DISPENSE_BALL(4800.0, 4800.0),
    PREP_TO_STOP(0.0, 4800.0),
    AGITATE(-500, -500),
    JORK_FORWARD(100, 0),
    JORK_BACKWARD(-100, 0);

    public final double spindexerRPM;
    public final double exitRollerRPM;

    private IndexerState(double spindexerRPM, double exitRollerRPM) {
        this.spindexerRPM = spindexerRPM;
        this.exitRollerRPM = exitRollerRPM;
    }
}
