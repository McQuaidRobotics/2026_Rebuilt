package igknighters.subsystems.indexer;

public enum IndexerState {
    IDLE(0.0, 0.0),
    DISPENSE_BALL(100.0, 100.0);

    public final double spindexerRPM;
    public final double exitRollerRPM;

    private IndexerState(double spindexerRPM, double exitRollerRPM) {
        this.spindexerRPM = spindexerRPM;
        this.exitRollerRPM = exitRollerRPM;
    }


}
