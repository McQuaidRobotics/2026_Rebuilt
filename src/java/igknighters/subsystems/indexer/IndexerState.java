package igknighters.subsystems.indexer;

import igknighters.Robot;
import java.util.function.Supplier;

public enum IndexerState {
    STOP(() -> 0.0, () -> 0.0),
    DISPENSE_BALL(
            () -> Robot.consts.indexer().kSpindexer().MAX_SPEED_RPM(),
            () -> Robot.consts.indexer().kExitRollers().MAX_SPEED_RPM()),
    PREP_TO_STOP(() -> 0.0, () -> Robot.consts.indexer().kExitRollers().MAX_SPEED_RPM()),
    AGITATE(() -> -500.0, () -> -500.0),
    JORK_FORWARD(() -> 100.0, () -> 0.0),
    JORK_BACKWARD(() -> -100.0, () -> 0.0);

    private final Supplier<Double> spindexerRPM;
    private final Supplier<Double> exitRollerRPM;

    private IndexerState(Supplier<Double> spindexerRPM, Supplier<Double> exitRollerRPM) {
        this.spindexerRPM = spindexerRPM;
        this.exitRollerRPM = exitRollerRPM;
    }

    public double getSpindexerRPM() {
        return spindexerRPM.get();
    }

    public double getExitRollerRPM() {
        return exitRollerRPM.get();
    }
}
