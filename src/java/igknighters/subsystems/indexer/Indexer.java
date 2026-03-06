package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.launcherRollers.ExitRollers;
import igknighters.subsystems.indexer.spindexer.Spindexer;

public class Indexer {
    public ExitRollers exitRollers = new ExitRollers();
    public Spindexer spindexer = new Spindexer();

    public Indexer() {}

    public Command idle() {
        return spindexer
                .jorkRepeating()
                .alongWith(exitRollers.holdSpeed(RPM.of(0.0)))
                .withName("INDEXER IDLE");
    }

    public Command dispense() {
        return spindexer
                .holdAtState(IndexerState.DISPENSE_BALL)
                .alongWith(exitRollers.holdAtState(IndexerState.DISPENSE_BALL))
                .withName("INDEXER DISPENSE");
    }
}
