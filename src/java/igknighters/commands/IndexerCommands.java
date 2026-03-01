package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.IndexerState;
import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.DISPENSE_BALL))
                .withName("DISPENSE");
    }

    public static Command stopDispensing(Indexer indexer) {
        return indexer.run(() -> indexer.goToState(IndexerState.PREP_TO_STOP))
                .withTimeout(3)
                .andThen(indexer.runOnce(() -> indexer.goToState(IndexerState.STOP)))
                .withName("STOPPING");
    }

    public static Command justStop(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.STOP)).withName("JUST STOP");
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
