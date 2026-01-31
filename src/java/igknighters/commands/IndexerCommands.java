package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.IndexerState;
import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense(Indexer indexer) {
        return indexer.run(() -> indexer.goToState(IndexerState.DISPENSE_BALL));
    }

    public static Command stop(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.STOP));
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
