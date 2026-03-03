package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.IndexerState;
import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.DISPENSE_BALL))
                .withName("DISPENSE");
    }

    public static Command jorkIt(Indexer indexer) {
        return indexer.run(() -> indexer.goToState(IndexerState.JORK_BACKWARD))
                .withTimeout(.05)
                .andThen(
                        indexer.run(() -> indexer.goToState(IndexerState.JORK_FORWARD))
                                .withTimeout(.05))
                .andThen(indexer.runOnce(() -> indexer.goToState(IndexerState.STOP)))
                .andThen(Commands.waitSeconds(.25));
    }

    public static Command unBlock(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.AGITATE));
    }

    public static Command justStop(Indexer indexer) {
        return indexer.runOnce(() -> indexer.goToState(IndexerState.STOP)).withName("JUST STOP");
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
