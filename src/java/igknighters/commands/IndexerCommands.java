package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.IndexerState;
import igknighters.subsystems.intake.IntakeState;

import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense(Indexer indexer) {
        return indexer.run(() -> indexer.goToState(IndexerState.DISPENSE_BALL))
                .alongWith(Commands.print("IM DISPENSING UHHHHHH"));
    }

    public static Command stopDispensing(Indexer indexer){
        return indexer.run(() -> indexer.goToState(IndexerState.PREP_TO_STOP)).withTimeout(3).andThen(
                indexer.run(() -> indexer.goToState(IndexerState.STOP))
        );
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
