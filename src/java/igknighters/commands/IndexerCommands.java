package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.Indexer;
import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense(Indexer indexer, double RPM) {
        return indexer.run(() -> indexer.setRPM(RPM))
                .alongWith(Commands.print("IM DISPENSING UHHHHHH"));
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
