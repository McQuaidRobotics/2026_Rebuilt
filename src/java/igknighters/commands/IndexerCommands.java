package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

// this is a placehoder because we dont have an indexer in code yet
public class IndexerCommands {

    public static Command dispense() {
        return Commands.none();
    }

    public static BooleanSupplier isBallPresent() {
        return () -> false;
    }
}
