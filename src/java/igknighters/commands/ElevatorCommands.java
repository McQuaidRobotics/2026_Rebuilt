package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.elevator.Elevator;

public class ElevatorCommands {
    /** Moves the elevator to a specific height and finishes when it reaches that height. */
    public static Command moveToHeight(Elevator elevator, double height) {
        return elevator.run(() -> elevator.moveToHeight(height))
                .until(() -> elevator.isAt(height, 0.01));
    }

    /** Holds the elevator at a specific height indefinitely. */
    public static Command holdAt(Elevator elevator, double height) {
        return elevator.run(() -> elevator.moveToHeight(height));
    }
}
