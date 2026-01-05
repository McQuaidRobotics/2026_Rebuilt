package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.elevator.Elevator;

public class ElevatorComands {
    public static Command MoveToHeightCommand(Elevator elevator, double height) {
        return elevator.run(() -> elevator.moveToHeight(height))
                .until(() -> elevator.isAt(height, 0.01))
                .withName("MOVING TO " + height + " METERS");
    }
}
