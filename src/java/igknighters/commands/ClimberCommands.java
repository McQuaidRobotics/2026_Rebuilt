package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.climber.Climber;

public class ClimberCommands {
    public static Command goToMax(Climber climber) {
        return climber.run(() -> climber.goToInches(SubsystemConstants.kClimber.MAX_HEIGHT_INCHES))
                .until(() -> climber.isAt(SubsystemConstants.kClimber.MAX_HEIGHT_INCHES, .5))
                .withName("GOING TO MIN");
    }

    public static Command goToMin(Climber climber) {
        return climber.run(() -> climber.goToInches(SubsystemConstants.kClimber.MIN_HEIGHT_INCHES))
                .until(() -> climber.isAt(SubsystemConstants.kClimber.MIN_HEIGHT_INCHES, .5))
                .withName("GOING TO MIN");
    }

    public static Command goTo(Climber climber, double inches) {
        return climber.run(() -> climber.goToInches(inches))
                .until(() -> climber.isAt(inches, .5))
                .withName("GOING TO: " + inches);
    }
}
