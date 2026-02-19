package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.climber.ClimberState;
import java.util.function.BooleanSupplier;

public class ClimberCommands {
    public static Command goToMax(Climber climber) {
        return climber.run(
                        () ->
                                climber.goToInches(
                                        SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES))
                .until(
                        () ->
                                climber.isAt(
                                        SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES,
                                        .5))
                .withName("GOING TO MAX");
    }

    public static Command goToMin(Climber climber) {
        return climber.run(
                        () ->
                                climber.goToInches(
                                        SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES))
                .until(
                        () ->
                                climber.isAt(
                                        SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES,
                                        .5))
                .withName("GOING TO MIN");
    }

    public static Command goTo(Climber climber, double inches) {
        return climber.run(() -> climber.goToInches(inches))
                .until(() -> climber.isAt(inches, .5))
                .withName("GOING TO: " + inches);
    }

    public static BooleanSupplier isBumperPressed(Climber climber) {
        return () -> climber.isSensorHit();
    }

    public static Command goToState(Climber climber, ClimberState state) {
        return climber.run(() -> climber.goToState(state));
    }

    public static Command home(Climber climber) {
        return climber.runOnce(
                () -> climber.setPositionInches(0.0)); // Set current position to 0 inches
    }
}
