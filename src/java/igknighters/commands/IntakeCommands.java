package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.IntakeState;

public class IntakeCommands {
    public static Command goToIntake(Intake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Intake)).withName("Intake Balls");
    }

    public static Command goToStow(Intake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Stowed)).withName("Stow Intake");
    }

    public static Command goTo(Intake intake, double angle, double rpm) {
        return intake.run(() -> intake.goTo(angle, rpm)).withName("Go to");
    }

    public static Command neutral(Intake intake) {
        return intake.run(() -> intake.goTo(0, 0)).withName("Neutral");
    }
}
