package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command intakeBalls(Intake intake) {
        return intake.run(() -> intake.goTo(0.0, 1000.0)).withName("Intake Balls");
    }
    public static Command stow(Intake intake) {
        return intake.run(() -> intake.goTo(90.0, 0.0)).withName("Stow Intake");
    }
}
