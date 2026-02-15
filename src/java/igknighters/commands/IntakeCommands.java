package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command intakeBalls(Intake intake) {
        return intake.run(() -> intake.goTo(0.0, 1000.0)).withName("Intake Balls");
    }
    public static Command goTo(Intake intake, double angle, double rpm) {
        return intake.run(() -> intake.goTo(angle, rpm)).withName("Go to");
    }
    public static Command neutral(Intake intake) {
        return intake.run(() -> intake.goTo(0, 0)).withName("Neutral");
    }
     
}
