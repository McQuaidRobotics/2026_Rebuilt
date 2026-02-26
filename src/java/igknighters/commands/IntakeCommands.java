package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

    public static Command goTo(Intake intake, Angle angle, AngularVelocity speed) {
        return intake.run(() -> intake.goTo(angle, speed)).withName("Go to");
    }

    public static Command neutral(Intake intake) {
        return intake.run(() -> intake.goTo(Degrees.of(0), RPM.of(0))).withName("Neutral");
    }
}
