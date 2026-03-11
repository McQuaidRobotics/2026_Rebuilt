package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.IntakeState;
import igknighters.util.log.Log;
import java.util.function.Supplier;

public class IntakeCommands {
    /**
     * Holds the intake in the intake position. This will not end unless a new command is called on
     * the intake
     *
     * @param intake
     * @return
     */
    public static Command holdAtIntake(Intake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Intake)).withName("Intake Balls");
    }

    /**
     * Holds the intake in the stowed position. This will not end unless a new command is called on
     * the intake
     *
     * @param intake
     * @return
     */
    public static Command holdAtStow(Intake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Stowed)).withName("Stow Intake");
    }

    /**
     * Instantly holds the intake at a specified state. This will instantly afterwards. It relies on
     * the motors pid controller holding state. Should be called repeatedly
     *
     * @param intake
     * @param state
     * @return
     */
    public static Command instantHoldAtState(Intake intake, IntakeState state) {
        return intake.runOnce(() -> intake.goTo(state)).withName("Instant Hold at State");
    }

    public static Command jorkIt(Intake intake) {
        return holdAtIntake(intake)
                .withTimeout(.5)
                .andThen(holdAtStow(intake))
                .withTimeout(.5)
                .withName("JORK INTAKE");
    }

    public static Command protectedIntake(Intake intake, Supplier<Pose2d> poseSupplier) {
        return intake.run(
                () -> {
                    // if on bump we should be stowed
                    if (FieldConstants.BUMP.isInside(poseSupplier.get())) {
                        Log.log("ROBOT/Commands/Protected Intake", "Inside BUMP, stowing intake");
                        instantHoldAtState(intake, IntakeState.Stowed);
                    } else {
                        Log.log("ROBOT/Commands/Protected Intake", "Outside BUMP, holding intake");
                        instantHoldAtState(intake, IntakeState.Intake);
                    }
                });
    }

    public static Command holdAt(Intake intake, Angle angle, AngularVelocity speed) {
        return intake.run(() -> intake.goTo(angle, speed)).withName("Go to");
    }

    public static Command neutral(Intake intake) {
        return intake.run(() -> intake.goTo(Degrees.of(0), RPM.of(0))).withName("Neutral");
    }
}
