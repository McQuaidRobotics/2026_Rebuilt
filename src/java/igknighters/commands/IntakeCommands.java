package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.intake.AbstractIntake;
import igknighters.subsystems.intake.IntakeState;
import igknighters.util.log.Log;

public class IntakeCommands {
    public static boolean toggledState = true;

    // called

    /**
     * Holds the AbstractIntakein the AbstractIntakeposition. This will not end unless a new command
     * is called on the intake
     *
     * @param intake
     * @return
     */
    public static Command holdAtIntake(AbstractIntake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Intake)).withName("AbstractIntakeBalls");
    }

    /**
     * Holds the AbstractIntakein the stowed position. This will not end unless a new command is
     * called on the intake
     *
     * @param intake
     * @return
     */
    public static Command holdAtStow(AbstractIntake intake) {
        return intake.run(() -> intake.goTo(IntakeState.Stowed)).withName("Stow Intake");
    }

    public static Command toggleHoldState(AbstractIntake intake) {
        return intake.startRun(() -> toggledState = !toggledState, () -> intake.goTo(toggledState))
                .withName("AbstractIntakeBalls");
    }

    public static Command expell(AbstractIntake intake) {
        return intake.run(() -> intake.setRollerSpeed(RPM.of(-3000))).withName("Expell Balls");
    }

    /**
     * Instantly holds the AbstractIntakeat a specified state. This will instantly afterwards. It
     * relies on the motors pid controller holding state. Should be called repeatedly
     *
     * @param intake
     * @param state
     * @return
     */
    public static Command instantHoldAtState(AbstractIntake intake, IntakeState state) {
        return intake.runOnce(() -> intake.goTo(state)).withName("Instant Hold at State");
    }

    public static Command holdAtState(AbstractIntake intake, IntakeState state) {
        return intake.run(() -> intake.goTo(state)).withName("Hold at State");
    }

    public static Command jorkIt(AbstractIntake intake) {
        return holdAtIntake(intake)
                .withTimeout(.5)
                .andThen(holdAtStow(intake))
                .withTimeout(.5)
                .repeatedly()
                .withName("JORK INTAKE");
    }

    public static Command slightJorkIntake(AbstractIntake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.2),
                        holdAtState(intake, IntakeState.slightJork).withTimeout(.2))
                .repeatedly()
                .withName("Slight Jork");
    }

    public static Command largeJorkIntake(AbstractIntake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.2),
                        holdAtState(intake, IntakeState.largeJork).withTimeout(.2))
                .repeatedly()
                .withName("Large Jork");
    }

    public static Command intakeWhileSlightJorking(AbstractIntake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.7),
                        holdAtState(intake, IntakeState.slightJork).withTimeout(.2))
                .repeatedly()
                .withName("Slight Jork-y Intake-y");
    }

    public static Command protectedIntake(AbstractIntake intake) {
        return intake.run(
                () -> {
                    // if on bump we should be stowed
                    if (FieldConstants.BUMP.isInside(Robot.pose_pred.getDynamicPredictedPose())) {
                        Log.log("ROBOT/Commands/Protected Intake", "Inside BUMP, stowing intake");
                        holdAtStow(intake);
                    } else {
                        Log.log("ROBOT/Commands/Protected Intake", "Outside BUMP, holding intake");
                        holdAtIntake(intake);
                    }
                });
    }

    public static Command holdAt(AbstractIntake intake, Angle angle, AngularVelocity speed) {
        return intake.run(() -> intake.goTo(angle, speed)).withName("Go to");
    }

    public static Command neutral(AbstractIntake intake) {
        return intake.run(() -> intake.goTo(Degrees.of(0), RPM.of(0))).withName("Neutral");
    }
}
