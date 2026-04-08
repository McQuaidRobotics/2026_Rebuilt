package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.intake.IntakeState;
import igknighters.util.log.Log;

public class IntakeCommands {
    public static boolean toggledState = true;

    // called

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

    public static Command toggleHoldState(Intake intake) {
        return intake.startRun(() -> toggledState = !toggledState, () -> intake.goTo(toggledState))
                .withName("Intake Balls");
    }

    public static Command expell(Intake intake) {
        return intake.run(() -> intake.setRollerSpeed(RPM.of(-3000))).withName("Expell Balls");
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

    public static Command holdAtState(Intake intake, IntakeState state) {
        return intake.run(() -> intake.goTo(state)).withName("Hold at State");
    }

    public static Command jorkIt(Intake intake) {
        return holdAtIntake(intake)
                .withTimeout(.5)
                .andThen(holdAtStow(intake))
                .withTimeout(.5)
                .repeatedly()
                .withName("JORK INTAKE");
    }

    public static Command slightJorkIntake(Intake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.2),
                        holdAtState(intake, IntakeState.slightJork).withTimeout(.2))
                .repeatedly()
                .withName("Slight Jork");
    }

    public static Command largeJorkIntake(Intake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.2),
                        holdAtState(intake, IntakeState.largeJork).withTimeout(.2))
                .repeatedly()
                .withName("Large Jork");
    }

    public static Command intakeWhileSlightJorking(Intake intake) {
        return Commands.sequence(
                        holdAtIntake(intake).withTimeout(.7),
                        holdAtState(intake, IntakeState.slightJork).withTimeout(.2))
                .repeatedly()
                .withName("Slight Jork-y Intake-y");
    }

    public static Command protectedIntake(Intake intake) {
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

    public static Command holdAt(Intake intake, Angle angle, AngularVelocity speed) {
        return intake.run(() -> intake.goTo(angle, speed)).withName("Go to");
    }

    public static Command neutral(Intake intake) {
        return intake.run(() -> intake.goTo(Degrees.of(0), RPM.of(0))).withName("Neutral");
    }
}
