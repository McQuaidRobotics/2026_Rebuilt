package igknighters.subsystems.YamsIntake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class YamIntake {
    public final Pivot pivot;
    public final Rollers rollers;

    public YamIntake() {
        this.pivot = new Pivot();
        this.rollers = new Rollers();
    }

    public Command targetState(YamIntakeState state) {
        return Commands.parallel(
                this.pivot.targetAngle(state.pivotAngle),
                this.rollers.targetVelocity(state.rollerVelocity));
    }

    public Command jorkIntake() {
        return Commands.sequence(
                        targetState(YamIntakeState.DEPLOYED).withTimeout(1),
                        targetState(YamIntakeState.STOWED).withTimeout(1))
                .repeatedly();
    }

    public Angle getPivotAngle() {
        return this.pivot.getAngle();
    }

    public AngularVelocity getRollerVelocity() {
        return this.rollers.getVelocity();
    }

    public boolean isAt(YamIntakeState state, Angle tolerance, AngularVelocity velocityTolerance) {
        return this.pivot.isAt(state.pivotAngle, tolerance)
                && this.rollers.isAt(state.rollerVelocity, velocityTolerance);
    }
}
