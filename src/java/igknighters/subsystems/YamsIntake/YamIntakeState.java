package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.Robot;

public enum YamIntakeState {
    STOWED(Degrees.of(Robot.consts.intake().kPivot().STOWED_ANGLE_DEGREES()), RPM.of(0.0)),
    PARTIAL_STOW(Degrees.of(Robot.consts.intake().kPivot().PARTIAL_STOW()), RPM.of(500.0)),
    DEPLOYED(Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()), RPM.of(4000));

    public final Angle pivotAngle;
    public final AngularVelocity rollerVelocity;

    YamIntakeState(Angle pivotAngle, AngularVelocity rollerVelocity) {
        this.pivotAngle = pivotAngle;
        this.rollerVelocity = rollerVelocity;
    }
}
