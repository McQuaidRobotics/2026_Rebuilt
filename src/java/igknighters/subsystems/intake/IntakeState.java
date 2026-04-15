package igknighters.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.Robot;
import java.util.function.Supplier;

public enum IntakeState implements StructSerializable {
    FULL_STOW(() -> Degrees.of(0.0), () -> RPM.of(0.0), () -> Degrees.of(5.0)),
    Intake(
            () -> Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()),
            () -> RPM.of(Robot.consts.intake().kRollers().MAX_SPEED_RPM()),
            () -> Degrees.of(5)),
    slightJork(
            () -> Degrees.of(Robot.consts.intake().kPivot().JORK_ANGLE()),
            () -> RPM.of(Robot.consts.intake().kRollers().MAX_SPEED_RPM()),
            () -> Degrees.of(5)),

    largeJork(
            () -> Degrees.of(Robot.consts.intake().kPivot().JORK_ANGLE()),
            () -> RPM.of(Robot.consts.intake().kRollers().MAX_SPEED_RPM()),
            () -> Degrees.of(5)),

    partialStow(
            () -> Degrees.of(Robot.consts.intake().kPivot().PARTIAL_STOW()),
            () -> RPM.of(100),
            () -> Degrees.of(5)),
    Stowed(
            () -> Degrees.of(Robot.consts.intake().kPivot().STOWED_ANGLE_DEGREES()),
            () -> RPM.of(-300),
            () -> Degrees.of(5));

    private final Supplier<Angle> pivotDegrees;
    private final Supplier<AngularVelocity> rollerSpeedRPM;
    private final Supplier<Angle> tolerenceDegrees;

    IntakeState(
            Supplier<Angle> pivotDegrees,
            Supplier<AngularVelocity> rollerSpeedRPM,
            Supplier<Angle> tolerenceDegrees) {
        this.pivotDegrees = pivotDegrees;
        this.rollerSpeedRPM = rollerSpeedRPM;
        this.tolerenceDegrees = tolerenceDegrees;
    }

    IntakeState(Supplier<Angle> pivotAngle, Supplier<AngularVelocity> rollerSpeed) {
        this(pivotAngle, rollerSpeed, () -> Degrees.of(5));
    }

    public Angle getPivotAngle() {
        return pivotDegrees.get();
    }

    public AngularVelocity getRollerSpeed() {
        return rollerSpeedRPM.get();
    }

    public Angle getTolerance() {
        return tolerenceDegrees.get();
    }
}
