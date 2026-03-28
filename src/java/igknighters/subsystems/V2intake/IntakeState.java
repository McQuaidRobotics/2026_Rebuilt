package igknighters.subsystems.V2intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.struct.StructSerializable;

public enum IntakeState implements StructSerializable {
    Intake(Degrees.of(66), RPM.of(3500), Degrees.of(5)),
    slightJork(Degrees.of(55), RPM.of(3500), Degrees.of(5)),
    PREP_TO_STOW(Degrees.of(30), RPM.of(500), Degrees.of(10)),
    Stowed(Degrees.of(0), RPM.of(100), Degrees.of(5));

    public final Angle pivotDegrees;
    public final AngularVelocity rollerSpeedRPM;
    public final Angle tolerenceDegrees;

    IntakeState(Angle pivotDegrees, AngularVelocity rollerSpeedRPM, Angle tolerenceDegrees) {
        this.pivotDegrees = pivotDegrees;
        this.rollerSpeedRPM = rollerSpeedRPM;
        this.tolerenceDegrees = tolerenceDegrees;
    }

    IntakeState(Angle pivotAngle, AngularVelocity rollerSpeed) {
        this(pivotAngle, rollerSpeed, Degrees.of(5));
    }
}
