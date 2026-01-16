package igknighters.subsystems.intake;

import edu.wpi.first.util.struct.StructSerializable;

public enum IntakeState implements StructSerializable {
    Intake(0, 1500, 5),
    Stowed(90, 0, 5);

    public final double pivotDegrees;
    public final double rollerSpeedRPM;
    public final double tolerenceDegrees;

    IntakeState(double pivotDegrees, double rollerSpeedRPM, double tolerenceDegrees) {
        this.pivotDegrees = pivotDegrees;
        this.rollerSpeedRPM = rollerSpeedRPM;
        this.tolerenceDegrees = tolerenceDegrees;
    }

    IntakeState(double pivotDegrees, double rollerSpeedRPM) {
        this(pivotDegrees, rollerSpeedRPM, 5);
    }
}
