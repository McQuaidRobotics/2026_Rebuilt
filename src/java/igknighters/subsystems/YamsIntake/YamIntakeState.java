package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public enum YamIntakeState {
    STOWED(Degrees.of(0.0), RPM.of(0.0)),
    PARTIAL_STOW(Degrees.of(22.5), RPM.of(500.0)),
    DEPLOYED(Degrees.of(45.0), RPM.of(1000.0));

    Angle pivotAngle;
    AngularVelocity rollerVelocity;

    YamIntakeState(Angle pivotAngle, AngularVelocity rollerVelocity) {
        this.pivotAngle = pivotAngle;
        this.rollerVelocity = rollerVelocity;
    }
}
