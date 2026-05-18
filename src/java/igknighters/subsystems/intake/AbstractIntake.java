package igknighters.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractIntake extends SubsystemBase {
    public abstract Angle getPivotAngle();

    public abstract AngularVelocity getRollerSpeed();

    public abstract boolean isAt(
            Angle angle, AngularVelocity velo, Angle angleTolerance, AngularVelocity veloTolerance);

    public abstract void goTo(Angle angle, AngularVelocity velocity);

    public abstract void goTo(boolean targetState);

    public abstract void goTo(IntakeState state);

    public abstract void setRollerSpeed(AngularVelocity speed);

    public abstract boolean isStowed();

    public abstract void setMode(Intake.Mode mode);

    public abstract Intake.Mode getMode();
}
