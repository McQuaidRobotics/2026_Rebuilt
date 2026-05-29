package igknighters.subsystems.YamShooter.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Turret extends SubsystemBase {
    public abstract Command targetAngle(Angle angle);

    public abstract Command setAngleAndStop(Angle angle, Angle tolerance);

    public abstract void setAngleSetpoint(Angle angle);

    public abstract Angle getAngle();

    public abstract boolean isAt(Angle angle, Angle tolerance);
}
