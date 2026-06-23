package igknighters.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Turret extends SubsystemBase {
    public abstract void targetAngle(Angle angle);

    public abstract Angle getCurentAngle();
}
