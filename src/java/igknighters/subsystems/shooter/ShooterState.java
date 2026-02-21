package igknighters.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterState {
    public final AngularVelocity flywheelSpeed;
    public final Angle turretAngle;
    public final Angle hoodAngle;

    public ShooterState(AngularVelocity rollerSpeed, Angle turretAngle, Angle hoodAngle) {
        this.flywheelSpeed = rollerSpeed;
        this.turretAngle = turretAngle;
        this.hoodAngle = hoodAngle;
    }
}
