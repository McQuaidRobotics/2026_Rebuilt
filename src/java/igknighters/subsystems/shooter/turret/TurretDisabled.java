package igknighters.subsystems.shooter.turret;

import edu.wpi.first.units.measure.Angle;

public class TurretDisabled extends Turret {

    @Override
    public void periodic() {
        // Do nothing
    }

    @Override
    public void setAngle(Angle angle) {
        // Do nothing
    }

    @Override
    public double getAngleDegrees() {
        return 0.0;
    }

    @Override
    public void goToAngleDegrees(Angle angle) {
        // Do nothing
    }
}
