package igknighters.subsystems.shooter.turret;

public class TurretDisabled extends Turret {

    @Override
    public void periodic() {
        // Do nothing
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        // Do nothing
    }

    @Override
    public double getAngleDegrees() {
        return 0.0;
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        // Do nothing
    }
}
