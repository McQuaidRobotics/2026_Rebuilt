package igknighters.subsystems.shooter.hood;

import edu.wpi.first.units.measure.Angle;

public class HoodDisabled extends Hood {
    @Override
    public void setAngle(double angleDegrees) {
        // Do nothing
    }

    @Override
    public double getAngleDegrees() {
        return 0.0;
    }

    @Override
    public void periodic() {
        // Do nothing
    }

    @Override
    public boolean isSensorHit() {
        return false;
    }

    @Override
    public void goToAngle(Angle angle) {
        // Do nothing
    }

    @Override
    public void setVoltage(double voltage) {
        // Do nothing
    }
}
