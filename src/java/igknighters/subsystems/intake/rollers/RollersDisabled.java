package igknighters.subsystems.intake.rollers;

import edu.wpi.first.units.measure.AngularVelocity;

public class RollersDisabled extends Rollers {

    @Override
    public void goToSpeed(AngularVelocity speed) {
        // Do nothing
    }

    @Override
    public double getSpeedRPM() {
        return 0.0;
    }

    @Override
    public void stop() {
        // Do nothing
    }

    @Override
    public void periodic() {
        // Do nothing
    }
}
