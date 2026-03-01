package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class RollersDisabled extends Rollers {

    @Override
    public void goToSpeed(AngularVelocity speed) {
        // Do nothing
    }

    @Override
    public AngularVelocity getSpeed() {
        return RPM.of(0.0);
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
