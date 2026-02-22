package igknighters.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelDisabled extends Flywheel {
    @Override
    public void setSpeed(AngularVelocity speed) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public AngularVelocity getSpeed() {
        // TODO Auto-generated method stub
        return RPM.of(0);
    }
}
