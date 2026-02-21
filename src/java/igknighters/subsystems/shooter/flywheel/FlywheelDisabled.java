package igknighters.subsystems.shooter.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelDisabled extends Flywheel {
    @Override
    public void setSpeed(AngularVelocity speedMetersPerSecond) {
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
    public double getSpeedRPM() {
        // TODO Auto-generated method stub
        return 0;
    }
}
