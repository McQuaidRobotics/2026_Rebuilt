package igknighters.subsystems.shooter.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Flywheel {

    public abstract void setSpeed(AngularVelocity speedRPM);

    public abstract void setVoltage(double voltage);

    public abstract void periodic();

    public abstract AngularVelocity getSpeed();
}
