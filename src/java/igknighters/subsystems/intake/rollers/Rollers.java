package igknighters.subsystems.intake.rollers;

import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Rollers {

    protected final double speed = 0.0;
    protected final boolean isAtSpeed = false;
    protected final double targetSpeed = 0.0;

    public abstract void goToSpeed(AngularVelocity speed);

    public abstract AngularVelocity getSpeed();

    public abstract void stop();

    public abstract void periodic();
}
