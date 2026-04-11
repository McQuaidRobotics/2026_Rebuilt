package igknighters.subsystems.intake.rollers;

import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Rollers {

    public abstract void goToSpeed(AngularVelocity speed);

    public abstract AngularVelocity getSpeed();

    public abstract void stop();

    public abstract void periodic();
}
