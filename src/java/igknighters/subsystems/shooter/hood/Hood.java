package igknighters.subsystems.shooter.hood;

import edu.wpi.first.epilogue.Logged;

@Logged
public abstract class Hood {
    @Logged(name = "target_in_degrees")
    protected double targetDegrees = 0.0;

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void periodic();

    public abstract void goToAngleDegrees(double angleDegrees);
}
