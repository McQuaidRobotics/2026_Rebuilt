package igknighters.subsystems.shooter.turret;

import edu.wpi.first.epilogue.Logged;
import igknighters.subsystems.shooter.Shooter;

@Logged
public abstract class Turret extends Shooter {

    @Logged protected double degrees;
    @Logged protected double targetDegrees;

    public abstract void periodic();

    public abstract void setAngleDegrees(double angleDegrees);

    public abstract double getAngleDegrees();

    public abstract void goToAngleDegrees(double angleDegrees);
}
