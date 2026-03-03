package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class TurretSim extends Turret {

    public TurretSim() {}

    private boolean isControlledThisCycle = false;

    @Override
    public void setAngle(Angle angle) {
        double wrappedAngleDegrees = wrapAngleDegrees(angle.in(Degrees));
        super.degrees = wrappedAngleDegrees;
    }

    @Override
    public void goToAngleDegrees(Angle angle) {
        double wrappedAngleDegrees = wrapAngleDegrees(angle.in(Degrees));
        super.degrees = wrappedAngleDegrees;
        super.targetDegrees = wrappedAngleDegrees;
    }

    @Override
    public double getAngleDegrees() {
        return super.degrees;
    }

    @Override
    public void periodic() {

        // Log.log("LOGGING/Subsystems/Shooter/Turret/AngleDegrees", getAngleDegrees());
        // Log.log("LOGGING/Subsystems/Shooter/Turret/TargetDegrees", super.targetDegrees);

        isControlledThisCycle = false;
    }
}
