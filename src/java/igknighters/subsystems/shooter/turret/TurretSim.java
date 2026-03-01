package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import igknighters.util.log.Log;

public class TurretSim extends Turret {

    public TurretSim() {}

    private boolean isControlledThisCycle = false;

    @Override
    public void setAngle(Angle angle) {
        double wrappedAngleDegrees = wrapAngleDegrees(angle.in(Degrees));
        super.degrees = angle.in(Degrees);
    }

    @Override
    public void goToAngleDegrees(Angle angle) {
        super.degrees = angle.in(Degrees);
        super.targetDegrees = angle.in(Degrees);
    }

    @Override
    public double getAngleDegrees() {
        return super.degrees;
    }

    @Override
    public void periodic() {

        Log.log("Subsystems/Shooter/Turret/AngleDegrees", getAngleDegrees());
        Log.log("Subsystems/Shooter/Turret/TargetDegrees", super.targetDegrees);

        isControlledThisCycle = false;
    }
}
