package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import igknighters.util.log.Log;

public class HoodSim extends Hood {
    double currentAngleDegrees = 0.0;

    @Override
    public void periodic() {

        Log.log("ROBOT/Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
        Log.log("ROBOT/Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
    }

    @Override
    public void setAngle(double angleDegrees) {
        currentAngleDegrees = angleDegrees;
    }

    @Override
    public void goToAngle(Angle angle) {
        super.targetDegrees = angle.in(Degrees);

        currentAngleDegrees = angle.in(Degrees);
    }

    @Override
    public double getAngleDegrees() {
        // Convert RADIANS from sim back to DEGREES for your robot logic
        return currentAngleDegrees;
    }
}
