package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;

public class HoodSim extends Hood {
    double currentAngleDegrees = 0.0;

    @Override
    public void periodic() {

        DogLog.log("Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
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
