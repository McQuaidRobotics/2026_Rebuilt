package igknighters.subsystems.YamsIntake.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetPositionRotations = 0.0;
    }

    /** Update the inputs from hardware. Called every loop cycle. */
    default void updateInputs(PivotIOInputs inputs) {}

    /** Set the target angle for the arm. */
    default void setAngleSetpoint(Angle angle) {}

    default Angle getAngle() {
        return Degrees.of(0.0);
    }

    default void updateTelemetry() {}

    default void simIterate() {}
}
