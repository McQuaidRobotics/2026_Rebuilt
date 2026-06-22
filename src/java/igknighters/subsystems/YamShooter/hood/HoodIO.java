package igknighters.subsystems.YamShooter.hood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    /**
     * Inputs that will be logged and replayed. The @AutoLog annotation generates
     * HoodIOInputsAutoLogged class.
     */
    @AutoLog
    public static class HoodIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetPositionRotations = 0.0;
        public boolean limitSwitchTripped = false;
    }

    /** Update the inputs from hardware. Called every loop cycle. */
    default void updateInputs(HoodIOInputs inputs) {}

    /** Set the target angle for the arm. */
    default void setTargetAngle(Angle angle) {}

    default void zeroAt(Angle angle) {}

    default boolean isLimitSwitchTripped() {
        return false;
    }

    default void setVoltage(double voltage) {}

    default void zeroHoodCheck() {}

    default void updateTelemetry() {}

    default void simIterate() {}
}
