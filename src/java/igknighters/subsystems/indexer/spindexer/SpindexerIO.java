package igknighters.subsystems.indexer.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetVelocityRotationsPerSec = 0.0;
    }

    default void updateInputs(SpindexerIOInputs inputs) {}

    default void setVelocitySetpoint(AngularVelocity velocity) {}

    default void setVoltage(double voltage) {}

    default AngularVelocity getVelocity() {
        return RPM.of(0.0);
    }

    default void simIterate() {}

    default void updateTelemetry() {}
}
