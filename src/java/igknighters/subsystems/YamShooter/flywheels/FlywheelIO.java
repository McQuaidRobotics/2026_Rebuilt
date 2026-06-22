package igknighters.subsystems.YamShooter.flywheels;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetVelocityRotationsPerSec = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocitySetpoint(AngularVelocity velocity) {}

    default void setVoltage(double voltage) {}

    default AngularVelocity getVelocity() {
        return RPM.of(0.0);
    }

    default void simIterate() {}

    default void updateTelemetry() {}
}
