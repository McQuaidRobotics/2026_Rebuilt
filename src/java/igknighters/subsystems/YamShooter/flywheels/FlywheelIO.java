package igknighters.subsystems.YamShooter.flywheels;

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
}
