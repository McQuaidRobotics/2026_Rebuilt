package igknighters.subsystems.YamShooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    /**
     * Inputs that will be logged and replayed. The @AutoLog annotation generates
     * TurretIOInputsAutoLogged class.
     */
    @AutoLog
    public static class TurretIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetPositionRotations = 0.0;
    }

    /** Update the inputs from hardware. Called every loop cycle. */
    default void updateInputs(TurretIOInputs inputs) {}

    /** Set the target angle for the arm. */
    default void setTargetAngle(double rotations) {}

    /** Stop the arm motor. */
    default void stop() {}
}
