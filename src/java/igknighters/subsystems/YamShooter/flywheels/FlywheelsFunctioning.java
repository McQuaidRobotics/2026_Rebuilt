package igknighters.subsystems.YamShooter.flywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelsFunctioning extends Flywheels implements FlywheelIO {

    private TalonFX followerMotor =
            new TalonFX(
                    Robot.consts.shooter().kFlywheels().FOLLOWER_MOTOR_ID(),
                    Robot.consts.shooter().kCANBUS());
    private SmartMotorControllerConfig smcConfig =
            Robot.consts.shooter().kFlywheels().getConfig(this, followerMotor);

    private TalonFX leaderMotor =
            new TalonFX(
                    Robot.consts.shooter().kFlywheels().LEADER_MOTOR_ID(),
                    Robot.consts.shooter().kCANBUS());

    private TalonFXConfiguration getConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 35;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    public FlywheelsFunctioning() {
        followerMotor.getConfigurator().apply(getConfig()); // give folower current limits
    }

    private SmartMotorController talonSMC =
            new TalonFXWrapper(leaderMotor, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig shooterConfig =
            new FlyWheelConfig(talonSMC)
                    // Diameter of the flywheel.
                    .withDiameter(Inches.of(4))
                    // Mass of the flywheel.
                    .withMass(Pounds.of(3))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("Flywheel Mechanism", TelemetryVerbosity.HIGH);

    private FlyWheel shooter = new FlyWheel(shooterConfig);

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        shooter.updateTelemetry();
        updateInputs(null);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRotationsPerSec = talonSMC.getMechanismVelocity().in(RotationsPerSecond);
        inputs.appliedVolts = talonSMC.getVoltage().in(Volts);
        inputs.supplyCurrentAmps = talonSMC.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = talonSMC.getStatorCurrent().in(Amps);
        inputs.temperatureCelsius = talonSMC.getTemperature().in(Celsius);
        inputs.targetVelocityRotationsPerSec =
                talonSMC.getMechanismSetpointVelocity()
                        .map(v -> v.in(RotationsPerSecond))
                        .orElse(0.0);
    }

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    @Override
    public void setVoltage(double voltage) {
        shooter.set(voltage / 12);
    }

    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return shooter.run(speed);
    }

    /**
     * Set the shooter velocity setpoint.
     *
     * @param speed Speed to set
     */
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }
}
