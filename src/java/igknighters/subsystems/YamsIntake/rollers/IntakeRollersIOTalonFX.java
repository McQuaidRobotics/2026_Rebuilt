package igknighters.subsystems.YamsIntake.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {

    private SmartMotorControllerConfig smcConfig;

    private SmartMotorController talonSMC;

    private final FlyWheelConfig shooterConfig;

    private FlyWheel shooter;

    private TalonFX leaderMotor =
            new TalonFX(
                    Robot.consts.intake().kRollers().LEADER_MOTOR_ID(),
                    Robot.consts.intake().kCANBUS());

    public IntakeRollersIOTalonFX(SubsystemBase subsystem) {
        smcConfig = Robot.consts.intake().kRollers().getConfig(subsystem);
        talonSMC = new TalonFXWrapper(leaderMotor, DCMotor.getKrakenX60(1), smcConfig);
        shooterConfig =
                new FlyWheelConfig(talonSMC)
                        // Diameter of the flywheel.
                        .withDiameter(Inches.of(4))
                        // Mass of the flywheel.
                        .withMass(Pounds.of(3))
                        // Telemetry name and verbosity for the arm.
                        .withTelemetry("INTAKE_ROLLERS", TelemetryVerbosity.HIGH);

        shooter = new FlyWheel(shooterConfig);
    }

    @Override
    public void simIterate() {
        shooter.simIterate();
    }

    @Override
    public void updateTelemetry() {
        shooter.updateTelemetry();
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
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
     * Gets the current velocity of the rollers.
     *
     * @return Shooter velocity.
     */
    @Override
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    @Override
    public void setVoltage(double voltage) {
        shooter.set(voltage / 12);
    }

    /**
     * Set the rollers velocity setpoint.
     *
     * @param speed Speed to set
     */
    @Override
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }
}
