package igknighters.subsystems.indexer.launcherRollers;

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

public class ExitRollersIOTalonFx implements ExitRollersIO {

    private SmartMotorControllerConfig smcConfig;

    private TalonFX spindexerMotor;

    private SmartMotorController talonSMC;

    private FlyWheelConfig shooterConfig;

    private FlyWheel shooter;

    public ExitRollersIOTalonFx(SubsystemBase subsystem) {

        smcConfig = Robot.consts.indexer().kExitRollers().getConfig(subsystem);
        spindexerMotor =
                new TalonFX(
                        Robot.consts.indexer().kExitRollers().LEADER_MOTOR_ID(),
                        Robot.consts.indexer().kCANBUS());

        talonSMC = new TalonFXWrapper(spindexerMotor, DCMotor.getKrakenX44(1), smcConfig);
        shooterConfig =
                new FlyWheelConfig(talonSMC)
                        // Diameter of the flywheel.
                        .withDiameter(Inches.of(4))
                        // Mass of the flywheel.
                        .withMass(Pounds.of(1))
                        // Maximum speed of the shooter.
                        // Telemetry name and verbosity for the arm.
                        .withTelemetry("EXIT ROLLERS", TelemetryVerbosity.HIGH);
        shooter = new FlyWheel(shooterConfig);
    }

    @Override
    public void updateInputs(ExitRollersIOInputs inputs) {
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

    @Override
    public void updateTelemetry() {
        // This method will be called once per scheduler run
        shooter.updateTelemetry();
    }

    @Override
    public void setVoltage(double voltage) {
        talonSMC.setVoltage(Volts.of(voltage));
    }

    @Override
    public void simIterate() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    @Override
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Set the shooter velocity setpoint.
     *
     * @param speed Speed to set
     */
    @Override
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }
}
