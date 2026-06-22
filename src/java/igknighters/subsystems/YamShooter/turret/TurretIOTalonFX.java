package igknighters.subsystems.YamShooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretIOTalonFX implements TurretIO {
    private final CANcoder turretCancoder =
            new CANcoder(
                    Robot.consts.shooter().kTurret().CANCODER_ID(),
                    Robot.consts.shooter().kCANBUS());

    private final SmartMotorControllerConfig turretConfig;

    private final TalonFX turretMotor =
            new TalonFX(
                    Robot.consts.shooter().kTurret().MOTOR_ID(), Robot.consts.shooter().kCANBUS());

    private final SmartMotorController turretController;

    private final PivotConfig pivotConfig;

    private final Pivot turret;

    public TurretIOTalonFX(SubsystemBase subsystem) {

        turretConfig = Robot.consts.shooter().kTurret().getConfig(subsystem, turretCancoder);
        turretController = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), turretConfig);
        pivotConfig =
                new PivotConfig(turretController)
                        .withSimStartingPosition(Degrees.of(0.0))
                        .withHardLimits(
                                Degrees.of(
                                        Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES() - 10),
                                Degrees.of(
                                        Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES() + 10))
                        .withTelemetry("Turret Motor", TelemetryVerbosity.HIGH); // Telemetry;
        turret = new Pivot(pivotConfig);
    }

    @Override
    public Angle wrapAngle(Angle angle) {
        double ogDegrees = angle.in(Degrees);
        double maxDegrees = Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES();
        double MIN_ANGLE_DEGREES = Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES();

        double width = maxDegrees - MIN_ANGLE_DEGREES;

        double newDegs =
                MIN_ANGLE_DEGREES + (((ogDegrees - MIN_ANGLE_DEGREES) % width + width) % width);

        return Degrees.of(newDegs);
    }

    // LOG ALL OF THE MOTORS OUTPUTS FOR REPLAY (MASON/KYLE IF YOU ARE READING THIS FOR A EXAMPLE
    // DONT DO THIS YET)
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionRotations = turretController.getMechanismPosition().in(Rotations);
        inputs.velocityRotationsPerSec =
                turretController.getMechanismVelocity().in(RotationsPerSecond);
        inputs.appliedVolts = turretController.getVoltage().in(Volts);
        inputs.supplyCurrentAmps =
                turretController.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = turretController.getStatorCurrent().in(Amps);
        inputs.temperatureCelsius = turretController.getTemperature().in(Celsius);
        inputs.targetPositionRotations =
                turretController
                        .getMechanismPositionSetpoint()
                        .map(a -> a.in(Rotations))
                        .orElse(0.0);
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    @Override
    public void setAngleSetpoint(Angle angle) {
        turret.setMechanismPositionSetpoint(angle);
    }

    /**
     * @return The current angle of the turret.
     */
    @Override
    public Angle getAngle() {
        return turret.getAngle();
    }

    @Override
    public void updateTelemetry() {
        // This method will be called once per scheduler run
        turret.updateTelemetry();
    }

    @Override
    public void simIterate() {
        // This method will be called once per scheduler run during simulation
        turret.simIterate();
    }
}
