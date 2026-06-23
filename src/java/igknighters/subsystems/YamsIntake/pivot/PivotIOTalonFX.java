package igknighters.subsystems.YamsIntake.pivot;

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

public class PivotIOTalonFX implements PivotIO {
    private CANcoder pivotCancoder =
            new CANcoder(
                    Robot.consts.intake().kPivot().CANCODER_ID(), Robot.consts.intake().kCANBUS());
    private TalonFX pivotMotor =
            new TalonFX(Robot.consts.intake().kPivot().MOTOR_ID(), Robot.consts.intake().kCANBUS());
    private SmartMotorControllerConfig smcConfig;

    private final SmartMotorController talonSmartMotorController;

    private final PivotConfig pivotConfig;

    // Arm Mechanism
    private Pivot pivot;

    public PivotIOTalonFX(SubsystemBase subsystem) {

        smcConfig = Robot.consts.intake().kPivot().getConfig(pivotCancoder, subsystem);
        talonSmartMotorController =
                new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), smcConfig);
        pivotConfig =
                new PivotConfig(talonSmartMotorController)
                        .withHardLimits(
                                Degrees.of(Robot.consts.intake().kPivot().MIN_ANGLE_DEGREES() - 10),
                                Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES() + 10))
                        .withSimStartingPosition(
                                Degrees.of(Robot.consts.intake().kPivot().STOWED_ANGLE_DEGREES()))
                        .withTelemetry("INTAKE_PIVOT", TelemetryVerbosity.HIGH);
        pivot = new Pivot(pivotConfig);
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    @Override
    public void setAngleSetpoint(Angle angle) {
        pivot.setMechanismPositionSetpoint(angle);
    }

    @Override
    public Angle getAngle() {
        return pivot.getAngle();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRotations = talonSmartMotorController.getMechanismPosition().in(Rotations);
        inputs.velocityRotationsPerSec =
                talonSmartMotorController.getMechanismVelocity().in(RotationsPerSecond);
        inputs.appliedVolts = talonSmartMotorController.getVoltage().in(Volts);
        inputs.supplyCurrentAmps =
                talonSmartMotorController.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = talonSmartMotorController.getStatorCurrent().in(Amps);
        inputs.temperatureCelsius = talonSmartMotorController.getTemperature().in(Celsius);
        inputs.targetPositionRotations =
                talonSmartMotorController
                        .getMechanismPositionSetpoint()
                        .map(a -> a.in(Rotations))
                        .orElse(0.0);
    }

    @Override
    public void updateTelemetry() {
        pivot.updateTelemetry();
    }

    @Override
    public void simIterate() {
        pivot.simIterate();
    }
}
