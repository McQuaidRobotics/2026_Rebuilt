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
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretFunctioning extends Turret implements TurretIO{
    private final CANcoder turretCancoder =
            new CANcoder(
                    Robot.consts.shooter().kTurret().CANCODER_ID(),
                    Robot.consts.shooter().kCANBUS());

    private final SmartMotorControllerConfig turretConfig =
            Robot.consts.shooter().kTurret().getConfig(this, turretCancoder);

    private final TalonFX turretMotor =
            new TalonFX(
                    Robot.consts.shooter().kTurret().MOTOR_ID(), Robot.consts.shooter().kCANBUS());

    private final SmartMotorController turretController =
            new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), turretConfig);

    private final PivotConfig pivotConfig =
            new PivotConfig(turretController)
                    .withSimStartingPosition(Degrees.of(0.0))
                    .withHardLimits(
                            Degrees.of(Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES() - 10),
                            Degrees.of(Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES() + 10))
                    .withTelemetry("Turret Motor", TelemetryVerbosity.HIGH); // Telemetry;

    private final Pivot turret = new Pivot(pivotConfig);
   
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
    // LOG ALL OF THE MOTORS OUTPUTS FOR REPLAY (MASON/KYLE IF YOU ARE READING THIS FOR A EXAMPLE DONT DO THIS YET)
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionRotations = turretController.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = turretController.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = turretController.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = turretController.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = turretController.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = turretController.getTemperature().in(Celsius);
    inputs.targetPositionRotations = turretController.getMechanismPositionSetpoint()
        .map(a -> a.in(Rotations)).orElse(0.0);
    }

    /**
     * Set the angle of the arm, does not stop when the arm reaches the setpoint.
     *
     * @param angle Angle to go to.
     * @return A command.
     */
    public Command targetAngle(Angle angle) {
        return turret.run(angle);
    }

    /**
     * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
     * setpoint.
     *
     * @param angle Angle to go to.
     * @param tolerance Angle tolerance for completion.
     * @return A Command
     */
    public Command setAngleAndStop(Angle angle, Angle tolerance) {
        return turret.runTo(angle, tolerance);
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    public void setAngleSetpoint(Angle angle) {
        turret.setMechanismPositionSetpoint(angle);
    }

    /**
     * Move the arm up and down.
     *
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return turret.set(dutycycle);
    }

    // /** Run sysId on the {@link Arm} */
    // public Command sysId() {
    //     return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    // }

    public Angle getAngle() {
        return turret.getAngle();
    }

    public boolean isAt(Angle angle, Angle tolerance) {
        return turret.isNear(angle, tolerance).getAsBoolean();
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        turret.simIterate();
    }
}
