package igknighters.subsystems.YamShooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class HoodIOTalonFX implements HoodIO {
    private DigitalInput dio =
            new DigitalInput(Robot.consts.shooter().kHood().REVERSE_LIMIT_SWITCH_ID());

    boolean hasZeroed = false;

    private final SmartMotorControllerConfig hoodConfig;

    private final TalonFX hoodMotor =
            new TalonFX(
                    Robot.consts.shooter().kHood().MOTOR_ID(), Robot.consts.shooter().kCANBUS());

    private final Sensor limitSwitch;
    private final SmartMotorController hoodController;
    private PivotConfig pivotConfig;

    private Pivot hood;

    public HoodIOTalonFX(SubsystemBase subsystem) {
        hoodConfig = Robot.consts.shooter().kHood().getConfig(subsystem);
        hoodController = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodConfig);
        pivotConfig =
                new PivotConfig(hoodController)
                        // Soft limit is applied to the SmartMotorControllers PID
                        // Hard limit is applied to the simulation.
                        .withHardLimits(
                                Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES() - 5),
                                Degrees.of(Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES() + 5))
                        // Starting position is where your arm starts
                        .withSimStartingPosition(
                                Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES() + 5))
                        // Telemetry name and verbosity for the arm.
                        .withTelemetry("Shooter Hood", TelemetryVerbosity.HIGH);

        hood = new Pivot(pivotConfig);
        limitSwitch =
                new SensorConfig("Hood Limit Switch") // Name of the sensor
                        .withField(
                                "Switch", dio::get,
                                false) // Add a Field to the sensor named "Beam" whose value is
                        // default false
                        .withSimulatedValue(
                                "Switch",
                                hood.isNear(
                                        Rotations.of(
                                                Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()),
                                        Rotations.of(.01)),
                                true) // Change "Beam" field to true when the arm is near 40deg +-
                        // 2deg
                        .getSensor(); // Get the sensor.
    }

    // EVERYTHING WILL BE DONE IN ROTATIONS
    // THE SMC IS SET UP TO THE PID DRIVING TO ROTATIONS OF HOOD OUTPUT EG MECHANSIM FRAME
    // ALL THESE LIMITS SHOULD BE STRAIGHT CONSTANTS FOR THE HOOD

    /**
     * Set the angle of the arm, does not stop when the arm reaches the setpoint.
     *
     * @param angle Angle to go to.
     * @return A command.
     */
    public Command targetAngle(Angle angle) {
        // input is something like 30
        // all ready in the hood mechanism frame so just pass
        return hood.run(angle);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = hoodController.getMechanismPosition().in(Rotations);
        inputs.velocityRotationsPerSec =
                hoodController.getMechanismVelocity().in(RotationsPerSecond);
        inputs.appliedVolts = hoodController.getVoltage().in(Volts);
        inputs.supplyCurrentAmps =
                hoodController.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = hoodController.getStatorCurrent().in(Amps);
        inputs.temperatureCelsius = hoodController.getTemperature().in(Celsius);
        inputs.targetPositionRotations =
                hoodController.getMechanismPositionSetpoint().map(a -> a.in(Rotations)).orElse(0.0);
        inputs.limitSwitchTripped = limitSwitch.getAsBoolean("Switch");
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    @Override
    public void setTargetAngle(Angle angle) {

        hood.setMechanismPositionSetpoint(angle);
    }

    @Override
    public void zeroAt(Angle angle) {
        // THE SMC ZEROS STUFF IN TERMS OF MECHANISM POSITION
        // SEE TALONFX WRAPPER
        // m_talonfx.setPosition(angle);
        // it just calls this as well as doing anything with the encoder
        // .setPosition() zeros in terms of mechansim rots
        hoodController.setEncoderPosition(angle);
    }

    @Override
    public Angle getHoodAngle() {
        // hood in rots
        return hood.getAngle();
    }

    public boolean isAt(Angle angle, Angle tolerance) {
        // in mechanism frame already bc gear ratio 24
        return hood.isNear(angle, tolerance).getAsBoolean();
    }

    @Override
    public boolean isLimitSwitchTripped() {
        return limitSwitch.getAsBoolean("Switch");
    }

    @Override
    public void setVoltage(double voltage) {
        hood.set(voltage / 12.0);
    }

    @Override
    public void zeroHoodCheck() {

        if (limitSwitch.getAsBoolean("Switch") && !hasZeroed) {
            hasZeroed = true;
            zeroAt(Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
        } else if (!limitSwitch.getAsBoolean("Switch")) {
            hasZeroed =
                    false; // to ensure no spaming config aplies. Only becomes false once off sensor
        }
    }

    @Override
    public void simIterate() {
        hood.simIterate();
    }

    @Override
    public void updateTelemetry() {
        hood.updateTelemetry();
    }
}
