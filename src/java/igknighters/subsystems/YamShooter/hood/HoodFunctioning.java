package igknighters.subsystems.YamShooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class HoodFunctioning extends Hood {
    private DigitalInput dio =
            new DigitalInput(Robot.consts.shooter().kHood().REVERSE_LIMIT_SWITCH_ID());

    boolean hasZeroed = false;

    private final SmartMotorControllerConfig hoodConfig =
            Robot.consts.shooter().kHood().getConfig(this);

    private final TalonFX hoodMotor =
            new TalonFX(
                    Robot.consts.shooter().kHood().MOTOR_ID(), Robot.consts.shooter().kCANBUS());

    private final SmartMotorController hoodController =
            new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodConfig);
    // EVERYTHING WILL BE DONE IN ROTATIONS
    // THE SMC IS SET UP TO THE PID DRIVING TO ROTATIONS OF HOOD OUTPUT EG MECHANSIM FRAME
    // ALL THESE LIMITS SHOULD BE STRAIGHT CONSTANTS FOR THE HOOD
    private PivotConfig pivotConfig =
            new PivotConfig(hoodController)
                    // Soft limit is applied to the SmartMotorControllers PID
                    .withMOI(Meters.of(0.25), Pounds.of(.5))
                    .withSoftLimits(
                            Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()),
                            Degrees.of(Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES()))
                    // Hard limit is applied to the simulation.
                    .withHardLimit(
                            Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES() - 5),
                            Degrees.of(Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES() + 5))
                    // Starting position is where your arm starts
                    .withStartingPosition(
                            Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES() + 5))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("Shooter Hood", TelemetryVerbosity.HIGH);

    private Pivot hood = new Pivot(pivotConfig);

    private final Sensor limitSwitch =
            new SensorConfig("Hood Limit Switch") // Name of the sensor
                    .withField(
                            "Switch", dio::get,
                            false) // Add a Field to the sensor named "Beam" whose value is
                    .withSimulatedValue(
                            "Switch",
                            hood.isNear(
                                    Rotations.of(
                                            Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()),
                                    Rotations.of(.01)),
                            true) // Change "Beam" field to true when the arm is near 40deg +- 2deg
                    .getSensor(); // Get the sensor.

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

    /**
     * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
     * setpoint.
     *
     * @param angle Angle to go to.
     * @param tolerance Angle tolerance for completion.
     * @return A Command
     */
    public Command setAngleAndStop(Angle angle, Angle tolerance) {
        return hood.runTo(angle, tolerance); // force to rotations
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    public void setAngleSetpoint(Angle angle) {

        // Angle goalAngle =
        //         Rotations.of(
        //                 angle.in(Degrees)
        //                         / Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES());
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

    /**
     * Move the arm up and down.
     *
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return hood.set(dutycycle);
    }

    /** Run sysId on the {@link Arm} */
    public Command sysId() {
        return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    public Angle getAngle() {
        // hood in rots
        return Degrees.of(
                hood.getAngle().in(Rotations)
                        * Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES());
    }

    public boolean isAt(Angle angle, Angle tolerance) {
        // in mechanism frame already bc gear ratio 24
        return hood.isNear(angle, tolerance).getAsBoolean();
    }

    @Override
    public boolean isLimitSwitchTripped() {
        return limitSwitch.getAsBoolean("Switch");
    }

    /** Creates a new ExampleSubsystem. */
    public HoodFunctioning() {}

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
    public void setVoltage(double voltage) {
        hood.set(voltage / 12.0);
    }

    @Override
    public void periodic() {

        if (limitSwitch.getAsBoolean("Switch") && !hasZeroed) {
            hasZeroed = true;
            zeroAt(Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
        } else if (!limitSwitch.getAsBoolean("Switch")) {
            hasZeroed =
                    false; // to ensure no spaming config aplies. Only becomes false once off sensor
        }
        // This method will be called once per scheduler run
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        hood.simIterate();
    }
}
