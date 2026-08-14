package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.log.Log;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class Hood extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withStartingPosition(Degrees.of(kHood.MIN_ANGLE_DEGREES))
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(kHood.kP, kHood.kI, kHood.kD)
                    .withSimClosedLoopController(5, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kHood.MAX_SPEED_R_P_S),
                            RotationsPerSecondPerSecond.of(kHood.MAX_ACCEL_R_P_S_S))
                    // ----------------------------------------------------------------
                    .withSoftLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    // ----------------------------------------------------------------
                    .withSimClosedLoopController(5, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0, 0, 0))
                    .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                    // Telemetry name and verbosity level
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox
                    // attached to
                    // your motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(360 / 15) // 24:1 gearing
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withMomentOfInertia(Meters.of(.1), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));

    private DigitalInput dio = new DigitalInput(0); // Standard DIO

    private TalonFX talon = new TalonFX(kHood.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    MechanismPositionConfig hoodPosConfig =
            new MechanismPositionConfig()
                    .withRelativePosition(
                            new Translation3d(0.2, 0.0, 0.3)) // 20 cm forward, 30 cm up
                    .withMaxRobotLength(Meters.of(0.85))
                    .withMaxRobotHeight(Meters.of(1.20))
                    .withMovementPlane(MechanismPositionConfig.Plane.XZ);

    private final PivotConfig hoodConfig =
            new PivotConfig()
                    .withHardLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withMechanismPositionConfig(hoodPosConfig)
                    .withTelemetry("PivotExample", TelemetryVerbosity.HIGH)
                    // Soft limit is applied to the SmartMotorControllers PID
                    .withHardLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withTelemetry("SHOOTER_HOOD", TelemetryVerbosity.HIGH);
    private Pivot hood = new Pivot(hoodConfig, talonSmartMotorController);

    private final Sensor hoodLimit =
            new SensorConfig("hoodLimit") // Name of the sensor
                    .withField(
                            "Limit", dio::get,
                            false) // Add a Field to the sensor named "Beam" whose value is
                    // dio.get() and defaults to false
                    .withSimulatedValue(
                            "hoodLimit",
                            Seconds.of(3),
                            Seconds.of(4),
                            true) // Change the "Beam" field to true between 3s and 4s into a match
                    .withSimulatedValue(
                            "hoodLimit",
                            hood.isNear(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(2)),
                            true) // Change "Beam" field to true when the arm is near 40deg +- 2deg
                    .getSensor(); // Get the sensor.

    public boolean getHoodLimit() {
        return hoodLimit.getAsBoolean("Limit");
    }

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public Command targetAngleCommand(Angle angle) {
        return this.run(() -> hood.setMechanismPositionSetpoint(angle));
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        return hood.runTo(angle, Rotations.of(.05));
    }

    public boolean isZeroed = false;

    
    public static Command homeHood() {
        // return Commands.run(() -> shooter.setHoodVoltage(-1)).until(()
        // ->shooter.isHoodSensorHit());
        if (isZeroed == true) {
            pass;
        }
        return shooter.hood
                .run(() -> shooter.hood.setVoltage(-1))
                .until(() -> shooter.isHoodSensorTripped())
                .withTimeout(3.0)
                .withName("DRIVE DOWN HAS NOT HIT THE SENSOR YET HOME HOOD") // failsafe, can be
                // deleted if needed,
                // might be conflicting
                // with the
                // below code, needs testing on robot otherwise.
                .andThen(
                        Commands.runOnce(
                                () -> {
                                    shooter.hood.setVoltage(0);
                                    shooter.hood.zeroAt(
                                            Degrees.of(
                                                    Robot.consts
                                                            .shooter()
                                                            .kHood()
                                                            .MIN_ANGLE_DEGREES()));
                                }))
                .withName("HOOD IS DOWN ON SENSOR");
    }

    

    public Command zeroHood() {
        return hood.set(-.3).until(() -> getHoodLimit());
    }

    public void targetAngleNoCommand(Angle angle) {
        Log.log("ROBOT/Subsystems/Shooter/Hood/Target_Position", angle.in(Degrees));
        hood.setMechanismPositionSetpoint(angle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
        if (getHoodLimit()==false) {
            isZeroed = false;
        }


        if (getHoodLimit()==true && isZeroed==false) {
            talonSmartMotorController.setPosition(Degrees.of(kHood.MIN_ANGLE_DEGREES));
            isZeroed = true;
        }

        
        
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
