package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(kHood.kP, kHood.kI, kHood.kD)
                    .withSimClosedLoopController(5, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kHood.MAX_SPEED_R_P_S), RotationsPerSecondPerSecond.of(kHood.MAX_ACCEL_R_P_S_S))
                    // ----------------------------------------------------------------
                    .withSoftLimits(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    // ----------------------------------------------------------------
                    .withSimClosedLoopController(50, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0, 0, 0))
                    .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                    // Telemetry name and verbosity level
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(16.2)
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimits(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withMomentOfInertia(Meters.of(.1), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withExternalEncoderInverted(false)
                    .withExternalEncoderGearing(
                            new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withUseExternalFeedbackEncoder(false)
                    .withStartingPosition(Degrees.of(0));

    private TalonFX talon = new TalonFX(kHood.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getFalcon500(1), smcConfig);

    private final PivotConfig hoodConfig =
            new PivotConfig()
            .withHardLimits(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(kHood.MAX_ANGLE_DEGREES))
            .withTelemetry("PivotExample", TelemetryVerbosity.HIGH)
                    // Soft limit is applied to the SmartMotorControllers PID

                    .withHardLimits(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withTelemetry("SHOOTER_HOOD", TelemetryVerbosity.HIGH);
    private Pivot hood = new Pivot(hoodConfig, talonSmartMotorController);

    


    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public Command targetAngleCommand(Angle angle) {
        return hood.run(angle);
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        return hood.runTo(angle, Rotations.of(.05));
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
