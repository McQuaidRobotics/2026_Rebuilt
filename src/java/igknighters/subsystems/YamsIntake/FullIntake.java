package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FullIntake extends SubsystemBase {
    private SmartMotorControllerConfig pivotConfig = new SmartMotorControllerConfig(this)
	  .withControlMode(ControlMode.CLOSED_LOOP)
	  // Feedback Constants (PID Constants)
	  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
	  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
	  // Feedforward Constants
	  .withFeedforward(new ArmFeedforward(0, 0, 0))
	  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
	  // Telemetry name and verbosity level
	  .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
	  // Gearing from the motor rotor to final shaft.
	  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
	  .withGearing(new MechanismGearing(GearBox.fromReductionStages(Robot.consts.intake().kPivot().GEAR_RATIO())))
	  .withMotorInverted(Robot.consts.intake().kPivot().INVERTED().equals(InvertedValue.Clockwise_Positive))
	  .withIdleMode(MotorMode.BRAKE)
	  .withStatorCurrentLimit(Amps.of(40))
	  .withClosedLoopRampRate(Seconds.of(0.25))
	  .withOpenLoopRampRate(Seconds.of(0.25));
	
	  // Vendor motor controller object
	  private TalonFX pivotMotor = new TalonFX(4);
	
	  // Create our SmartMotorController from our Spark and config with the NEO.
	  private SmartMotorController pivotSMC = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotConfig);
	
	  private ArmConfig armCfg = new ArmConfig(pivotSMC)
	  // Soft limit is applied to the SmartMotorControllers PID
	  .withSoftLimits(Degrees.of(Robot.consts.intake().kPivot().MIN_ANGLE_DEGREES()), Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()))
	  // Hard limit is applied to the simulation.
	  .withHardLimit(Degrees.of(Robot.consts.intake().kPivot().MIN_ANGLE_DEGREES()), Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()))
	  // Starting position is where your arm starts
	  .withStartingPosition(Degrees.of(0))
	  // Length and mass of your arm for sim.
	  .withLength(Feet.of(2))
	  .withMass(Pounds.of(1))
	  // Telemetry name and verbosity for the arm.
	  .withTelemetry("Arm", TelemetryVerbosity.HIGH);
	
	  // Arm Mechanism
	  private Arm arm = new Arm(armCfg);

    private SmartMotorControllerConfig rollersSMCConfig = new SmartMotorControllerConfig(this)
	  .withControlMode(ControlMode.CLOSED_LOOP)
	  // Feedback Constants (PID Constants)
	  .withClosedLoopController(1, 0, 0)
	  .withFollowers(Pair.of(new TalonFX(Robot.consts.intake().kRollers().FOLLOWER_MOTOR_ID()), Robot.consts.intake().kRollers().DRIVE_RATIO() < 0))
	  .withSimClosedLoopController(1, 0, 0)
	  // Feedforward Constants
	  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
	  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
	  // Telemetry name and verbosity level
	  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
	  .withGearing(new MechanismGearing(GearBox.fromReductionStages(Robot.consts.intake().kRollers().GEAR_RATIO())))
	  // Motor properties to prevent over currenting.
	  .withMotorInverted(false)
	  .withIdleMode(MotorMode.COAST)
	  .withStatorCurrentLimit(Amps.of(40));
	
	  // Vendor motor controller object
	  private TalonFX roller = new TalonFX(Robot.consts.intake().kRollers().LEADER_MOTOR_ID());
	
	  // Create our SmartMotorController from our TalonFX and config with the Kraken X60.
	  private SmartMotorController rollerSmartMotorController = new TalonFXWrapper(roller, DCMotor.getKrakenX60(1), rollersSMCConfig);
	
	 private final FlyWheelConfig shooterConfig = new FlyWheelConfig(rollerSmartMotorController)
	  // Diameter of the flywheel.
	  .withDiameter(Inches.of(4))
	  // Mass of the flywheel.
	  .withMass(Pounds.of(1))
	  // Maximum speed of the shooter.
	  .withUpperSoftLimit(RPM.of(5000))
	  // Telemetry name and verbosity for the arm.
	  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
	
	  // Shooter Mechanism
	  private FlyWheel shooter = new FlyWheel(shooterConfig);


      public Command targetState(YamIntakeState state) { return Commands.parallel(
        arm.run(state.pivotAngle),
        shooter.run(state.rollerVelocity)
      );}

      public Command jorkIntake() { return 
        Commands.sequence(
            targetState(YamIntakeState.DEPLOYED).withTimeout(1),
            targetState(YamIntakeState.STOWED).withTimeout(1)
        ).repeatedly();
    }

    public AngularVelocity getRollerVelocity() { 
        return shooter.getSpeed();
    }
    public Angle getPivotAngle() { 
        return arm.getAngle();
    }

    public boolean isAt(YamIntakeState state, AngularVelocity velocityTolerance, Angle angleTolerance) {
        return Math.abs(this.getRollerVelocity().in(RPM) - state.rollerVelocity.in(RPM)) < velocityTolerance.in(RPM) &&
               Math.abs(this.getPivotAngle().in(Degrees) - state.pivotAngle.in(Degrees)) < angleTolerance.in(Degrees);
    }

}
