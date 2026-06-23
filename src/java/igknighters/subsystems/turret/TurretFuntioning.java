package igknighters.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import igknighters.constants.SubsystemConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretFuntioning extends Turret {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0)
  .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0)
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(16.2)
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withSoftLimits(Degrees.of(-270),Degrees.of(90))
  .withMomentOfInertia(Meters.of(1),Pounds.of(.15))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withStartingPosition(Degrees.of(0));

    private TalonFX talon = new TalonFX(4, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController = new TalonFXWrapper(talon,DCMotor.getFalcon500(1), smcConfig);
  
  private final PivotConfig shooterConfig = new PivotConfig(talonSmartMotorController)
	  // Soft limit is applied to the SmartMotorControllers PID
      
  .withHardLimits(Degrees.of(-280), Degrees.of(100))
  .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  private Pivot shooter = new Pivot(shooterConfig);

  @Override
  public void targetAngle(Angle angle) {
      // shooter.setMeasurementPositionSetpoint(); OG
      shooter.setMechanismPositionSetpoint(angle); // fix
  }
}

