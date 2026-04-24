// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)

                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(
                            Robot.consts.intake().kPivot().kP(),
                            Robot.consts.intake().kPivot().kI(),
                            Robot.consts.intake().kPivot().kD(),
                            RotationsPerSecond.of(
                                    Robot.consts.intake().kPivot().MAX_SPEED_ROTATIONS_PER_SECOND()),
                            RotationsPerSecondPerSecond.of(
                                    Robot.consts
                                            .intake()
                                            .kPivot()
                                            .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()))
                    .withSimClosedLoopController(
                            10,
                            Robot.consts.intake().kPivot().kI(),
                            Robot.consts.intake().kPivot().kD(),
                            DegreesPerSecond.of(360),
                            DegreesPerSecondPerSecond.of(480))
                    // Feedforward Constants
                    .withFeedforward(
                            new ArmFeedforward(
                                    Robot.consts.intake().kPivot().kS(),
                                    0,
                                    Robot.consts.intake().kPivot().kV(),
                                    Robot.consts.intake().kPivot().kA()))
                    .withSimFeedforward(
                            new ArmFeedforward(
                                    Robot.consts.intake().kPivot().kS(),
                                    0,
                                    Robot.consts.intake().kPivot().kV(),
                                    Robot.consts.intake().kPivot().kA()))
                    // Telemetry name and verbosity level
                    .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(15)))
                    .withMotorInverted(
                            Robot.consts
                                    .intake()
                                    .kPivot()
                                    .INVERTED()
                                    .equals(InvertedValue.Clockwise_Positive))
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(
                            Amps.of(Robot.consts.intake().kPivot().STATOR_CURRENT_LIMIT()))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    private TalonFX pivotMotor = new TalonFX(Robot.consts.intake().kPivot().MOTOR_ID());

    // Create our SmartMotorController from our Spark and config with the Kraken.
    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), smcConfig);

    private PivotConfig pivotConfig =
            new PivotConfig(talonSmartMotorController)
                    // Soft limit is applied to the SmartMotorControllers PID
                    .withMOI(Meters.of(0.25), Pounds.of(1))
                    .withSoftLimits(
                            Degrees.of(Robot.consts.intake().kPivot().MIN_ANGLE_DEGREES()),
                            Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()))
                    // Hard limit is applied to the simulation.
                    .withHardLimit(
                            Degrees.of(Robot.consts.intake().kPivot().MIN_ANGLE_DEGREES()),
                            Degrees.of(Robot.consts.intake().kPivot().MAX_ANGLE_DEGREES()))
                    // Starting position is where your arm starts
                    .withStartingPosition(
                            Degrees.of(Robot.consts.intake().kPivot().STOWED_ANGLE_DEGREES()))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Pivot pivot = new Pivot(pivotConfig);

    /**
     * Set the angle of the arm, does not stop when the arm reaches the setpoint.
     *
     * @param angle Angle to go to.
     * @return A command.
     */
    public Command targetAngle(Angle angle) {
        return pivot.run(angle);
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
        return pivot.runTo(angle, tolerance);
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     *
     * @param angle Angle to go to.
     */
    public void setAngleSetpoint(Angle angle) {
        pivot.setMechanismPositionSetpoint(angle);
    }

    /**
     * Move the arm up and down.
     *
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return pivot.set(dutycycle);
    }

    /** Run sysId on the {@link Arm} */
    public Command sysId() {
        return pivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    public Angle getAngle() {
        return pivot.getAngle();
    }

    public boolean isAt(Angle angle, Angle tolerance) {
        return pivot.isNear(angle, tolerance).getAsBoolean();
    }

    /** Creates a new ExampleSubsystem. */
    public IntakePivot() {}

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
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        pivot.simIterate();
    }
}
