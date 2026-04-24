// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Rollers extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(
                            Robot.consts.intake().kRollers().kP(),
                            Robot.consts.intake().kRollers().kI(),
                            Robot.consts.intake().kRollers().kD())
                    .withFollowers(
                            Pair.of(
                                    new TalonFX(
                                            Robot.consts.intake().kRollers().FOLLOWER_MOTOR_ID()),
                                    Robot.consts.intake().kRollers().DRIVE_RATIO() < 0))
                    .withSimClosedLoopController(
                            Robot.consts.intake().kRollers().kP(),
                            Robot.consts.intake().kRollers().kI(),
                            Robot.consts.intake().kRollers().kD())
                    // Feedforward Constants
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    Robot.consts.intake().kRollers().kS(),
                                    Robot.consts.intake().kRollers().kV(),
                                    Robot.consts.intake().kRollers().kA()))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    Robot.consts.intake().kRollers().kS(),
                                    Robot.consts.intake().kRollers().kV(),
                                    Robot.consts.intake().kRollers().kA()))
                    // Telemetry name and verbosity level
                    .withTelemetry("Rollers", TelemetryVerbosity.HIGH)
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromReductionStages(
                                            Robot.consts.intake().kRollers().GEAR_RATIO())))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(
                            Robot.consts
                                    .intake()
                                    .kRollers()
                                    .INVERTED()
                                    .equals(InvertedValue.Clockwise_Positive))
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(
                            Amps.of(Robot.consts.intake().kRollers().STATOR_CURRENT_LIMIT()));

    // Vendor motor controller object
    private TalonFX roller = new TalonFX(Robot.consts.intake().kRollers().LEADER_MOTOR_ID());

    // Create our SmartMotorController from our TalonFX and config with the Kraken X60.
    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(roller, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig rollerConfig =
            new FlyWheelConfig(talonSmartMotorController)
                    // Diameter of the flywheel.
                    .withDiameter(
                            edu.wpi.first.units.Units.Meters.of(
                                    Robot.consts.intake().kRollers().WHEEL_RADIUS_METERS() * 2))
                    // Mass of the flywheel.
                    .withMass(Pounds.of(1))
                    // Maximum speed of the shooter.
                    .withUpperSoftLimit(RPM.of(Robot.consts.intake().kRollers().MAX_SPEED_RPM()))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("IntakeRollers", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(rollerConfig);

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command targetVelocity(AngularVelocity speed) {
        return shooter.run(speed);
    }

    public boolean isAt(AngularVelocity speed, AngularVelocity tolerance) {
        return shooter.isNear(speed, tolerance).getAsBoolean();
    }

    /**
     * Set the shooter velocity setpoint.
     *
     * @param speed Speed to set
     */
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    /** Creates a new ExampleSubsystem. */
    public Rollers() {}

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
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }
}
