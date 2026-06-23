// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Rollers extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig = Robot.consts.intake().kRollers().getConfig(this);

    // Vendor motor controller object
    private TalonFX roller = new TalonFX(Robot.consts.intake().kRollers().LEADER_MOTOR_ID());

    // Create our SmartMotorController from our TalonFX and config with the Kraken X60.
    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(roller, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig rollerConfig =
            new FlyWheelConfig(talonSmartMotorController)
                    // Diameter of the flywheel.
                    .withDiameter(Inches.of(4))
                    // Mass of the flywheel.
                    .withMass(Pounds.of(.15))
                    // Maximum speed of the shooter.
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
