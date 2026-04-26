// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters.subsystems.YamsIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends SubsystemBase {
    private CANcoder pivotCancoder =
            new CANcoder(
                    Robot.consts.intake().kPivot().CANCODER_ID(), Robot.consts.intake().kCANBUS());
    private SmartMotorControllerConfig smcConfig =
            Robot.consts.intake().kPivot().getConfig(pivotCancoder, this);
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
    public IntakePivot() {
        smcConfig.withSubsystem(this);
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
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        pivot.simIterate();
    }
}
