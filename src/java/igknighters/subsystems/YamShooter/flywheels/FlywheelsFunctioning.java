package igknighters.subsystems.YamShooter.flywheels;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelsFunctioning extends Flywheels {

    private TalonFX followerMotor = new TalonFX(Robot.consts.shooter().kFlywheels().FOLLOWER_MOTOR_ID(), Robot.consts.shooter().kCANBUS());
     private SmartMotorControllerConfig smcConfig =
            Robot.consts.shooter().kFlywheels().getConfig(this, followerMotor);

    private TalonFX leaderMotor =
            new TalonFX(
                    Robot.consts.shooter().kFlywheels().LEADER_MOTOR_ID(),
                    Robot.consts.shooter().kCANBUS());

    private SmartMotorController talonSMC =
            new TalonFXWrapper(leaderMotor, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig shooterConfig =
            new FlyWheelConfig(talonSMC)
                    // Diameter of the flywheel.
                    .withDiameter(Inches.of(4))
                    // Mass of the flywheel.
                    .withMass(Pounds.of(3))
                    // Maximum speed of the shooter.
                    .withUpperSoftLimit(RPM.of(5800))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("Flywheel Mechanism", TelemetryVerbosity.HIGH);

    private FlyWheel shooter = new FlyWheel(shooterConfig);

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
    public Command setVelocity(AngularVelocity speed) {
        return shooter.run(speed);
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
}
