package igknighters.subsystems.indexer.launcherRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class ExitRollersFunctioning extends ExitRollersBase {

    private SmartMotorControllerConfig smcConfig = Robot.consts.indexer().kExitRollers().getConfig(this);

    private TalonFX spindexerMotor;

    public ExitRollersFunctioning() {
            spindexerMotor = new TalonFX(Robot.consts.indexer().kExitRollers().LEADER_MOTOR_ID());
            talonSMC = new TalonFXWrapper(spindexerMotor, DCMotor.getKrakenX60(1), smcConfig);
            shooterConfig =
                    new FlyWheelConfig(talonSMC)
                            // Diameter of the flywheel.
                            .withDiameter(Inches.of(4))
                            // Mass of the flywheel.
                            .withMass(Pounds.of(1))
                            // Maximum speed of the shooter.
                            .withUpperSoftLimit(RPM.of(5000))
                            // Telemetry name and verbosity for the arm.
                            .withTelemetry("EXIT ROLLERS", TelemetryVerbosity.HIGH);
            shooter = new FlyWheel(shooterConfig);
    }

    private SmartMotorController talonSMC;

    private FlyWheelConfig shooterConfig;

    private FlyWheel shooter;

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
