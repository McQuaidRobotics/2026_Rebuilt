package igknighters.subsystems.indexer.launcherRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants.kIndexer;
import igknighters.constants.SubsystemConstants.kIndexer.kSpindexer;
import igknighters.subsystems.indexer.IndexerState;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ExitRollers extends SubsystemBase {
    private SmartMotorControllerConfig config =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            kIndexer.kSpindexer.kP, kIndexer.kSpindexer.kI, kIndexer.kSpindexer.kD)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    kIndexer.kSpindexer.kS,
                                    kIndexer.kSpindexer.kV,
                                    kIndexer.kSpindexer.kA))
                    .withTelemetry("Spindexer", TelemetryVerbosity.HIGH)
                    .withGearing(1.0)
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(kSpindexer.STATOR_CURRENT_LIMIT));

    private TalonFX exitRollerMotor = new TalonFX(kSpindexer.LEADER_MOTOR_ID, kIndexer.CANBUS);

    private SmartMotorController exitRollerController =
            new TalonFXWrapper(exitRollerMotor, DCMotor.getKrakenX60(1), config);

    private final FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig(exitRollerController)
                    .withDiameter(Inches.of(4.0))
                    .withUpperSoftLimit(RPM.of(5000))
                    .withMass(Pound.of(.5))
                    .withTelemetry("SpindexerMech", TelemetryVerbosity.HIGH);

    private FlyWheel exitRoller = new FlyWheel(flyWheelConfig);

    public AngularVelocity getSpeed() {
        return exitRoller.getSpeed();
    }

    public Command holdSpeed(AngularVelocity speed) {
        return exitRoller.run(speed);
    }

    public void stop() {
        exitRoller.set(0);
    }

    public Command holdAtState(IndexerState state) {
        return exitRoller.run(RPM.of(state.exitRollerRPM));
    }
}
