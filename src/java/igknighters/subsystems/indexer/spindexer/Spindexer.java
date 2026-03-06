package igknighters.subsystems.indexer.spindexer;

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

public class Spindexer extends SubsystemBase {
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

    private TalonFX spindexerMotor = new TalonFX(kSpindexer.LEADER_MOTOR_ID, kIndexer.CANBUS);

    private SmartMotorController spindexerController =
            new TalonFXWrapper(spindexerMotor, DCMotor.getKrakenX60(1), config);

    private final FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig(spindexerController)
                    .withDiameter(Inches.of(4.0))
                    .withUpperSoftLimit(RPM.of(5000))
                    .withMass(Pound.of(.5))
                    .withTelemetry("SpindexerMech", TelemetryVerbosity.HIGH);

    private FlyWheel spindexer = new FlyWheel(flyWheelConfig);

    public AngularVelocity getSpeed() {
        return spindexer.getSpeed();
    }

    public Command holdSpeed(AngularVelocity speed) {
        return spindexer.run(speed);
    }

    public void stop() {
        spindexer.set(0);
    }

    public Command holdAtState(IndexerState state) {
        return spindexer.run(RPM.of(state.spindexerRPM));
    }

    public Command jorkOnce() {
        return spindexer
                .runTo(RPM.of(IndexerState.JORK_FORWARD.spindexerRPM), RPM.of(10))
                .andThen(
                        spindexer.runTo(
                                RPM.of(IndexerState.JORK_BACKWARD.spindexerRPM), RPM.of(10)));
    }

    public Command jorkRepeating() {
        return jorkOnce().repeatedly();
    }
}
