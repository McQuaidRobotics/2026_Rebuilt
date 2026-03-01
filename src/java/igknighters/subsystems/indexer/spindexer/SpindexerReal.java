package igknighters.subsystems.indexer.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import igknighters.util.log.Log;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer;

public class SpindexerReal extends Spindexer {
    private final TalonFX spindexer =
            new TalonFX(SubsystemConstants.kIndexer.kSpindexer.LEADER_MOTOR_ID, kIndexer.CANBUS);

    // private final MotionMagicVelocityVoltage velocityControl = new
    // MotionMagicVelocityVoltage(0.0);

    private final MotionMagicVelocityVoltage velocityControl;
    // private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
    //         new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // private final DigitalInput beamBreakSensor = new
    // DigitalInput(SubsystemConstants.Shooter.BEAM_BREAK_SENSOR_CHANNEL);

    private BaseStatusSignal spindexerVelocity;
    private BaseStatusSignal spindexerCurrent;
    private BaseStatusSignal spindexerVoltage;
    private BaseStatusSignal spindexerTemperature;

    // private BaseStatusSignal isBeamBreakTripped;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.kIndexer.kSpindexer.kP;
        config.Slot0.kI = SubsystemConstants.kIndexer.kSpindexer.kI;
        config.Slot0.kD = SubsystemConstants.kIndexer.kSpindexer.kD;
        config.Slot0.kS = SubsystemConstants.kIndexer.kSpindexer.kS;
        config.Slot0.kV = SubsystemConstants.kIndexer.kSpindexer.kV;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kIndexer.kSpindexer.GEAR_RATIO;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotionMagic.MotionMagicJerk =
                SubsystemConstants.kIndexer.kSpindexer.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kIndexer.kSpindexer.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kIndexer.kSpindexer.MAX_SPEED_RPM;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kIndexer.kSpindexer.SUPPLY_CURRENT_LIMIT;
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                SubsystemConstants.kIndexer.kSpindexer.PEAK_CURRENT_LIMIT;

        return config;
    }

    public SpindexerReal() {

        spindexer.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        spindexerVelocity = spindexer.getVelocity();
        spindexerCurrent = spindexer.getSupplyCurrent();
        spindexerVoltage = spindexer.getSupplyVoltage();
        spindexerTemperature = spindexer.getDeviceTemp();
    }

    @Override
    public void goToRPM(double RPM) {
        Log.log("Subsystems/Indexer/Spindexer/setSpeed", RPM);
        spindexer.setControl(velocityControl.withVelocity(RPM / 60.0));
        spindexer.setControl(velocityControl.withVelocity(RPM / 60.0));
    }

    @Override
    public void stop() {
        spindexer.setControl(dutyCycleControl.withOutput(0.0));
    }

    @Override
    public double getRPM() {
        return spindexerVelocity.getValueAsDouble() * 60;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                spindexerVelocity, spindexerCurrent, spindexerVoltage, spindexerTemperature);
        Log.log(
                "Subsystems/Indexer/Spindexer/velocity",
                spindexerVelocity.getValueAsDouble() * 60.0);
        Log.log("Subsystems/Indexer/Spindexer/current", spindexerCurrent.getValueAsDouble());
        Log.log("Subsystems/Indexer/Spindexer/voltage", spindexerVoltage.getValueAsDouble());
        Log.log(
                "Subsystems/Indexer/Spindexer/temperature",
                spindexerTemperature.getValueAsDouble());
    }
}
