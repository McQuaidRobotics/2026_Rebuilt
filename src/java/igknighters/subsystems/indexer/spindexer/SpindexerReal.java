package igknighters.subsystems.indexer.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer;
import igknighters.util.log.Log;

public class SpindexerReal extends Spindexer {
    private final TalonFX spindexer =
            new TalonFX(SubsystemConstants.kIndexer.kSpindexer.LEADER_MOTOR_ID, kIndexer.CANBUS);

    private final MotionMagicVelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

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
    }

    @Override
    public void goToRPM(double RPM) {

        if (!SubsystemConstants.kIndexer.kSpindexer.disableSpindexerLogs) {
            Log.log("ROBOT/Subsystems/Indexer/Spindexer/setSpeed", RPM);
        }
        spindexer.setControl(velocityControl.withVelocity(RPM / 60.0));
        spindexer.setControl(velocityControl.withVelocity(RPM / 60.0));
    }

    @Override
    public void stop() {
        spindexer.setControl(dutyCycleControl.withOutput(0.0));
    }

    @Override
    public double getRPM() {
        return spindexer.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void periodic() {
        if (!SubsystemConstants.kIndexer.kSpindexer.disableSpindexerLogs) {
            Log.log("Subsystems/Indexer/Spindexer/velocity", getRPM());
        }
    }
}
