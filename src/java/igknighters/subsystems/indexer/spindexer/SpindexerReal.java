package igknighters.subsystems.indexer.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import igknighters.Robot;
import igknighters.util.log.Log;

public class SpindexerReal extends Spindexer {
    private final TalonFX spindexer =
            new TalonFX(
                    Robot.consts.indexer().kSpindexer().LEADER_MOTOR_ID(),
                    Robot.consts.indexer().kCANBUS());

    private final MotionMagicVelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Robot.consts.indexer().kSpindexer().kP();
        config.Slot0.kI = Robot.consts.indexer().kSpindexer().kI();
        config.Slot0.kD = Robot.consts.indexer().kSpindexer().kD();
        config.Slot0.kS = Robot.consts.indexer().kSpindexer().kS();
        config.Slot0.kV = Robot.consts.indexer().kSpindexer().kV();

        config.Feedback.SensorToMechanismRatio = Robot.consts.indexer().kSpindexer().GEAR_RATIO();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotionMagic.MotionMagicJerk =
                Robot.consts.indexer().kSpindexer().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.indexer().kSpindexer().MAX_ACCELERATION_RPM();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.indexer().kSpindexer().MAX_SPEED_RPM();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                Robot.consts.indexer().kSpindexer().SUPPLY_CURRENT_LIMIT();
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                Robot.consts.indexer().kSpindexer().PEAK_CURRENT_LIMIT();

        return config;
    }

    public SpindexerReal() {

        spindexer.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);
    }

    @Override
    public void goToRPM(double RPM) {

        if (!Robot.consts.indexer().kSpindexer().disableSpindexerLogs()) {
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
        if (!Robot.consts.indexer().kSpindexer().disableSpindexerLogs()) {
            Log.log("Subsystems/Indexer/Spindexer/velocity", getRPM());
        }
    }
}
