package igknighters.subsystems.indexer.launcherRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.util.log.Log;

public class ExitRollersReal extends ExitRollers {

    private final TalonFX exitRollerMotor;
    private final MotionMagicVelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private BaseStatusSignal shooterVelocity;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = Robot.consts.indexer().kExitRollers().kP();
        config.Slot0.kI = Robot.consts.indexer().kExitRollers().kI();
        config.Slot0.kD = Robot.consts.indexer().kExitRollers().kD();
        config.Slot0.kS = Robot.consts.indexer().kExitRollers().kS();
        config.Slot0.kV = Robot.consts.indexer().kExitRollers().kV();
        config.Feedback.SensorToMechanismRatio = Robot.consts.indexer().kExitRollers().GEAR_RATIO();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotionMagic.MotionMagicJerk =
                Robot.consts.indexer().kExitRollers().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.indexer().kExitRollers().MAX_ACCELERATION_RPM();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.indexer().kExitRollers().MAX_SPEED_RPM();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                Robot.consts.indexer().kExitRollers().SUPPLY_CURRENT_LIMIT();
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                Robot.consts.indexer().kExitRollers().PEAK_CURRENT_LIMIT();

        return config;
    }

    public ExitRollersReal() {

        exitRollerMotor =
                new TalonFX(
                        Robot.consts.indexer().kExitRollers().LEADER_MOTOR_ID(),
                        Robot.consts.indexer().kCANBUS());

        exitRollerMotor.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = exitRollerMotor.getVelocity();
    }

    @Override
    public void setSpeedRPM(double speedRpm) {
        if (!Robot.consts.indexer().kExitRollers().disableExitRollersLogs()) {
            Log.log("ROBOT/Subsystems/Indexer/ExitRollers/setSpeed", speedRpm);
        }
        exitRollerMotor.setControl(velocityControl.withVelocity(speedRpm / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        exitRollerMotor.setControl(dutyCycleControl.withOutput(voltage / 12.0));
    }

    @Override
    public boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        double currentRPM = getSpeedRPM();
        return Math.abs(currentRPM - targetRPM) <= toleranceRPM;
    }

    @Override
    public double getSpeedRPM() {
        return exitRollerMotor.getVelocity().getValueAsDouble() * Conv.RPS_TO_RPM;
    }

    @Override
    public void periodic() {

        if (!Robot.consts.indexer().kExitRollers().disableExitRollersLogs()) {
            Log.log("Subsystems/Indexer/ExitRollers/velocity", getSpeedRPM());
        }
    }
}
