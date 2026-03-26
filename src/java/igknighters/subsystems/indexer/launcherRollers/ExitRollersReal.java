package igknighters.subsystems.indexer.launcherRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import igknighters.constants.Conv;
import igknighters.constants.RobotConsts;
import igknighters.util.log.Log;

public class ExitRollersReal extends ExitRollers {

    private final RobotConsts consts;

    private final TalonFX exitRollerMotor;
    private final MotionMagicVelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private BaseStatusSignal shooterVelocity;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = consts.indexer().kExitRollers().kP();
        config.Slot0.kI = consts.indexer().kExitRollers().kI();
        config.Slot0.kD = consts.indexer().kExitRollers().kD();
        config.Slot0.kS = consts.indexer().kExitRollers().kS();
        config.Slot0.kV = consts.indexer().kExitRollers().kV();
        config.Feedback.SensorToMechanismRatio = consts.indexer().kExitRollers().GEAR_RATIO();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotionMagic.MotionMagicJerk = consts.indexer().kExitRollers().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicAcceleration =
                consts.indexer().kExitRollers().MAX_ACCELERATION_RPM();
        config.MotionMagic.MotionMagicCruiseVelocity =
                consts.indexer().kExitRollers().MAX_SPEED_RPM();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit =
                consts.indexer().kExitRollers().SUPPLY_CURRENT_LIMIT();
        config.MotorOutput.PeakReverseDutyCycle = 0.0; // do not allow the motor to run in reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                consts.indexer().kExitRollers().PEAK_CURRENT_LIMIT();

        return config;
    }

    public ExitRollersReal(RobotConsts consts) {
        this.consts = consts;

        exitRollerMotor =
                new TalonFX(
                        consts.indexer().kExitRollers().LEADER_MOTOR_ID(),
                        consts.indexer().kCANBUS());

        exitRollerMotor.getConfigurator().apply(getLeaderConfig());

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = exitRollerMotor.getVelocity();
    }

    @Override
    public void setSpeedRPM(double speedRpm) {
        if (!consts.indexer().kExitRollers().disableExitRollersLogs()) {
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

        if (!consts.indexer().kExitRollers().disableExitRollersLogs()) {
            Log.log("Subsystems/Indexer/ExitRollers/velocity", getSpeedRPM());
        }
    }
}
