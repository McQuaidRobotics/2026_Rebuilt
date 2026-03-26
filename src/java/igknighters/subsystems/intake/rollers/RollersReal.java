package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.constants.RobotConsts;
import igknighters.util.log.Log;

public class RollersReal extends Rollers {

    private final RobotConsts consts;
    private final TalonFX intakeMotor;

    private final TalonFX followerMotor;
    private final MotionMagicVelocityVoltage velocityContorl =
            new MotionMagicVelocityVoltage(0.0).withSlot(0);
    private BaseStatusSignal intakeSpeed;

    public RollersReal(RobotConsts consts) {
        this.consts = consts;
        intakeMotor =
                new TalonFX(
                        consts.intake().kRollers().LEADER_MOTOR_ID(), consts.intake().kCANBUS());
        followerMotor =
                new TalonFX(
                        consts.intake().kRollers().FOLLOWER_MOTOR_ID(), consts.intake().kCANBUS());
        intakeMotor.getConfigurator().apply(getLeaderConfig());
        followerMotor.setControl(
                new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        intakeSpeed = intakeMotor.getVelocity();
    }

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = consts.intake().kRollers().kP();
        config.Slot0.kI = consts.intake().kRollers().kI();
        config.Slot0.kD = consts.intake().kRollers().kD();

        config.MotionMagic.MotionMagicJerk = consts.intake().kRollers().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicCruiseVelocity = consts.intake().kRollers().MAX_SPEED_RPM();
        config.MotionMagic.MotionMagicAcceleration =
                consts.intake().kRollers().MAX_ACCELERATION_RPM();
        config.CurrentLimits.StatorCurrentLimit = 35.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = consts.intake().kRollers().GEAR_RATIO();

        return config;
    }

    @Override
    public AngularVelocity getSpeed() {
        return intakeMotor.getVelocity().getValue();
    }

    @Override
    public void goToSpeed(AngularVelocity speed) {
        intakeMotor.setControl(velocityContorl.withVelocity(speed.in(RotationsPerSecond)));
    }

    @Override
    public void stop() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {

        if (!consts.intake().kRollers().disableRollersLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Rollers/SpeedRPS", getSpeed());
        }
    }
}
