package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.Robot;
import igknighters.util.log.Log;

public class RollersReal extends Rollers {

    private final TalonFX topMotor;

    private final TalonFX bottomMotor;
    private final MotionMagicVelocityVoltage velocityContorl =
            new MotionMagicVelocityVoltage(0.0).withSlot(0);

    public RollersReal() {
        topMotor =
                new TalonFX(
                        Robot.consts.intake().kRollers().LEADER_MOTOR_ID(),
                        Robot.consts.intake().kCANBUS());
        bottomMotor =
                new TalonFX(
                        Robot.consts.intake().kRollers().FOLLOWER_MOTOR_ID(),
                        Robot.consts.intake().kCANBUS());
        topMotor.getConfigurator().apply(getTopConfig());
        bottomMotor.getConfigurator().apply(getBottomConfiguration());
    }

    public TalonFXConfiguration getTopConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Robot.consts.intake().kRollers().kP();
        config.Slot0.kI = Robot.consts.intake().kRollers().kI();
        config.Slot0.kD = Robot.consts.intake().kRollers().kD();

        config.MotionMagic.MotionMagicJerk = Robot.consts.intake().kRollers().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.intake().kRollers().MAX_SPEED_RPM();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.intake().kRollers().MAX_ACCELERATION_RPM();
        config.CurrentLimits.StatorCurrentLimit = 35.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = Robot.consts.intake().kRollers().GEAR_RATIO();

        return config;
    }

    public TalonFXConfiguration getBottomConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Robot.consts.intake().kRollers().kP();
        config.Slot0.kI = Robot.consts.intake().kRollers().kI();
        config.Slot0.kD = Robot.consts.intake().kRollers().kD();

        config.MotionMagic.MotionMagicJerk = Robot.consts.intake().kRollers().MOTION_MAGIC_JERK();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.intake().kRollers().MAX_SPEED_RPM();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.intake().kRollers().MAX_ACCELERATION_RPM();
        config.CurrentLimits.StatorCurrentLimit = 35.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = Robot.consts.intake().kRollers().GEAR_RATIO();

        return config;
    }

    @Override
    public AngularVelocity getSpeed() {
        return topMotor.getVelocity().getValue();
    }

    @Override
    public void goToSpeed(AngularVelocity speed) {
        topMotor.setControl(velocityContorl.withVelocity(speed.in(RotationsPerSecond)));
        bottomMotor.setControl(
                velocityContorl.withVelocity(
                        speed.in(RotationsPerSecond)
                                * Robot.consts.intake().kRollers().DRIVE_RATIO()));
    }

    @Override
    public void stop() {
        topMotor.setVoltage(0.0);
        bottomMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {

        if (!Robot.consts.intake().kRollers().disableRollersLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Rollers/SpeedRPSTOP", getSpeed());
            Log.log(
                    "ROBOT/Subsystems/Intake/Rollers/SpeedRPSBOTTOM",
                    bottomMotor.getVelocity().refresh().getValueAsDouble());
        }
    }
}
