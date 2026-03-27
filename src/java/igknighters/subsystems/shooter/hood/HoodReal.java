package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.Robot;
import igknighters.util.log.Log;

public class HoodReal extends Hood {
    private final TalonFX motor;

    private final DigitalInput reverseLimitSwitch;
    private Angle targetAngle;
    private boolean hasHomed = false;

    private final PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(0);

    public TalonFXConfiguration flapConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Robot.consts.shooter().kHood().kP();
        config.Slot0.kI = Robot.consts.shooter().kHood().kI();
        config.Slot0.kD = Robot.consts.shooter().kHood().kD();
        config.Slot0.kS = Robot.consts.shooter().kHood().kS();
        config.Slot0.kV = Robot.consts.shooter().kHood().kV();
        config.Slot0.kA = Robot.consts.shooter().kHood().kA();

        config.MotionMagic.MotionMagicJerk = Robot.consts.shooter().kHood().MAX_JERK();
        config.MotionMagic.MotionMagicAcceleration =
                Robot.consts.shooter().kHood().MAX_ACCEL_R_P_S_S();
        config.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.shooter().kHood().MAX_SPEED_R_P_S();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 1.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES()
                        / Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES();
        return config;
    }

    public HoodReal() {
        reverseLimitSwitch =
                new DigitalInput(Robot.consts.shooter().kHood().REVERSE_LIMIT_SWITCH_ID());
        targetAngle = Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
        motor =
                new TalonFX(
                        Robot.consts.shooter().kHood().MOTOR_ID(),
                        Robot.consts.shooter().kCANBUS());
        if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/Hood/Initialized", true);
        }
        motor.getConfigurator().apply(flapConfiguration());
    }

    @Override
    public double getAngleDegrees() {
        return motor.getPosition().getValueAsDouble()
                * Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES();
    }

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()
                && angleDegrees <= Robot.consts.shooter().kHood().MAX_ANGLE_DEGREES();
    }

    @Override
    public void goToAngle(Angle targetAngle) {
        if (!isLegalPosition(targetAngle.in(Degrees))) {
            if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
                Log.log("ROBOT/Subsystems/Shooter/Hood/IllegalPosition", targetAngle.in(Degrees));
            }
            return;
        }
        this.targetAngle = targetAngle;
        super.targetDegrees = targetAngle.in(Degrees);
        motor.setControl(
                positionControl.withPosition(
                        Rotations.of(
                                targetAngle.in(Degrees)
                                        / Robot.consts
                                                .shooter()
                                                .kHood()
                                                .MOTOR_ROTS_TO_HOOD_DEGREES())));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public boolean isSensorHit() {
        return reverseLimitSwitch.get();
    }

    public void handleLimitSwitch() {
        if (reverseLimitSwitch.get() && targetAngle.in(Degrees) < getAngleDegrees()) {
            if (!hasHomed) {
                hasHomed = true;
                setAngle(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
            }
            motor.setVoltage(0.0);
        } else if (reverseLimitSwitch.get() && !hasHomed) {
            hasHomed = true;
            setAngle(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
        } else if (!reverseLimitSwitch.get()) {
            hasHomed = false;
        }
    }

    @Override
    public void periodic() {

        handleLimitSwitch();
        if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
            Log.log("ROBOT/Subsystems/Shooter/Hood/Homing", hasHomed);
            Log.log("ROBOT/Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
            Log.log("ROBOT/Subsystems/Shooter/Hood/ReverseLimitSwitch", reverseLimitSwitch.get());
        }
    }

    @Override
    public void setAngle(double angleDegrees) {
        motor.setPosition(
                Rotation.of(
                        angleDegrees
                                / Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES()));
    }

    public void setAngle(Angle angle) {
        motor.setPosition(
                Rotations.of(
                        angle.in(Rotations)
                                / Robot.consts.shooter().kHood().MOTOR_ROTS_TO_HOOD_DEGREES()));
    }
}
