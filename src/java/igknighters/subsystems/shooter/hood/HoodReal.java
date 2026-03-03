package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.log.Log;

public class HoodReal extends Hood {
    private final TalonFX motor =
            new TalonFX(SubsystemConstants.kShooter.kHood.MOTOR_ID, kShooter.CANBUS);

    private final BaseStatusSignal flapAngleRots = motor.getPosition();
    private final DigitalInput reverseLimitSwitch =
            new DigitalInput(kShooter.kHood.REVERSE_LIMIT_SWITCH_ID);
    private final BaseStatusSignal motorRots = motor.getPosition();
    private Angle targetAngle = Degrees.of(kHood.MIN_ANGLE_DEGREES);
    private boolean hasHomed = false;

    private final PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(0);

    public TalonFXConfiguration flapConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.kShooter.kHood.kP;
        config.Slot0.kI = SubsystemConstants.kShooter.kHood.kI;
        config.Slot0.kD = SubsystemConstants.kShooter.kHood.kD;
        config.Slot0.kS = SubsystemConstants.kShooter.kHood.kS;
        config.Slot0.kV = SubsystemConstants.kShooter.kHood.kV;
        config.Slot0.kA = SubsystemConstants.kShooter.kHood.kA;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kShooter.kHood.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kShooter.kHood.MAX_ACCEL_R_P_S_S;
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kShooter.kHood.MAX_SPEED_R_P_S;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 1.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES
                        / kHood.MOTOR_ROTS_TO_HOOD_DEGREES;
        return config;
    }

    public HoodReal() {
        if (!SubsystemConstants.kShooter.kHood.disableHoodLogs) {
            Log.log("Subsystems/Shooter/Hood/Initialized", true);
        }
        motor.getConfigurator().apply(flapConfiguration());
    }

    @Override
    public double getAngleDegrees() {
        return motorRots.getValueAsDouble() * kHood.MOTOR_ROTS_TO_HOOD_DEGREES;
    }

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= kHood.MIN_ANGLE_DEGREES && angleDegrees <= kHood.MAX_ANGLE_DEGREES;
    }

    @Override
    public void goToAngle(Angle targetAngle) {
        if (!isLegalPosition(targetAngle.in(Degrees))) {
            if (!SubsystemConstants.kShooter.kHood.disableHoodLogs) {
                Log.log("Subsystems/Shooter/Hood/IllegalPosition", targetAngle.in(Degrees));
            }
            return;
        }
        this.targetAngle = targetAngle;
        super.targetDegrees = targetAngle.in(Degrees);
        motor.setControl(
                positionControl.withPosition(
                        Rotations.of(targetAngle.in(Degrees) / kHood.MOTOR_ROTS_TO_HOOD_DEGREES)));
    }

    public void handleLimitSwitch() {
        if (reverseLimitSwitch.get() && targetAngle.in(Degrees) < getAngleDegrees()) {
            if (!hasHomed) {
                hasHomed = true;
                setAngle(kHood.MIN_ANGLE_DEGREES);
            }
            motor.setVoltage(0.0);
        } else if (reverseLimitSwitch.get() && !hasHomed) {
            hasHomed = true;
            setAngle(kHood.MIN_ANGLE_DEGREES);
        } else if (!reverseLimitSwitch.get()) {
            hasHomed = false;
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorRots);
        handleLimitSwitch();
        if (!SubsystemConstants.kShooter.kHood.disableHoodLogs) {
            Log.log("Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
            Log.log("Subsystems/Shooter/Hood/Homing", hasHomed);
            Log.log("Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
            Log.log("Subsystems/Shooter/Hood/ReverseLimitSwitch", reverseLimitSwitch.get());
        }
    }

    @Override
    public void setAngle(double angleDegrees) {
        motor.setPosition(Rotation.of(angleDegrees / kHood.MOTOR_ROTS_TO_HOOD_DEGREES));
    }

    public void setAngle(Angle angle) {
        motor.setPosition(Rotations.of(angle.in(Rotations) / kHood.MOTOR_ROTS_TO_HOOD_DEGREES));
    }
}
