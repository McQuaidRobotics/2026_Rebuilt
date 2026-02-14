package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;

public class HoodReal extends Hood {
    private final TalonFX motor = new TalonFX(SubsystemConstants.kShooter.kHood.MOTOR_ID);
    private final DigitalInput reverseLimitSwitch =
            new DigitalInput(kShooter.kHood.REVERSE_LIMIT_SWITCH_ID);
    private final BaseStatusSignal flapAngleRots = motor.getPosition();

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);

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
                SubsystemConstants.kShooter.kHood.MAX_ACCELERATION_DEGREES_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kShooter.kHood.MAX_SPEED_DEGREES_PER_SECOND;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kShooter.kHood.GEAR_RATIO;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES;

        return config;
    }

    public HoodReal() {
        DogLog.log("Subsystems/Shooter/Hood/Initialized", true);
        motor.getConfigurator().apply(flapConfiguration());
    }

    @Override
    public double getAngleDegrees() {
        return flapAngleRots.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        if (!reverseLimitSwitch.get()) {
            DogLog.log("Subsystems/Shooter/Hood/IS LIMIT TRIPPED", true);
            motor.setVoltage(0.0);
            setAngleDegrees(kShooter.kHood.MIN_ANGLE_DEGREES);
        } else {
            DogLog.log("Subsystems/Shooter/Hood/IS LIMIT TRIPPED", false);
            motor.setControl(
                    positionControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(flapAngleRots);
        DogLog.log("Subsystems/Shooter/Hood/AngleDegrees", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Hood/TargetDegrees", super.targetDegrees);
        DogLog.log("Subsystems/Shooter/Hood/ReverseLimitSwitch", !reverseLimitSwitch.get());
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        motor.setPosition(Rotation.of(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }
}
