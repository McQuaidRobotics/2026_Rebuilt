package igknighters.subsystems.climber.arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class ArmsReal extends Arms {
    private final MotionMagicExpoVoltage positionControl =
            new MotionMagicExpoVoltage(0.0).withSlot(0);
    private final DutyCycleOut stop = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public ArmsReal() {
        leftMotor = new TalonFX(SubsystemConstants.kClimber.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(SubsystemConstants.kClimber.RIGHT_MOTOR_ID);

        armPosition = leftMotor.getPosition();
        armCurrent = leftMotor.getStatorCurrent();

        leftMotor.getConfigurator().apply(arm1Config());
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public TalonFXConfiguration arm1Config() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.kClimber.kP;
        config.Slot0.kI = SubsystemConstants.kClimber.kI;
        config.Slot0.kD = SubsystemConstants.kClimber.kD;
        config.Slot0.kS = SubsystemConstants.kClimber.kS;
        config.Slot0.kV = SubsystemConstants.kClimber.kV;
        config.Slot0.kA = SubsystemConstants.kClimber.kA;

        config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.kClimber.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.kClimber.SUPPLY_CURRENT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kClimber.MAX_VELOCITY_METERS_PER_SECOND;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kClimber.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kClimber.MAX_JERK;
        config.Feedback.RotorToSensorRatio = SubsystemConstants.kClimber.GEAR_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kClimber.MAX_ANGLE_DEGREES;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kClimber.MIN_ANGLE_DEGREES;

        return config;
    }

    @Override
    void setPosition(double angleDegrees) {
        leftMotor.setPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS);
    }

    @Override
    void coast() {
        leftMotor.setControl(coastControl);
    }

    @Override
    double getPositionDegrees() {
        return armPosition.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    void stop() {
        leftMotor.setControl(stop);
    }

    @Override
    void goToAngleDegrees(double angleDegrees) {
        leftMotor.setControl(
                positionControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    void periodic() {
        BaseStatusSignal.refreshAll(armPosition, armCurrent);
        DogLog.log(
                "Subsystems/Climber/Position",
                armPosition.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES);
        DogLog.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
    }
}
