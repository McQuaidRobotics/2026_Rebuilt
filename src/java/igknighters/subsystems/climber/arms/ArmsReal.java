package igknighters.subsystems.climber.arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class ArmsReal extends Arms {
    private final MotionMagicExpoVoltage positionControl =
            new MotionMagicExpoVoltage(0.0).withSlot(0);
    private final DutyCycleOut stop = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX armMotor;
    private final TalonFX armMotor2;

    public ArmsReal() {
        armMotor = new TalonFX(SubsystemConstants.Climber.ARM_MOTOR_ID);
        armMotor2 = new TalonFX(SubsystemConstants.Climber.ARM_MOTOR2_ID);

        armPosition = armMotor.getPosition();
        armCurrent = armMotor.getStatorCurrent();

        armMotor.getConfigurator().apply(arm1Config());
        armMotor2.setControl(new Follower(armMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public TalonFXConfiguration arm1Config() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.Climber.kP;
        config.Slot0.kI = SubsystemConstants.Climber.kI;
        config.Slot0.kD = SubsystemConstants.Climber.kD;
        config.Slot0.kS = SubsystemConstants.Climber.kS;
        config.Slot0.kV = SubsystemConstants.Climber.kV;
        config.Slot0.kA = SubsystemConstants.Climber.kA;

        config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.Climber.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.Climber.SUPPLY_CURRENT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.Climber.MAX_VELOCITY_METERS_PER_SECOND;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.Climber.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Climber.MAX_JERK;
        config.Feedback.RotorToSensorRatio = SubsystemConstants.Climber.GEAR_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.Climber.MAX_ANGLE_DEGREES;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.Climber.MIN_ANGLE_DEGREES;

        return config;
    }

    @Override
    void setPosition(double angleDegrees) {
        armMotor.setPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS);
    }

    @Override
    void coast() {
        armMotor.setControl(coastControl);
    }

    @Override
    double getPositionDegrees() {
        return armPosition.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    void stop() {
        armMotor.setControl(stop);
    }

    @Override
    void goToAngleDegrees(double angleDegrees) {
        armMotor.setControl(positionControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    void periodic() {
        BaseStatusSignal.refreshAll(armPosition, armCurrent);
    }
}
