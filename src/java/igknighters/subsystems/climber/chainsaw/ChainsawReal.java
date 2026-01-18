package igknighters.subsystems.climber.chainsaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class ChainsawReal extends Chainsaw {
    private final MotionMagicExpoVoltage positionControl =
            new MotionMagicExpoVoltage(0.0).withSlot(0);
    private final DutyCycleOut stop = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;

    // private final TalonFX rightMotor;

    public ChainsawReal() {
        leftMotor = new TalonFX(SubsystemConstants.kClimber.LEFT_MOTOR_ID);
        // rightMotor = new TalonFX(SubsystemConstants.kClimber.RIGHT_MOTOR_ID);

        armPosition = leftMotor.getPosition();
        armCurrent = leftMotor.getStatorCurrent();

        leftMotor.getConfigurator().apply(arm1Config());
        // rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
        // MotorAlignmentValue.Opposed));
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
                SubsystemConstants.kClimber.MAX_HEIGHT_INCHES;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kClimber.MIN_HEIGHT_INCHES;

        return config;
    }

    @Override
    void setPositionInches(double positionInches) {
        leftMotor.setPosition(positionInches);
    }

    @Override
    void coast() {
        leftMotor.setControl(coastControl);
    }

    @Override
    double getPositionInches() {
        return armPosition.getValueAsDouble();
    }

    @Override
    void stop() {
        leftMotor.setControl(stop);
    }

    @Override
    void goToInches(double inches) {
        DogLog.log("Subsystems/Climber/Target", inches);
        leftMotor.setControl(positionControl.withPosition(inches));
    }

    @Override
    void periodic() {

        BaseStatusSignal.refreshAll(armPosition, armCurrent);
        DogLog.log(
                "Subsystems/Climber/Inches",
                armPosition.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES);
        DogLog.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
    }
}
