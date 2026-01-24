package igknighters.subsystems.climber.chainsaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.SubsystemConstants;

public class ChainsawReal extends Chainsaw {
    private final MotionMagicExpoVoltage positionControl =
            new MotionMagicExpoVoltage(0.0).withSlot(0);
    private final DutyCycleOut stop = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private double currentPosition = 0.0;
    private double previousPosition = 0.0;

    private final DigitalInput bumberSensor =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.BUMPER_SENSOR_ID);

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;

    private final CANcoder positionCanCoder =
            new CANcoder(SubsystemConstants.kClimber.kChainsaw.CANCODER_ID);

    // private final TalonFX rightMotor;

    public ChainsawReal() {
        leftMotor = new TalonFX(SubsystemConstants.kClimber.kChainsaw.LEFT_MOTOR_ID);
        // rightMotor = new TalonFX(SubsystemConstants.kClimber.kChainsaw.RIGHT_MOTOR_ID);

        armPosition = leftMotor.getPosition();
        armCurrent = leftMotor.getStatorCurrent();

        leftMotor.getConfigurator().apply(arm1Config());
        // rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
        // MotorAlignmentValue.Opposed));
    }

    public CANcoderConfiguration canCoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = SubsystemConstants.kClimber.kChainsaw.CANCODER_OFFSET;
        config.MagnetSensor.SensorDirection =
                SensorDirectionValue.Clockwise_Positive; // this is made up
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
                1.0; // at 1 rotation it wraps and says its at 0

        return config;
    }

    public TalonFXConfiguration arm1Config() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.kClimber.kChainsaw.kP;
        config.Slot0.kI = SubsystemConstants.kClimber.kChainsaw.kI;
        config.Slot0.kD = SubsystemConstants.kClimber.kChainsaw.kD;
        config.Slot0.kS = SubsystemConstants.kClimber.kChainsaw.kS;
        config.Slot0.kV = SubsystemConstants.kClimber.kChainsaw.kV;
        config.Slot0.kA = SubsystemConstants.kClimber.kChainsaw.kA;

        config.CurrentLimits.StatorCurrentLimit =
                SubsystemConstants.kClimber.kChainsaw.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kClimber.kChainsaw.SUPPLY_CURRENT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kClimber.kChainsaw.MAX_VELOCITY_METERS_PER_SECOND;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kClimber.kChainsaw.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kClimber.kChainsaw.MAX_JERK;
        // DO NOT RUN UNTILL THIS IS FIGURED OUT !!!!!!!!!!!!!!!!!!
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = positionCanCoder.getDeviceID();
        config.Feedback.RotorToSensorRatio =
                SubsystemConstants.kClimber
                        .kChainsaw
                        .GEAR_RATIO; // ratio from motor output to cancoder shaft which is 25/4.5

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES;

        return config;
    }

    @Override
    public void setPositionInches(double positionInches) {
        leftMotor.setPosition(
                positionInches * SubsystemConstants.kClimber.kChainsaw.INCHES_TO_ROTATIONS);
    }

    @Override
    public void coast() {
        leftMotor.setControl(coastControl);
    }

    @Override
    public double getPositionInches() {
        return armPosition.getValueAsDouble()
                * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES;
    }

    @Override
    public void stop() {
        leftMotor.setControl(stop);
    }

    @Override
    public void goToInches(double inches) {
        DogLog.log("Subsystems/Climber/Target", inches);
        leftMotor.setControl(
                positionControl.withPosition(
                        inches * SubsystemConstants.kClimber.kChainsaw.INCHES_TO_ROTATIONS));
    }

    @Override
    public boolean isSensorHit() {
        return !bumberSensor.get(); // assuming that when pushed the current flows
    }

    @Override
    public void periodic() {

        BaseStatusSignal.refreshAll(armPosition, armCurrent);
        DogLog.log(
                "Subsystems/Climber/Inches",
                armPosition.getValueAsDouble()
                        * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES);
        DogLog.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
    }
}
