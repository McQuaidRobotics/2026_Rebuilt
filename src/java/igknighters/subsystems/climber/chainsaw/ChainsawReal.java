package igknighters.subsystems.climber.chainsaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kClimber;

public class ChainsawReal extends Chainsaw {
    private final MotionMagicExpoVoltage positionControl =
            new MotionMagicExpoVoltage(0.0).withSlot(0);
    private final DutyCycleOut stop = new DutyCycleOut(0.0);
    private final CoastOut coastControl = new CoastOut();

    private double currentPosition = 0.0;
    private double previousPosition = 0.0;

    private final DigitalInput bumperSensor =
            new DigitalInput(SubsystemConstants.kClimber.kChainsaw.BUMPER_SENSOR_ID);

    private final BaseStatusSignal armPosition, armCurrent;

    private final TalonFX leftMotor;

    //     private final CANcoder positionCanCoder =
    //             new CANcoder(SubsystemConstants.kClimber.kChainsaw.CANCODER_ID);

    // private final TalonFX rightMotor;

    public ChainsawReal() {
        leftMotor =
                new TalonFX(SubsystemConstants.kClimber.kChainsaw.LEFT_MOTOR_ID, kClimber.CANBUS);

        armPosition = leftMotor.getPosition();
        armCurrent = leftMotor.getStatorCurrent();

        leftMotor.getConfigurator().apply(arm1Config());
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

        // config.Feedback.RotorToSensorRatio =
        //         SubsystemConstants.kClimber
        //                 .kChainsaw
        //                 .GEAR_RATIO; // ratio from motor output to gears = 25

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.kClimber.kChainsaw.GEAR_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES;

        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_INCHES;

        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
                SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_INCHES;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID =
                SubsystemConstants.kClimber.kChainsaw.MAX_HEIGHT_SENSOR_ID;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
                SubsystemConstants.kClimber.kChainsaw.MIN_HEIGHT_SENSOR_ID;

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
        return !bumperSensor.get(); // assuming that when pushed the current flows
    }

    @Override
    public void periodic() {

        BaseStatusSignal.refreshAll(armPosition, armCurrent);
        DogLog.log(
                "Subsystems/Climber/Inches",
                armPosition.getValueAsDouble()
                        * SubsystemConstants.kClimber.kChainsaw.ROTATIONS_TO_INCHES);
        DogLog.log(
                "Subsystems/Climber/IS REVERSE LIMIT HIT",
                leftMotor.getReverseLimit().refresh().getValue().equals(ReverseLimitValue.Open));
        DogLog.log(
                "Subsystems/Climber/IS FORWARD LIMIT HIT",
                leftMotor.getForwardLimit().refresh().getValue().equals(ForwardLimitValue.Open));
        DogLog.log("Subsystems/Climber/Current", armCurrent.getValueAsDouble());
    }
}
