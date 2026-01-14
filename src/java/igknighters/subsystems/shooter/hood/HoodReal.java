package igknighters.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class HoodReal extends Hood {
    private final TalonFX motor = new TalonFX(SubsystemConstants.Hood.MOTOR_ID);

    private final BaseStatusSignal flapAngleRots = motor.getPosition();

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);

    public TalonFXConfiguration flapConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.Hood.kP;
        config.Slot0.kI = SubsystemConstants.Hood.kI;
        config.Slot0.kD = SubsystemConstants.Hood.kD;
        config.Slot0.kS = SubsystemConstants.Hood.kS;
        config.Slot0.kV = SubsystemConstants.Hood.kV;
        config.Slot0.kA = SubsystemConstants.Hood.kA;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Hood.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration = SubsystemConstants.Hood.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.Hood.MAX_SPEED_RPM;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.Hood.GEAR_RATIO;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
                SubsystemConstants.Hood.REVERSE_LIMIT_SWITCH_ID;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.Hood.MAX_ANGLE_DEGREES;

        return config;
    }

    public HoodReal() {
        motor.getConfigurator().apply(flapConfiguration());
    }

    @Override
    public double getAngleDegrees() {
        return flapAngleRots.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        motor.setControl(positionControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(flapAngleRots);
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        motor.setPosition(Rotation.of(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }
}
