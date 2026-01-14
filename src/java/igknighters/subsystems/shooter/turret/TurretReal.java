package igknighters.subsystems.shooter.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class TurretReal extends Turret {

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);
    private final DutyCycleOut voltageControl = new DutyCycleOut(0.0);

    private final TalonFX motor = new TalonFX(SubsystemConstants.Turret.MOTOR_ID);
    private final CANcoder turretCaNcoder = new CANcoder(SubsystemConstants.Turret.CANCODER_ID);

    private final BaseStatusSignal turretAngle = motor.getPosition();
    private final BaseStatusSignal turretCurrent = motor.getStatorCurrent();

    private final TalonFXConfiguration turretConfiguration() {
        var cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = SubsystemConstants.Turret.kP;
        cfg.Slot0.kD = SubsystemConstants.Turret.kD;
        cfg.Slot0.kS = SubsystemConstants.Turret.kS;
        cfg.Slot0.kV = SubsystemConstants.Turret.kV;
        cfg.Slot0.kA = SubsystemConstants.Turret.kA;

        cfg.Feedback.RotorToSensorRatio = SubsystemConstants.Turret.GEAR_RATIO;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = SubsystemConstants.Turret.CANCODER_ID;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.Turret.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.Turret.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;

        cfg.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.Turret.MAX_SPEED_RPM * Conv.RPM_TO_RPS;
        cfg.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.Turret.MAX_ACCELERATION_RPM * Conv.RPM_TO_RPS;

        cfg.CurrentLimits.StatorCurrentLimit = SubsystemConstants.Turret.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.Turret.SUPPLY_CURRENT_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return cfg;
    }

    private final CANcoderConfiguration wristCaNcoderConfiguration() {
        var cfg = new CANcoderConfiguration();

        cfg.MagnetSensor.MagnetOffset = SubsystemConstants.Turret.CANCODER_OFFSET_ROTATIONS;
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection =
                SensorDirectionValue.Clockwise_Positive; // Adjust as needed

        return cfg;
    }

    public TurretReal() {
        turretCaNcoder.getConfigurator().apply(wristCaNcoderConfiguration());
        motor.getConfigurator().apply(turretConfiguration());
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        motor.setPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        super.targetDegrees = angleDegrees;
        motor.setControl(positionControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    public double getAngleDegrees() {
        return turretAngle.getValueAsDouble() * 360.0;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngle, turretCurrent);

        super.degrees = turretAngle.getValueAsDouble() * 360.0;
    }
}
