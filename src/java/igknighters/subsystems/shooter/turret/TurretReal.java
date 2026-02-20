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
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.constants.SubsystemConstants.kShooter.kTurret;

public class TurretReal extends Turret {

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);
    private final DutyCycleOut voltageControl = new DutyCycleOut(0.0);

    private final TalonFX motor =
            new TalonFX(SubsystemConstants.kShooter.kTurret.MOTOR_ID, kShooter.CANBUS);
    private final CANcoder turretCaNcoder =
            new CANcoder(SubsystemConstants.kShooter.kTurret.CANCODER_ID, kShooter.CANBUS);

    private final BaseStatusSignal turretAngle = motor.getPosition();
    private final BaseStatusSignal turretCurrent = motor.getStatorCurrent();
    private final BaseStatusSignal canCoderAngle = turretCaNcoder.getAbsolutePosition();

    private final TalonFXConfiguration turretConfiguration() {
        var cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = SubsystemConstants.kShooter.kTurret.kP;
        cfg.Slot0.kD = SubsystemConstants.kShooter.kTurret.kD;
        cfg.Slot0.kS = SubsystemConstants.kShooter.kTurret.kS;
        cfg.Slot0.kV = SubsystemConstants.kShooter.kTurret.kV;
        cfg.Slot0.kA = SubsystemConstants.kShooter.kTurret.kA;

        cfg.Feedback.RotorToSensorRatio = SubsystemConstants.kShooter.kTurret.GEAR_RATIO;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.RemoteCANcoder; // should be fused but rio bomb not pro
        cfg.Feedback.FeedbackRemoteSensorID = SubsystemConstants.kShooter.kTurret.CANCODER_ID;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kShooter.kTurret.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kShooter.kTurret.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;

        cfg.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kShooter.kTurret.MAX_SPEED_RPM * Conv.RPM_TO_RPS;
        cfg.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kShooter.kTurret.MAX_ACCELERATION_RPM * Conv.RPM_TO_RPS;

        cfg.CurrentLimits.StatorCurrentLimit =
                SubsystemConstants.kShooter.kTurret.STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kShooter.kTurret.SUPPLY_CURRENT_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return cfg;
    }

    private final CANcoderConfiguration wristCaNcoderConfiguration() {
        var cfg = new CANcoderConfiguration();

        cfg.MagnetSensor.MagnetOffset =
                SubsystemConstants.kShooter.kTurret.CANCODER_OFFSET_ROTATIONS;
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

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= SubsystemConstants.kShooter.kTurret.MIN_ANGLE_DEGREES
                && angleDegrees <= SubsystemConstants.kShooter.kTurret.MAX_ANGLE_DEGREES;
    }

    public boolean isLegalPositionWrapped(double angleDegrees) {
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees);
        return isLegalPosition(wrappedAngleDegrees);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        super.targetDegrees = angleDegrees;
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees);
        if (!isLegalPositionWrapped(angleDegrees)) {
            DriverStation.reportError(
                    "Turret angle out of bounds: "
                            + wrappedAngleDegrees
                            + " degrees. Commanded: "
                            + angleDegrees,
                    false);
            return;
        }
        motor.setControl(
                positionControl.withPosition(wrappedAngleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    public double getAngleDegrees(){
        return turretAngle.getValueAsDouble() * 360.0;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngle, turretCurrent, canCoderAngle);
        DogLog.log(
                "Subsystems/Shooter/Turret/Position (deg)", getAngleDegrees());
        DogLog.log("Subsystems/Shooter/Turret/Current (A)", turretCurrent.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Turret/Target Degrees", super.targetDegrees);
        DogLog.log(
                "Subsystems/Shooter/Turret/CANcoder Angle (deg)",
                canCoderAngle.getValueAsDouble() * 360.0);

        super.degrees = turretAngle.getValueAsDouble() * 360.0;
    }
}
