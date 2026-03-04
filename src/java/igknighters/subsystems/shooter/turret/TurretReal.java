package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.util.log.Log;

public class TurretReal extends Turret {

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);

    private final TalonFX motor =
            new TalonFX(SubsystemConstants.kShooter.kTurret.MOTOR_ID, kShooter.CANBUS);
    private final CANcoder turretCaNcoder =
            new CANcoder(SubsystemConstants.kShooter.kTurret.CANCODER_ID, kShooter.CANBUS);

    private final BaseStatusSignal turretAngle = motor.getPosition();
    private final BaseStatusSignal turretCurrent = motor.getStatorCurrent();

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
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
        cfg.MagnetSensor.SensorDirection =
                SensorDirectionValue.Clockwise_Positive; // Adjust as needed

        return cfg;
    }

    public TurretReal() {
        turretCaNcoder.getConfigurator().apply(wristCaNcoderConfiguration());
        motor.getConfigurator().apply(turretConfiguration());
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setPosition(angle.in(Rotations));
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
    public void goToAngleDegrees(Angle angleDegrees) {
        super.targetDegrees = angleDegrees.in(Degrees);
        double wrappedAngleDegrees = wrapAngleDegrees(angleDegrees.in(Degrees));
        if (!isLegalPositionWrapped(angleDegrees.in(Degrees))) {
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
    public double getAngleDegrees() {
        return turretAngle.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngle, turretCurrent);
        Log.logMotor("Subsystems/Shooter/Turret/Motor", motor);
        if (!SubsystemConstants.kShooter.kTurret.disableTurretLogs) {
            Log.log("Subsystems/Shooter/Turret/Target Degrees", super.targetDegrees);
        }

        super.degrees = turretAngle.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }
}
