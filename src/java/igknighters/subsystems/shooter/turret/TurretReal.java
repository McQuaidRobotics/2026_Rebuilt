package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.util.log.Log;

public class TurretReal extends Turret {

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withSlot(0);

    private final TalonFX motor;
    private final CANcoder turretCaNcoder;

    private final TalonFXConfiguration turretConfiguration() {
        var cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = Robot.consts.shooter().kTurret().kP();
        cfg.Slot0.kD = Robot.consts.shooter().kTurret().kD();
        cfg.Slot0.kS = Robot.consts.shooter().kTurret().kS();
        cfg.Slot0.kV = Robot.consts.shooter().kTurret().kV();
        cfg.Slot0.kA = Robot.consts.shooter().kTurret().kA();

        cfg.Feedback.RotorToSensorRatio = Robot.consts.shooter().kTurret().GEAR_RATIO();
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.RemoteCANcoder; // should be fused but rio bomb not pro
        cfg.Feedback.FeedbackRemoteSensorID = Robot.consts.shooter().kTurret().CANCODER_ID();

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES() * Conv.DEGREES_TO_ROTATIONS;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES() * Conv.DEGREES_TO_ROTATIONS;

        cfg.MotionMagic.MotionMagicCruiseVelocity =
                Robot.consts.shooter().kTurret().MAX_SPEED_RPM() * Conv.RPM_TO_RPS;
        cfg.MotionMagic.MotionMagicAcceleration =
                Robot.consts.shooter().kTurret().MAX_ACCELERATION_RPM() * Conv.RPM_TO_RPS;

        cfg.CurrentLimits.StatorCurrentLimit =
                Robot.consts.shooter().kTurret().STATOR_CURRENT_LIMIT();
        cfg.CurrentLimits.SupplyCurrentLimit =
                Robot.consts.shooter().kTurret().SUPPLY_CURRENT_LIMIT();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // inverted used to be c p

        return cfg;
    }

    private final CANcoderConfiguration wristCaNcoderConfiguration() {
        var cfg = new CANcoderConfiguration();

        cfg.MagnetSensor.MagnetOffset =
                Robot.consts.shooter().kTurret().CANCODER_OFFSET_ROTATIONS();
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
        cfg.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive; // used to be c p

        return cfg;
    }

    public TurretReal() {
        turretCaNcoder =
                new CANcoder(
                        Robot.consts.shooter().kTurret().CANCODER_ID(),
                        Robot.consts.shooter().kCANBUS());
        motor =
                new TalonFX(
                        Robot.consts.shooter().kTurret().MOTOR_ID(),
                        Robot.consts.shooter().kCANBUS());
        turretCaNcoder.getConfigurator().apply(wristCaNcoderConfiguration());
        motor.getConfigurator().apply(turretConfiguration());
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setPosition(angle.in(Rotations));
    }

    public boolean isLegalPosition(double angleDegrees) {
        return angleDegrees >= Robot.consts.shooter().kTurret().MIN_ANGLE_DEGREES()
                && angleDegrees <= Robot.consts.shooter().kTurret().MAX_ANGLE_DEGREES();
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
        return motor.getPosition().getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void periodic() {

        if (!Robot.consts.shooter().kTurret().disableTurretLogs()) {
            //     Log.logMotor("Subsystems/Shooter/Turret/Motor", motor);
            Log.log("ROBOT/Subsystems/Shooter/Turret/Target Degrees", super.targetDegrees);
        }

        super.degrees = motor.getPosition().getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }
}
