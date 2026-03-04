package igknighters.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake;
import igknighters.util.log.Log;

public class PivotReal extends Pivot {
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;
    private PositionVoltage motionMagicControl;
    private BaseStatusSignal rps, angleRotations;
    private double targetDegrees = 0.0;
    private boolean beingCommanded = false;

    public PivotReal() {
        pivotMotor = new TalonFX(SubsystemConstants.kIntake.kPivot.MOTOR_ID, kIntake.CANBUS);
        pivotMotor.getConfigurator().apply(getPivotConfig());

        pivotEncoder = new CANcoder(SubsystemConstants.kIntake.kPivot.CANCODER_ID, kIntake.CANBUS);
        pivotEncoder.getConfigurator().apply(getPivotEncoderConfig());

        motionMagicControl = new PositionVoltage(0.0).withSlot(0);

        rps = pivotMotor.getVelocity();
        angleRotations = pivotMotor.getPosition();
    }

    public CANcoderConfiguration getPivotEncoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = SubsystemConstants.kIntake.kPivot.ENCODER_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return config;
    }

    public TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.kIntake.kPivot.kP;
        config.Slot0.kI = SubsystemConstants.kIntake.kPivot.kI;
        config.Slot0.kD = SubsystemConstants.kIntake.kPivot.kD;
        config.Slot0.kS = SubsystemConstants.kIntake.kPivot.kS;
        config.Slot0.kV = SubsystemConstants.kIntake.kPivot.kV;
        config.Slot0.kA = SubsystemConstants.kIntake.kPivot.kA;

        config.Feedback.FeedbackRemoteSensorID = SubsystemConstants.kIntake.kPivot.CANCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit =
                SubsystemConstants.kIntake.kPivot.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kIntake.kPivot.SUPPLY_CURRENT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kIntake.kPivot.MAX_SPEED_METERS_PER_SECOND;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kIntake.kPivot.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kIntake.kPivot.MAX_JERK;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = 1.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kIntake.kPivot.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kIntake.kPivot.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;

        return config;
    }

    @Override
    public void setAngle(Angle angle) {
        pivotMotor.setPosition(angle.in(Rotation));
    }

    @Override
    public void goToAngle(Angle angle) {
        targetDegrees = angle.in(Degrees);
        beingCommanded = true;
        if (!SubsystemConstants.kIntake.kPivot.disablePivotLogs) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Stopped", false);
            pivotMotor.setControl(motionMagicControl.withPosition(angle.in(Rotation)));
        }
    }

    @Override
    public void stop() {
        beingCommanded = true;
        if (!SubsystemConstants.kIntake.kPivot.disablePivotLogs) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Stopped", true);
        }
        pivotMotor.setVoltage(0.0);
    }

    @Override
    public Angle getAngle() {
        return Rotation.of(angleRotations.getValueAsDouble());
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(rps, angleRotations);

        if (!SubsystemConstants.kIntake.kPivot.disablePivotLogs) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Being Commanded Currently", beingCommanded);
            Log.logMotor("Subsystems/Intake/Pivot/Motor", pivotMotor);
            Log.log("ROBOT/Subsystems/Intake/Pivot/Target", targetDegrees);
        }
    }
}
