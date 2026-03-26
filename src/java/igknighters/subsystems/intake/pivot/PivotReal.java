package igknighters.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import igknighters.constants.Conv;
import igknighters.constants.RobotConsts;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake;
import igknighters.util.log.Log;

public class PivotReal extends Pivot {
    private final RobotConsts consts;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;
    private PositionVoltage motionMagicControl;
    private double targetDegrees = 0.0;
    private boolean beingCommanded = false;

    public PivotReal(RobotConsts consts) {
        this.consts = consts;
        pivotMotor = new TalonFX(consts.intake().kPivot().MOTOR_ID(), consts.intake().kCANBUS());
        pivotMotor.getConfigurator().apply(getPivotConfig());

        pivotEncoder = new CANcoder(consts.intake().kPivot().CANCODER_ID(), consts.intake().kCANBUS());
        pivotEncoder.getConfigurator().apply(getPivotEncoderConfig());

        motionMagicControl = new PositionVoltage(0.0).withSlot(0);
    }

    public CANcoderConfiguration getPivotEncoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = consts.intake().kPivot().ENCODER_OFFSET();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return config;
    }

    public TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = consts.intake().kPivot().kP();
        config.Slot0.kI = consts.intake().kPivot().kI();
        config.Slot0.kD = consts.intake().kPivot().kD();
        config.Slot0.kS = consts.intake().kPivot().kS();
        config.Slot0.kV = consts.intake().kPivot().kV();
        config.Slot0.kA = consts.intake().kPivot().kA();

        config.Feedback.FeedbackRemoteSensorID = consts.intake().kPivot().CANCODER_ID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.CurrentLimits.StatorCurrentLimit =
        //         consts.intake().kPivot().STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit =
                consts.intake().kPivot().SUPPLY_CURRENT_LIMIT();

        config.CurrentLimits.SupplyCurrentLimit =
                consts.intake().kPivot().SUPPLY_UPPER_LIMIT();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

        config.MotionMagic.MotionMagicCruiseVelocity =
                consts.intake().kPivot().MAX_SPEED_METERS_PER_SECOND();
        config.MotionMagic.MotionMagicAcceleration =
                consts.intake().kPivot().MAX_ACCELERATION_METERS_PER_SECOND_SQUARED();
        config.MotionMagic.MotionMagicJerk = consts.intake().kPivot().MAX_JERK();
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                consts.intake().kPivot().MAX_ANGLE_DEGREES() * Conv.DEGREES_TO_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                consts.intake().kPivot().MIN_ANGLE_DEGREES() * Conv.DEGREES_TO_ROTATIONS;

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
        if (!consts.intake().kPivot().disablePivotLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Stopped", false);
        }
        pivotMotor.setControl(motionMagicControl.withPosition(angle.in(Rotation)));
    }

    @Override
    public void stop() {
        beingCommanded = true;
        if (!consts.intake().kPivot().disablePivotLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Stopped", true);
        }
        pivotMotor.setVoltage(0.0);
    }

    @Override
    public Angle getAngle() {
        return Rotation.of(pivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {

        Log.log("ROBOT/Subsystems/Intake/Pivot/POSITION", getAngle().in(Degrees));

        if (!consts.intake().kPivot().disablePivotLogs()) {
            Log.log("ROBOT/Subsystems/Intake/Pivot/Being Commanded Currently", beingCommanded);
            Log.logMotor("Subsystems/Intake/Pivot/Motor", pivotMotor);
            Log.log("ROBOT/Subsystems/Intake/Pivot/Target", targetDegrees);
        }
    }
}
