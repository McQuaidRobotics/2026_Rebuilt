package igknighters.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class PivotReal extends Pivot {
    private TalonFX pivotMotor;
    private MotionMagicVoltage motionMagicControl;
    private BaseStatusSignal rps, angleRotations;
    private double targetDegrees = 0.0;
    private boolean beingCommanded = false;

    public PivotReal() {
        pivotMotor = new TalonFX(SubsystemConstants.kIntake.kPivot.MOTOR_ID);
        pivotMotor.getConfigurator().apply(getPivotConfig());

        motionMagicControl = new MotionMagicVoltage(0.0).withSlot(0);

        rps = pivotMotor.getVelocity();
        angleRotations = pivotMotor.getPosition();
    }

    public TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.kIntake.kPivot.kP;
        config.Slot0.kI = SubsystemConstants.kIntake.kPivot.kI;
        config.Slot0.kD = SubsystemConstants.kIntake.kPivot.kD;
        config.Slot0.kS = SubsystemConstants.kIntake.kPivot.kS;
        config.Slot0.kV = SubsystemConstants.kIntake.kPivot.kV;
        config.Slot0.kA = SubsystemConstants.kIntake.kPivot.kA;

        config.CurrentLimits.StatorCurrentLimit =
                SubsystemConstants.kIntake.kPivot.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit =
                SubsystemConstants.kIntake.kPivot.SUPPLY_CURRENT_LIMIT;

        config.MotionMagic.MotionMagicCruiseVelocity =
                SubsystemConstants.kIntake.kPivot.MAX_SPEED_METERS_PER_SECOND;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.kIntake.kPivot.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        config.MotionMagic.MotionMagicJerk = SubsystemConstants.kIntake.kPivot.MAX_JERK;
        config.Feedback.RotorToSensorRatio = SubsystemConstants.kIntake.kPivot.GEAR_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                SubsystemConstants.kIntake.kPivot.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                SubsystemConstants.kIntake.kPivot.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_ROTATIONS;

        return config;
    }

    @Override
    public void setAngleDegrees(double angleDegrees) {
        pivotMotor.setPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS);
    }

    @Override
    public void goToAngleDegrees(double angleDegrees) {
        targetDegrees = angleDegrees;
        beingCommanded = true;
        DogLog.log("Subsystems/Intake/Pivot/Stopped", false);
        pivotMotor.setControl(
                motionMagicControl.withPosition(angleDegrees * Conv.DEGREES_TO_ROTATIONS));
    }

    @Override
    public void stop() {
        beingCommanded = true;
        DogLog.log("Subsystems/Intake/Pivot/Stopped", true);
        pivotMotor.setVoltage(0.0);
    }

    @Override
    public double getAngleDegrees() {
        return angleRotations.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(rps, angleRotations);
        double angleDegrees = angleRotations.getValueAsDouble() * Conv.ROTATIONS_TO_DEGREES;
        double angleRPM = rps.getValueAsDouble() * 60.0;
        DogLog.log("Subsystems/Intake/Pivot/Being Commanded Currently", beingCommanded);
        DogLog.log("Subsystems/Intake/Pivot/AngleDegrees", angleDegrees);
        DogLog.log("Subsystems/Intake/Pivot/AngleRPM", angleRPM);
        DogLog.log("Subsystems/Intake/Pivot/Target", targetDegrees);
    }
}
