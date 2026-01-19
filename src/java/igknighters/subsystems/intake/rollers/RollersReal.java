package igknighters.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import igknighters.constants.SubsystemConstants.kIntake;

public class RollersReal extends Rollers {
    private final TalonFX intakeMotor = new TalonFX(kIntake.kRollers.LEADER_MOTOR_ID);
    private final MotionMagicVelocityVoltage velocityContorl =
            new MotionMagicVelocityVoltage(0.0).withSlot(0);
    private BaseStatusSignal intakeSpeed;

    public RollersReal() {
        intakeMotor.getConfigurator().apply(getLeaderConfig());
        intakeSpeed = intakeMotor.getVelocity();
    }

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kIntake.kRollers.kP;
        config.Slot0.kI = kIntake.kRollers.kI;
        config.Slot0.kD = kIntake.kRollers.kD;

        config.MotionMagic.MotionMagicJerk = kIntake.kRollers.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicCruiseVelocity = kIntake.kRollers.MAX_SPEED_RPM;
        config.MotionMagic.MotionMagicAcceleration = kIntake.kRollers.MAX_ACCELERATION_RPM;

        config.TorqueCurrent.PeakForwardTorqueCurrent = kIntake.kRollers.FORWARD_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = kIntake.kRollers.REVERSE_CURRENT_LIMIT;

        config.Feedback.SensorToMechanismRatio = kIntake.kRollers.GEAR_RATIO;

        return config;
    }

    @Override
    public double getSpeedRPM() {
        return intakeSpeed.getValueAsDouble();
    }

    @Override
    public void goToSpeedRPM(double speedRPS) {
        intakeMotor.setControl(velocityContorl.withVelocity(speedRPS));
    }

    @Override
    public void stop() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(intakeSpeed);
        DogLog.log("Subsystems/Intake/Rollers/SpeedRPS", getSpeedRPM());
    }
}
