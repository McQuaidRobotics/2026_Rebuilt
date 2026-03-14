package igknighters.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake;
import igknighters.util.log.Log;

public class RollersReal extends Rollers {
    private final TalonFX intakeMotor =
            new TalonFX(kIntake.kRollers.LEADER_MOTOR_ID, kIntake.CANBUS);
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
        config.CurrentLimits.StatorCurrentLimit = 35.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = kIntake.kRollers.GEAR_RATIO;

        return config;
    }

    @Override
    public AngularVelocity getSpeed() {
        return intakeMotor.getVelocity().getValue();
    }

    @Override
    public void goToSpeed(AngularVelocity speed) {
        intakeMotor.setControl(velocityContorl.withVelocity(speed.in(RotationsPerSecond)));
    }

    @Override
    public void stop() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {

        if (!SubsystemConstants.kIntake.kRollers.disableRollersLogs) {
            Log.log("ROBOT/Subsystems/Intake/Rollers/SpeedRPS", getSpeed());
        }
    }
}
