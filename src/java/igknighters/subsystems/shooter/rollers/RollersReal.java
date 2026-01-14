package igknighters.subsystems.shooter.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import igknighters.constants.SubsystemConstants;

public class RollersReal extends Rollers {
    private final TalonFX mainShooter = new TalonFX(SubsystemConstants.Shooter.LEADER_MOTOR_ID);

    // private final MotionMagicVelocityVoltage velocityControl = new
    // MotionMagicVelocityVoltage(0.0);

    private final MotionMagicVelocityVoltage velocityControl;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // private final DigitalInput beamBreakSensor = new
    // DigitalInput(SubsystemConstants.Shooter.BEAM_BREAK_SENSOR_CHANNEL);

    private BaseStatusSignal shooterVelocity;
    private BaseStatusSignal shooterCurrent;
    private BaseStatusSignal shooterVoltage;
    private BaseStatusSignal shooterTemperature;

    // private BaseStatusSignal isBeamBreakTripped;

    public RollersReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.Shooter.kP;
        config.Slot0.kI = SubsystemConstants.Shooter.kI;
        config.Slot0.kD = SubsystemConstants.Shooter.kD;
        config.Slot0.kS = SubsystemConstants.Shooter.kS;
        config.Slot0.kV = SubsystemConstants.Shooter.kV;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.Shooter.GEAR_RATIO;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Shooter.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration =
                SubsystemConstants.Shooter.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.Shooter.MAX_SPEED_RPM;

        mainShooter.getConfigurator().apply(config);

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = mainShooter.getVelocity();
        shooterCurrent = mainShooter.getSupplyCurrent();
        shooterVoltage = mainShooter.getSupplyVoltage();
        shooterTemperature = mainShooter.getDeviceTemp();
    }

    @Override
    public void setSpeed(double speedRpm) {
        DogLog.log("Subsystems/Shooter/setSpeed", speedRpm);
        mainShooter.setControl(velocityControl.withVelocity(speedRpm / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        mainShooter.setControl(dutyCycleControl.withOutput(voltage / 12.0));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                shooterVelocity, shooterCurrent, shooterVoltage, shooterTemperature);
        DogLog.log("Subsystems/Shooter/velocity", shooterVelocity.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/current", shooterCurrent.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/voltage", shooterVoltage.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/temperature", shooterTemperature.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/periodicing", true);
    }
}
