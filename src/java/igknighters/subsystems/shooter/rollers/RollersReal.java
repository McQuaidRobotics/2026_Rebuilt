package igknighters.subsystems.shooter.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import igknighters.constants.SubsystemConstants;

public class RollersReal extends Rollers {
    private final TalonFX mainShooter = new TalonFX(SubsystemConstants.Shooter.LEADER_MOTOR_ID);

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0);

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // private final DigitalInput beamBreakSensor = new DigitalInput(SubsystemConstants.Shooter.BEAM_BREAK_SENSOR_CHANNEL);

    private BaseStatusSignal shooterVelocity;
    private BaseStatusSignal shooterCurrent;
    private BaseStatusSignal shooterVoltage;
    private BaseStatusSignal shooterTemperature;
    private BaseStatusSignal isBeamBreakTripped;
    public RollersReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.Shooter.kP;
        config.Slot0.kI = SubsystemConstants.Shooter.kI;
        config.Slot0.kD = SubsystemConstants.Shooter.kD;
        config.Slot0.kS = SubsystemConstants.Shooter.kS;
        config.Slot0.kV = SubsystemConstants.Shooter.kV;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Shooter.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration = SubsystemConstants.Shooter.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.Shooter.MAX_SPEED_RPM;

        mainShooter.getConfigurator().apply(config);

        shooterVelocity = mainShooter.getVelocity();
        shooterCurrent = mainShooter.getSupplyCurrent();
        shooterVoltage = mainShooter.getSupplyVoltage();
        shooterTemperature = mainShooter.getDeviceTemp();


    }
    @Override
    public void setSpeed(double speedMetersPerSecond) {
        mainShooter.setControl(velocityControl.withVelocity(speedMetersPerSecond));

    }

    @Override
    public void setVoltage(double voltage) {
        mainShooter.setControl(dutyCycleControl.withOutput(voltage/12.0));

    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(shooterVelocity, shooterCurrent, shooterVoltage, shooterTemperature);
        DogLog.log("subsystems/shooter/velocity", shooterVelocity.getValueAsDouble());
        DogLog.log("subsystems/shooter/current", shooterCurrent.getValueAsDouble());
        DogLog.log("subsystems/shooter/voltage", shooterVoltage.getValueAsDouble());
        DogLog.log("subsystems/shooter/temperature", shooterTemperature.getValueAsDouble());

    }
}
