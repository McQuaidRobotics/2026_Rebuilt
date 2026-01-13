package igknighters.subsystems.shooter.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.constants.SubsystemConstants;

public class RollersReal extends Rollers {
    private TalonFX motor;
    private MotionMagicVelocityVoltage voltageControl;
    private final StatusSignal<AngularVelocity> motorVelocity;
    public RollersReal(){
        motor = new TalonFX(SubsystemConstants.Shooter.LEADER_MOTOR_ID);
        motorVelocity = motor.getVelocity();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = SubsystemConstants.Shooter.kP;
        config.Slot0.kI = SubsystemConstants.Shooter.kI;
        config.Slot0.kD = SubsystemConstants.Shooter.kD;
        config.Slot0.kS = SubsystemConstants.Shooter.kS;
        config.Slot0.kV = SubsystemConstants.Shooter.kV;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Shooter.MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration = SubsystemConstants.Shooter.MAX_ACCELERATION_RPM;
        
        motor.getConfigurator().apply(config);
    }
    
    @Override
    public void setSpeed(double speedMetersPerSecond) {
        
        motor.setControl(voltageControl.withVelocity(speedMetersPerSecond));

    }

    @Override
    public void setVoltage(double voltage) {
            motor.setVoltage(0);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorVelocity);
        DogLog.log("subsystems/shooter", motorVelocity.getValueAsDouble());

    }
}
