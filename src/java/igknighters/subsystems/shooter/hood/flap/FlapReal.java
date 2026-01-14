package igknighters.subsystems.shooter.hood.flap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;

public class FlapReal extends Flap {

    public TalonFXConfiguration flapConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = SubsystemConstants.Hood.kP;
        config.Slot0.kI = SubsystemConstants.Hood.kI;
        config.Slot0.kD = SubsystemConstants.Hood.kD;
        config.Slot0.kS = SubsystemConstants.Hood.kS;
        config.Slot0.kV = SubsystemConstants.Hood.kV;
        config.Slot0.kA = SubsystemConstants.Hood.kA;

        config.MotionMagic.MotionMagicJerk = SubsystemConstants.Hood.MAX_JERK;
        config.MotionMagic.MotionMagicAcceleration = SubsystemConstants.Hood.MAX_ACCELERATION_RPM;
        config.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.Hood.MAX_SPEED_RPM;

        config.Feedback.SensorToMechanismRatio = SubsystemConstants.Hood.GEAR_RATIO;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = SubsystemConstants.Hood.REVERSE_LIMIT_SWITCH_ID;


        return config;
    }
}
