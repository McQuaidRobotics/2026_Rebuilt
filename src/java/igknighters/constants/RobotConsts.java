package igknighters.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.util.LerpTable;
import yams.motorcontrollers.SmartMotorControllerConfig;

public abstract class RobotConsts {
    public abstract CANBus getSuperStructureBus();

    public abstract SWERVE_CONSTS swerve();

    public abstract boolean disableAllLogs();

    public abstract kShooterConsts shooter();

    public abstract kIndexerConsts indexer();

    public abstract kIntakeConsts intake();

    public abstract kLerpConsts lerp();

    public abstract kLimelightVisionConsts limelightVision();

    public interface SWERVE_CONSTS {
        CommonSwerveConsts getCommonSwerveConsts();
    }

    public interface kShooterConsts {
        CANBus kCANBUS();

        kFlywheelsConsts kFlywheels();

        kTurretConsts kTurret();

        kHoodConsts kHood();
    }

    public interface kLerpConsts {
        kRPMConsts kRPM();

        kHoodAngleConsts kHoodAngle();

        kRadialSOTMConsts kRadialSOTM();

        kTOFConsts kTimeOfFlight();

        kTangentialSOTMConsts kTangentialSOTM();
    }

    public interface kTOFConsts {
        LerpTable table();
    }

    public interface kRPMConsts {
        LerpTable table();
    }

    public interface kHoodAngleConsts {
        LerpTable table();
    }

    public interface kRadialSOTMConsts {
        LerpTable away();

        LerpTable towards();
    }

    public interface kTangentialSOTMConsts {
        LerpTable table();
    }

    public interface kFlywheelsConsts {

        SmartMotorControllerConfig getConfig(Subsystem subsystem, TalonFX follower);

        int LEADER_MOTOR_ID();

        int FOLLOWER_MOTOR_ID();

        double WHEEL_RADIUS_METERS();

        double ShooterHeightMeters();

        boolean disableFlywheelsLogs();

        default double RPM_TO_METERS_PER_SECOND_FACTOR() {
            return (2.0 * Math.PI * WHEEL_RADIUS_METERS()) / 60.0;
        }
    }

    public interface kTurretConsts {
        int MOTOR_ID();

        SmartMotorControllerConfig getConfig(Subsystem subsystem, CANcoder caNcoder);

        double TURRET_ROBOT_DISTANCE_FROM_CENTERS();

        int CANCODER_ID();

        double GEAR_RATIO();

        double MAX_ANGLE_DEGREES();

        double MIN_ANGLE_DEGREES();

        boolean disableTurretLogs();
    }

    public interface kHoodConsts {
        int MOTOR_ID();

        double MOTOR_ROTS_TO_HOOD_DEGREES();

        double MOTOR_ROTS_TO_HOOD_ROTS();

        SmartMotorControllerConfig getConfig(Subsystem subsystem);

        double MIN_ANGLE_DEGREES();

        double MAX_ANGLE_DEGREES();

        double LENGTH_METERS();

        int REVERSE_LIMIT_SWITCH_ID();

        boolean disableHoodLogs();
    }

    public interface kIndexerConsts {
        CANBus kCANBUS();

        kSpindexerConsts kSpindexer();

        kExitRollersConsts kExitRollers();
    }

    public interface kSpindexerConsts {
        int LEADER_MOTOR_ID();

        SmartMotorControllerConfig getConfig(Subsystem subsystem);

        double WHEEL_RADIUS_METERS();

        double GEAR_RATIO();

        double MOMENT_OF_INERTIA_KG_M2();

        double MAX_SPEED_RPM();

        double MAX_ACCELERATION_RPM();

        double MOTION_MAGIC_JERK();

        int BEAM_BREAK_SENSOR_CHANNEL();

        int FORWARD_CURRENT_LIMIT();

        int REVERSE_CURRENT_LIMIT();

        int STATOR_CURRENT_LIMIT();

        int SUPPLY_CURRENT_LIMIT();

        int PEAK_CURRENT_LIMIT();

        double kP();

        double kI();

        double kD();

        double kS();

        double kV();

        double kA();

        boolean disableSpindexerLogs();
    }

    public interface kIntakeConsts {
        CANBus kCANBUS();

        kPivotConsts kPivot();

        kRollersConsts kRollers();
    }

    public interface kPivotConsts {
        int MOTOR_ID();

        SmartMotorControllerConfig getConfig(CANcoder caNcoder, Subsystem subsystem);

        int CANCODER_ID();

        double PARTIAL_STOW();

        double JORK_ANGLE();

        double MAX_ANGLE_DEGREES();

        double MIN_ANGLE_DEGREES();

        double STOWED_ANGLE_DEGREES();

        double ENCODER_OFFSET();

        boolean disablePivotLogs();
    }

    public interface kRollersConsts {
        int LEADER_MOTOR_ID();

        int FOLLOWER_MOTOR_ID();

        SmartMotorControllerConfig getConfig(Subsystem subsystem);

        boolean disableRollersLogs();
    }

    public interface kExitRollersConsts {
        int LEADER_MOTOR_ID();

        SmartMotorControllerConfig getConfig(Subsystem subsystem);

        double GEAR_RATIO();

        double MOMENT_OF_INERTIA_KG_M2();

        double MAX_SPEED_RPM();

        double MAX_ACCELERATION_RPM();

        double MOTION_MAGIC_JERK();

        int BEAM_BREAK_SENSOR_CHANNEL();

        double kP();

        double kI();

        double kD();

        double kS();

        double kV();

        double kA();

        double PEAK_CURRENT_LIMIT();

        double SUPPLY_CURRENT_LIMIT();

        boolean disableExitRollersLogs();
    }

    public interface kLimelightVisionConsts {
        String turretCam();

        String intakeCam();

        String backCam();

        String rightCam();

        boolean disableVisionLogs();
    }
}
