package igknighters.constants;

import com.ctre.phoenix6.CANBus;

public class GeminiRobotConsts extends RobotConsts {

    public static final int firstMotorID = 15;
    public static final boolean disableAllLogs = false;
    public static final CANBus superStructure = new CANBus("SuperStructureBus");
    public static final CANBus drive = new CANBus("DriveBus");

    @Override
    public CANBus getSuperStructureBus() {
        return superStructure;
    }

    @Override
    public CANBus getDriveBus() {
        return drive;
    }

    @Override
    public boolean disableAllLogs() {
        return disableAllLogs;
    }

    @Override
    public kShooterConsts shooter() {
        return new GeminiShooterConsts();
    }

    @Override
    public kClimberConsts climber() {
        return new GeminiClimberConsts();
    }

    @Override
    public kIndexerConsts indexer() {
        return new GeminiIndexerConsts();
    }

    @Override
    public kIntakeConsts intake() {
        return new GeminiIntakeConsts();
    }

    @Override
    public kLimelightVisionConsts limelightVision() {
        return new GeminiLimelightVisionConsts();
    }

    public static class GeminiIntakeConsts implements kIntakeConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kPivotConsts kPivot() {
            return new GeminiPivotConsts();
        }

        @Override
        public kRollersConsts kRollers() {
            return new GeminiRollersConsts();
        }
    }

    public static class GeminiPivotConsts implements kPivotConsts {
        @Override
        public int MOTOR_ID() {
            return 18;
        }

        @Override
        public int CANCODER_ID() {
            return 21;
        }

        @Override
        public double GEAR_RATIO() {
            return 15.0;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 66.0;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return 0.0;
        }

        @Override
        public double MAX_SPEED_METERS_PER_SECOND() {
            return 1.0;
        }

        @Override
        public double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED() {
            return 0.5;
        }

        @Override
        public double ENCODER_OFFSET() {
            return 0.31982421875;
        }

        @Override
        public double MAX_JERK() {
            return 1.0;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 20;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 20;
        }

        @Override
        public int SUPPLY_UPPER_LIMIT() {
            return 25;
        }

        @Override
        public double kP() {
            return 45.0;
        }

        @Override
        public double kI() {
            return 0.0;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.0;
        }

        @Override
        public double kV() {
            return 0.0;
        }

        @Override
        public double kA() {
            return 0.0;
        }

        @Override
        public double JKG_M2() {
            return 0.01;
        }

        @Override
        public double LENGTH_METERS() {
            return 0.25;
        }

        @Override
        public boolean disablePivotLogs() {
            return false || disableAllLogs;
        }
    }

    public static class GeminiRollersConsts implements kRollersConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 20;
        }

        @Override
        public int FOLLOWER_MOTOR_ID() {
            return 28;
        }

        @Override
        public double WHEEL_RADIUS_METERS() {
            return 0.0508;
        }

        @Override
        public double GEAR_RATIO() {
            return 1.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 3500.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 1500.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 500.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public int FORWARD_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public int REVERSE_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 35;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 25;
        }

        @Override
        public double kP() {
            return 0.3;
        }

        @Override
        public double kI() {
            return 0.1;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.6;
        }

        @Override
        public double kV() {
            return 0.15;
        }

        @Override
        public double kA() {
            return 0.02;
        }

        @Override
        public boolean disableRollersLogs() {
            return true || disableAllLogs;
        }
    }

    public static class GeminiShooterConsts implements kShooterConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kFlywheelsConsts kFlywheels() {
            return new GeminiFlywheelsConsts();
        }

        @Override
        public kTurretConsts kTurret() {
            return new GeminiTurretConsts();
        }

        @Override
        public kHoodConsts kHood() {
            return new GeminiHoodConsts();
        }
    }

    public static class GeminiFlywheelsConsts implements kFlywheelsConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 23;
        }

        @Override
        public int FOLLOWER_MOTOR_ID() {
            return 22;
        }

        @Override
        public double WHEEL_RADIUS_METERS() {
            return 0.0508;
        }

        @Override
        public double GEAR_RATIO() {
            return 1.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 5000.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 3000.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 100.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public double kP() {
            return 0.3;
        }

        @Override
        public double kI() {
            return 0.1;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.17;
        }

        @Override
        public double kV() {
            return 0.1;
        }

        @Override
        public double kA() {
            return 0.02;
        }

        @Override
        public double ShooterHeightMeters() {
            return 0.5;
        }

        @Override
        public double PEAK_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double SUPPLY_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public boolean disableFlywheelsLogs() {
            return false;
        }
    }

    public static class GeminiTurretConsts implements kTurretConsts {
        @Override
        public int MOTOR_ID() {
            return 24;
        }

        @Override
        public double TURRET_ROBOT_DISTANCE_FROM_CENTERS() {
            return 7.0710678118655;
        }

        @Override
        public int CANCODER_ID() {
            return 25;
        }

        @Override
        public double CANCODER_OFFSET_ROTATIONS() {
            return -0.747314453125;
        }

        @Override
        public double GEAR_RATIO() {
            return 16.2;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 90.0;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return -270.0;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 600.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 800.0;
        }

        @Override
        public double MAX_JERK() {
            return 300;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public double kP() {
            return 65.0;
        }

        @Override
        public double kI() {
            return 0.05;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.0;
        }

        @Override
        public double kV() {
            return 0.0;
        }

        @Override
        public double kA() {
            return 0.0;
        }

        @Override
        public boolean disableTurretLogs() {
            return false;
        }
    }

    public static class GeminiHoodConsts implements kHoodConsts {
        @Override
        public int MOTOR_ID() {
            return 26;
        }

        @Override
        public double MOTOR_ROTS_TO_HOOD_DEGREES() {
            return 15.0;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 52.855225;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return 18.6;
        }

        @Override
        public double MAX_SPEED_R_P_S() {
            return 12.0;
        }

        @Override
        public double MAX_ACCEL_R_P_S_S() {
            return 24.0;
        }

        @Override
        public double MAX_JERK() {
            return 1.0;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 20;
        }

        @Override
        public double kP() {
            return 4.0;
        }

        @Override
        public double kI() {
            return 0.0;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.27;
        }

        @Override
        public double kV() {
            return 0.0;
        }

        @Override
        public double kA() {
            return 0.0;
        }

        @Override
        public double JKG_M2() {
            return 0.01;
        }

        @Override
        public double LENGTH_METERS() {
            return 0.25;
        }

        @Override
        public int REVERSE_LIMIT_SWITCH_ID() {
            return 9;
        }

        @Override
        public boolean disableHoodLogs() {
            return true;
        }
    }

    public static class GeminiClimberConsts implements kClimberConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kChainsawConsts kChainsaw() {
            return new GeminiChainsawConsts();
        }

        @Override
        public kServosConsts kServos() {
            return new GeminiServosConsts();
        }
    }

    public static class GeminiChainsawConsts implements kChainsawConsts {
        @Override
        public int LEFT_MOTOR_ID() {
            return 15;
        }

        @Override
        public int BUMPER_SENSOR_ID() {
            return 5;
        }

        @Override
        public int MAX_HEIGHT_SENSOR_ID() {
            return 8;
        }

        @Override
        public int MIN_HEIGHT_SENSOR_ID() {
            return 6;
        }

        @Override
        public int MIDDLE_HEIGHT_SENSOR_ID() {
            return 7;
        }

        @Override
        public boolean disableChainsawLogs() {
            return true;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.05;
        }

        @Override
        public double kP() {
            return 0.8;
        }

        @Override
        public double kI() {
            return 0.0;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.2;
        }

        @Override
        public double kG() {
            return 0.2;
        }

        @Override
        public double kV() {
            return 0.0;
        }

        @Override
        public double kA() {
            return 0.0;
        }

        @Override
        public double MAX_JERK() {
            return 0.05;
        }

        @Override
        public double MAX_VELOCITY_METERS_PER_SECOND() {
            return 0.50;
        }

        @Override
        public boolean inverted() {
            return false;
        }

        @Override
        public double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED() {
            return 0.20;
        }

        @Override
        public double GEAR_RATIO() {
            return 25.0;
        }

        @Override
        public double INCHES_TO_ROTATIONS() {
            return 1 / 4.5;
        }

        @Override
        public double ROTATIONS_TO_INCHES() {
            return 4.5;
        }

        @Override
        public double MAX_HEIGHT_INCHES() {
            return 20.0;
        }

        @Override
        public double MIN_HEIGHT_INCHES() {
            return 0.0;
        }

        @Override
        public double MIDDLE_HEIGHT_INCHES() {
            return 10.0;
        }

        @Override
        public double LENGTH_METERS() {
            return 0.5;
        }

        @Override
        public double MASS() {
            return 4.0;
        }

        @Override
        public double PEAK_FORWARD_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double PEAK_REVERSE_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public double STATOR_CURRENT_LIMIT() {
            return 50;
        }

        @Override
        public double SUPPLY_CURRENT_LIMIT() {
            return 50;
        }
    }

    public static class GeminiServosConsts implements kServosConsts {
        @Override
        public int SERVO_PORT_1() {
            return 9;
        }

        @Override
        public double MAX_ANGLE_DEGREES() {
            return 90.0;
        }

        @Override
        public double MIN_ANGLE_DEGREES() {
            return 0.0;
        }
    }

    public static class GeminiIndexerConsts implements kIndexerConsts {
        @Override
        public CANBus kCANBUS() {
            return superStructure;
        }

        @Override
        public kSpindexerConsts kSpindexer() {
            return new GeminiSpindexerConsts();
        }

        @Override
        public kExitRollersConsts kExitRollers() {
            return new GeminiExitRollersConsts();
        }
    }

    public static class GeminiSpindexerConsts implements kSpindexerConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 16;
        }

        @Override
        public double WHEEL_RADIUS_METERS() {
            return 0.0508;
        }

        @Override
        public double GEAR_RATIO() {
            return 4.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 5000.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 1500.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 600.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public int FORWARD_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public int REVERSE_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public int STATOR_CURRENT_LIMIT() {
            return 35;
        }

        @Override
        public int SUPPLY_CURRENT_LIMIT() {
            return 25;
        }

        @Override
        public int PEAK_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double kP() {
            return 0.5;
        }

        @Override
        public double kI() {
            return 0;
        }

        @Override
        public double kD() {
            return 0.001;
        }

        @Override
        public double kS() {
            return 0.178;
        }

        @Override
        public double kV() {
            return 0.5;
        }

        @Override
        public double kA() {
            return 0;
        }

        @Override
        public boolean disableSpindexerLogs() {
            return true;
        }
    }

    public static class GeminiExitRollersConsts implements kExitRollersConsts {
        @Override
        public int LEADER_MOTOR_ID() {
            return 17;
        }

        @Override
        public double GEAR_RATIO() {
            return 5.0;
        }

        @Override
        public double MOMENT_OF_INERTIA_KG_M2() {
            return 0.02;
        }

        @Override
        public double MAX_SPEED_RPM() {
            return 5000.0;
        }

        @Override
        public double MAX_ACCELERATION_RPM() {
            return 1500.0;
        }

        @Override
        public double MOTION_MAGIC_JERK() {
            return 5000.0;
        }

        @Override
        public int BEAM_BREAK_SENSOR_CHANNEL() {
            return 0;
        }

        @Override
        public double kP() {
            return 0.3;
        }

        @Override
        public double kI() {
            return 0.1;
        }

        @Override
        public double kD() {
            return 0.0;
        }

        @Override
        public double kS() {
            return 0.3;
        }

        @Override
        public double kV() {
            return 0.1;
        }

        @Override
        public double kA() {
            return 0.02;
        }

        @Override
        public double PEAK_CURRENT_LIMIT() {
            return 40;
        }

        @Override
        public double SUPPLY_CURRENT_LIMIT() {
            return 30;
        }

        @Override
        public boolean disableExitRollersLogs() {
            return true;
        }
    }

    public static class GeminiLimelightVisionConsts implements kLimelightVisionConsts {
        @Override
        public String turretCam() {
            return "limelight";
        }

        @Override
        public String intakeCam() {
            return "limelight-intake";
        }

        @Override
        public String backCam() {
            return "limelight-back";
        }

        @Override
        public String rightCam() {
            return "limelight-left";
        }

        @Override
        public boolean disableVisionLogs() {
            return true;
        }
    }
}
