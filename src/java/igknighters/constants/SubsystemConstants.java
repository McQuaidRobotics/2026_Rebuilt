package igknighters.constants;

public class SubsystemConstants {

    public static final int firstMotorID = 15;

    public static class kClimber {
        public static class kChainsaw {
            public static final int LEFT_MOTOR_ID = 15;
            public static final int BUMPER_SENSOR_ID = 0;
            public static final int MAX_HEIGHT_SENSOR_ID = 1;
            public static final int MIN_HEIGHT_SENSOR_ID = 2;

            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.05; // this is made up
            public static final double kP = 0.8;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.2;
            public static final double kG = 0.2;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double MAX_JERK = 0.05;
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.50;
            public static final boolean inverted = false;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20;
            public static final double GEAR_RATIO = 25.0;
            public static final double INCHES_TO_ROTATIONS =
                    1 / 4.5; // 1 rot at end of gearbox is 4.5 inches of linear movement
            public static final double ROTATIONS_TO_INCHES = 4.5;
            public static final double MAX_HEIGHT_INCHES = 20.0;
            public static final double MIN_HEIGHT_INCHES = 0.0;
            public static final double LENGTH_METERS = 0.5;
            public static final double MASS = 4.0;
            public static final double PEAK_FORWARD_CURRENT_LIMIT = 40;
            public static final double PEAK_REVERSE_CURRENT_LIMIT = 30;
            public static final double STATOR_CURRENT_LIMIT = 35;
            public static final double SUPPLY_CURRENT_LIMIT = 25;
        }

        public static class kServos {
            public static final int SERVO_PORT_1 = 2;
            public static final int SERVO_PORT_2 = 3;
            public static final int SERVO_PORT_3 = 4;
            public static final int SERVO_PORT_4 = 5;
            public static final double MAX_ANGLE_DEGREES = 180.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
        }
    }

    public static class kIndexer {
        public static class kSpindexer {
            public static final int LEADER_MOTOR_ID = 18;
            public static final int FOLLOWER_MOTOR_ID = 19;
            public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
            public static final double GEAR_RATIO = 5.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 70.0;
            public static final double MOTION_MAGIC_JERK = 17.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final int FORWARD_CURRENT_LIMIT = 40;
            public static final int REVERSE_CURRENT_LIMIT = 30;
            public static final int STATOR_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int PEAK_CURRENT_LIMIT = 40;
            public static final double kP = 0.5;
            public static final double kI = 0;
            public static final double kD = 0.001;
            public static final double kS = 0.178;
            public static final double kV = 0.5;
            public static final double kA = 0;
        }

        public static class kExitRollers {
            public static final int LEADER_MOTOR_ID = 50;
            public static final double GEAR_RATIO = 5.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 70.0;
            public static final double MOTION_MAGIC_JERK = 17.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.3;
            public static final double kV = 0.1;
            public static final double kA = 0.02;
            public static final double PEAK_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 30;
        }
    }

    public static class kIntake {
        public static class kRollers {
            // will be configured such that + voltage will intake game pieces
            public static final int LEADER_MOTOR_ID = 20;
            public static final int FOLLOWER_MOTOR_ID = 21;
            public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
            public static final double GEAR_RATIO = 1.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 70.0;
            public static final double MOTION_MAGIC_JERK = 17.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final int FORWARD_CURRENT_LIMIT = 40;
            public static final int REVERSE_CURRENT_LIMIT = 30;
            public static final int STATOR_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.6;
            public static final double kV = 0.15;
            public static final double kA = 0.02;
        }

        public static class kPivot {
            public static final int MOTOR_ID = 22;
            public static final int CANCODER_ID = 23;
            public static final double GEAR_RATIO = 15.0;
            public static final double MAX_ANGLE_DEGREES = 90.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double MAX_SPEED_METERS_PER_SECOND = 1.0;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .5;
            public static final double MAX_JERK = 1.0;
            public static final int STATOR_CURRENT_LIMIT = 30;
            public static final int SUPPLY_CURRENT_LIMIT = 20;
            public static final double kP = 0.2;
            public static final double kI = 0.0;
            public static final double kD = 0.05;
            public static final double kS = 0.1;
            public static final double kV = 0.02;
            public static final double kA = 0.005;
            public static final double JKG_M2 = 0.01;
            public static final double LENGTH_METERS =
                    .25; // distance from central shaft to edge of wrist
            public static final int REVERSE_LIMIT_SWITCH_ID = 6;
        }
    }

    public static class kShooter {
        public static class kRollers {
            public static final int LEADER_MOTOR_ID = 24;
            public static final int FOLLOWER_MOTOR_ID = 25;
            public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inch radius
            public static final double GEAR_RATIO = 1.0;
            public static final double MOMENT_OF_INERTIA_KG_M2 = 0.02;
            public static final double MAX_SPEED_RPM = 5000.0;
            public static final double MAX_ACCELERATION_RPM = 70.0;
            public static final double MOTION_MAGIC_JERK = 17.0;
            public static final int BEAM_BREAK_SENSOR_CHANNEL = 0;
            public static final double kP = 0.3; // .5 max
            public static final double kI = 0.1;
            public static final double kD = 0.0;
            public static final double kS = 0.3;
            public static final double kV = 0.1;
            public static final double kA = 0.02;
            public static final double ShooterHeightMeters =
                    .3; // 30 cm this is made up it will be off ground though
            public static final double PEAK_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 30;
        }

        public static class kTurret {
            public static final int MOTOR_ID = 26;
            public static final int CANCODER_ID = 27;
            public static final double CANCODER_OFFSET_ROTATIONS = 0.0;
            public static final double GEAR_RATIO = 12.0;
            public static final double MAX_ANGLE_DEGREES = 180.0;
            public static final double MIN_ANGLE_DEGREES = -180.0;
            public static final double MAX_SPEED_RPM = 90.0;
            public static final double MAX_ACCELERATION_RPM = 90.0;
            public static final double MAX_JERK = 10;
            public static final int STATOR_CURRENT_LIMIT = 40;
            public static final int SUPPLY_CURRENT_LIMIT = 30;
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.1;
            public static final double kS = 0.2;
            public static final double kV = 0.05;
            public static final double kA = 0.01;
        }

        public static class kHood {
            public static final int MOTOR_ID = 28;
            public static final int CANCODER_ID = 29;
            public static final double GEAR_RATIO = 10.0;
            public static final double MAX_ANGLE_DEGREES = 60.0;
            public static final double MIN_ANGLE_DEGREES = 0.0;
            public static final double MAX_SPEED_RPM = 60.0;
            public static final double MAX_ACCELERATION_RPM = 6.0;
            public static final double MAX_JERK = 1.0;
            public static final int STATOR_CURRENT_LIMIT = 30;
            public static final int SUPPLY_CURRENT_LIMIT = 20;
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.05;
            public static final double kS = 0.1;
            public static final double kV = 0.02;
            public static final double kA = 0.005;
            public static final double JKG_M2 = 0.01;
            public static final double LENGTH_METERS =
                    .25; // distance from central shaft to edge of flap
            public static final int REVERSE_LIMIT_SWITCH_ID = 7;
        }
    }

    public static class kLimelightVision {
        public static final String frontLeft = "limelight-fl";
        public static final String frontRight = "limelight-fr";
        public static final String backLeft = "limelight-bl";
        public static final String backRight = "limelight-br";
    }
}
