package igknighters.constants;

public class SubsystemConstants {
    public static class Elevator {
        public static final double MAX_HEIGHT_METERS = 10.0;
        public static final double MIN_HEIGHT_METERS = 0.0;
        public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
        public static final int LEADER_MOTOR_ID = 10;
        public static final int FOLLOWER_MOTOR_ID = 11;
        public static final int REVERSE_LIMIT_REMOTE_SENSOR_ID = 5;
        public static final double HEIGHT_TOLERANCE_METERS = 0.01;
        public static final double GEAR_RATIO = 10.0;
        public static final double CARRIAGE_MASS_KG = 5.0;
        public static final double DRUM_RADIUS_METERS = 0.0254; // 1 inch radius
        public static final double kP = 1.0;
        public static final double kI = 0.2;
        public static final double kD = 0.1;
        public static final double kS = 0.1;
        public static final double kG = 0.3;
        public static final double kV = 0.1;
        public static final double kA = 0.1;
    }

    public static class kClimber {
        public static final int LEFT_MOTOR_ID = 60;
        public static final int RIGHT_MOTOR_ID = 61;
        public static final double MOMENT_OF_INERTIA_KG_M2 = 0.05; // this is made up
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double MAX_JERK = 0.05;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.50;
        public static final boolean inverted = false;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20;
        public static final double GEAR_RATIO = 25.0;
        public static final double INCHES_TO_ROTATIONS = 4.0; // 25 motor rots = 4 inches so when comanding the motor we can convert
        public static final double MAX_HEIGHT_INCHES = 20.0 * Conv.INCHES_TO_METERS;
        public static final double MIN_HEIGHT_INCHES = 0.0;
        public static final double LENGTH_METERS = 0.5;
        public static final double MASS = 4.0;
        public static final double PEAK_FORWARD_CURRENT_LIMIT = 40;
        public static final double PEAK_REVERSE_CURRENT_LIMIT = 30;
        public static final double STATOR_CURRENT_LIMIT = 35;
        public static final double SUPPLY_CURRENT_LIMIT = 25;
    }

    public static class kIndexer {
        public static final int LEADER_MOTOR_ID = 50;
        public static final int FOLLOWER_MOTOR_ID = 22;
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

    public static class kIntake {
        public static class kRollers {
            // will be configured such that + voltage will intake game pieces
            public static final int LEADER_MOTOR_ID = 52;
            public static final int FOLLOWER_MOTOR_ID = 22;
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
            public static final int MOTOR_ID = 32;
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
            public static final int LEADER_MOTOR_ID = 50;
            public static final int FOLLOWER_MOTOR_ID = 51;
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
            public static final int MOTOR_ID = 52;
            public static final int CANCODER_ID = 53;
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
            public static final int MOTOR_ID = 53;
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
