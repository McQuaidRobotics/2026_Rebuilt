package igknighters.constants;

public class SubsystemConstants {
    public static class Elevator {
        public static final double MAX_HEIGHT_METERS = 2.0;
        public static final double MIN_HEIGHT_METERS = 0.0;
        public static final double MAX_SPEED_METERS_PER_SECOND = 1.0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
        public static final int LEADER_MOTOR_ID = 10;
        public static final int FOLLOWER_MOTOR_ID = 11;
        public static final int REVERSE_LIMIT_REMOTE_SENSOR_ID = 5;
        public static final double HEIGHT_TOLERANCE_METERS = 0.01;
        public static final double GEAR_RATIO = 10.0;
        public static final double CARRIAGE_MASS_KG = 5.0;
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.2;
        public static final double kG = 0.5;
        public static final double kV = 1.0;
    }
}
