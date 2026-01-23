package igknighters.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {
    public static class HUB {
        public static final double HEIGHT_METERS = 72.0 * Conv.INCHES_TO_METERS;
        public static final Pose2d POSITION_BLUE =
                new Pose2d(
                        181.56 * Conv.INCHES_TO_METERS,
                        158.32 * Conv.INCHES_TO_METERS,
                        new Rotation2d());
        public static final Pose3d POSE3D_BLUE =
                new Pose3d(
                        POSITION_BLUE.getX(),
                        POSITION_BLUE.getY(),
                        HEIGHT_METERS,
                        new Rotation3d());

        public static final Pose2d POSITION_RED =
                new Pose2d(LENGTH - POSITION_BLUE.getX(), POSITION_BLUE.getY(), new Rotation2d());
        public static final Pose3d POSE3D_RED =
                new Pose3d(
                        POSITION_RED.getX(), POSITION_RED.getY(), HEIGHT_METERS, new Rotation3d());
    }

    public static class CLIMB {
        public static final Pose2d POSITION_BLUE =
                new Pose2d(
                        33.0 * Conv.INCHES_TO_METERS,
                        158.32 * Conv.INCHES_TO_METERS,
                        new Rotation2d());
        public static final Pose2d POSITION_RED =
                new Pose2d(LENGTH - POSITION_BLUE.getX(), POSITION_BLUE.getY(), new Rotation2d());
    }

    public static class PASS {
        public static final Pose3d POSITION_RIGHT_BLUE =
                new Pose3d(
                        0.0 * Conv.INCHES_TO_METERS,
                        0.0 * Conv.INCHES_TO_METERS,
                        0.0 * Conv.INCHES_TO_METERS,
                        new Rotation3d());

        public static final Pose3d POSITION_LEFT_BLUE =
                new Pose3d(0.0, FieldConstants.WIDTH, 0.0, new Rotation3d());

        public static final Pose3d POSITION_RIGHT_RED =
                new Pose3d(FieldConstants.LENGTH - 0.0, 0.0, 0.0, new Rotation3d());

        public static final Pose3d POSITION_LEFT_RED =
                new Pose3d(FieldConstants.LENGTH, FieldConstants.WIDTH, 0.0, new Rotation3d());
    }

    public static final double WIDTH = 316.64 * Conv.INCHES_TO_METERS; // meters
    public static final double LENGTH = 650.12 * Conv.INCHES_TO_METERS; // meters
    public static final double ALIANCE_ZONE_BLUE = 181.56 * Conv.INCHES_TO_METERS; // meters
    public static final double ALIANCE_ZONE_RED = LENGTH - ALIANCE_ZONE_BLUE;
}
