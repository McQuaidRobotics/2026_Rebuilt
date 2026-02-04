package igknighters.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import igknighters.commands.Repulsor.obstacle;
import igknighters.commands.Repulsor.obstacleType;
import java.util.ArrayList;
import java.util.Arrays;

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

    public static class OBSTACLES {
        public static final obstacle BUMP_BLUE =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                158.32 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        23.5 * Conv.INCHES_TO_METERS,
                        109 * Conv.INCHES_TO_METERS,
                        obstacleType.SQUARE);
        public static final obstacle BUMP_RED =
                new obstacle(
                        new Pose2d(
                                LENGTH - 182.11 * Conv.INCHES_TO_METERS,
                                158.32 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        23.5 * Conv.INCHES_TO_METERS,
                        109 * Conv.INCHES_TO_METERS,
                        obstacleType.SQUARE);
        public static final obstacle WALL_DS_BLUE =
                new obstacle(
                        new Pose2d(0, 158.32 * Conv.INCHES_TO_METERS, new Rotation2d()),
                        1,
                        1,
                        WIDTH / 2,
                        obstacleType.SQUARE);
        public static final obstacle WALL_BLUE_TO_RED_BOTTOM =
                new obstacle(
                        new Pose2d(LENGTH / 2, 0, new Rotation2d()),
                        1,
                        WIDTH / 2,
                        1,
                        obstacleType.SQUARE);
        public static final obstacle WALL_BLUE_TO_RED_TOP =
                new obstacle(
                        new Pose2d(LENGTH / 2, WIDTH, new Rotation2d()),
                        1,
                        WIDTH / 2,
                        1,
                        obstacleType.SQUARE);
        public static final obstacle WALL_DS_RED =
                new obstacle(
                        new Pose2d(LENGTH, 158.32 * Conv.INCHES_TO_METERS, new Rotation2d()),
                        1,
                        1,
                        WIDTH / 2,
                        obstacleType.SQUARE);
        public static final ArrayList<obstacle> ALL_OBSTACLES =
                new ArrayList<>(
                        Arrays.asList(
                                BUMP_BLUE,
                                BUMP_RED,
                                WALL_DS_BLUE,
                                WALL_BLUE_TO_RED_BOTTOM,
                                WALL_BLUE_TO_RED_TOP,
                                WALL_DS_RED));
    }

    public static final double WIDTH = 316.64 * Conv.INCHES_TO_METERS; // meters
    public static final double LENGTH = 650.12 * Conv.INCHES_TO_METERS; // meters
}
