package igknighters.constants;

import dev.doglog.DogLog;
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
                        new Rotation2d(Math.PI)); // made up value
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

    public static class BUMP {

        public static final double HALF_WIDTH_METERS = 23.5 * Conv.INCHES_TO_METERS;
        public static final double HALF_HEIGHT_METERS = 109 * Conv.INCHES_TO_METERS;

        public static final double BUMP_1_X_METERS = 182.11 * Conv.INCHES_TO_METERS;
        public static final double BUMP_2_X_METERS = LENGTH - (182.11) * Conv.INCHES_TO_METERS;

        public static final double BUMP_1_Y_METERS = 158.32 * Conv.INCHES_TO_METERS;
        public static final double BUMP_2_Y_METERS = 158.32 * Conv.INCHES_TO_METERS;

        public static enum PROTECTION_MOVEMENT {
            GO_UP,
            GO_DOWN,
            YOU_CHILLIN_IN_THE_MIDDLE
        }

        public static boolean isInside(Pose2d pose) {
            double x = pose.getX();
            double y = pose.getY();
            DogLog.log("Commands/BumpProtection: x;", x);
            DogLog.log("Commands/BumpProtection: y;", y);

            // Bump 1
            if (x >= BUMP_1_X_METERS - HALF_WIDTH_METERS
                    && x <= BUMP_1_X_METERS + HALF_WIDTH_METERS) {
                if (y >= BUMP_1_Y_METERS - HALF_HEIGHT_METERS
                        && y <= BUMP_1_Y_METERS + HALF_HEIGHT_METERS) {
                    DogLog.log("Commands/BumpProtection: inside bump 1", true);
                    return true;
                }
            }

            // Bump 2
            if (x >= BUMP_2_X_METERS - HALF_WIDTH_METERS
                    && x <= BUMP_2_X_METERS + HALF_WIDTH_METERS) {
                if (y >= BUMP_2_Y_METERS - HALF_HEIGHT_METERS
                        && y <= BUMP_2_Y_METERS + HALF_HEIGHT_METERS) {
                    DogLog.log("Commands/BumpProtection: inside bump 2", true);
                    return true;
                }
            }
            DogLog.log("Commands/BumpProtection: inside bump 2", false);
            DogLog.log("Commands/BumpProtection: inside bump 1", false);

            return false;
        }

        public static boolean isAboutToFallOff(Pose2d pose, double acceptable_closeness) {
            if (!isInside(pose)) {
                return false;
            }
            double x = pose.getX();
            double y = pose.getY();

            if (Math.abs(y - BUMP_1_Y_METERS - HALF_HEIGHT_METERS) < acceptable_closeness) {
                return true;
            }
            if (Math.abs(y - BUMP_1_Y_METERS + HALF_HEIGHT_METERS) < acceptable_closeness) {
                return true;
            }
            return false;
        }

        /**
         * Assumes the robot is inside the bump, returns which direction to go to get out of the
         * bump. The direction is relative to the blue aliance so left is +y and right is -y
         *
         * @param pose
         * @return
         */
        public static PROTECTION_MOVEMENT getProtectionMovement(
                Pose2d pose, double acceptable_closeness) {
            if (!isInside(pose)) {
                return null;
            }
            double y = pose.getY();
            DogLog.log(
                    "Commands/Bump Protection/get protection movement/dTop",
                    Math.abs(y - (BUMP_1_Y_METERS + HALF_HEIGHT_METERS)));
            DogLog.log(
                    "Commands/Bump Protection/get protection movement/dBottom",
                    Math.abs(y - (BUMP_1_Y_METERS - HALF_HEIGHT_METERS)));

            if (Math.abs(y - (BUMP_1_Y_METERS + HALF_HEIGHT_METERS)) < acceptable_closeness) {
                return PROTECTION_MOVEMENT.GO_DOWN;
            } else if (Math.abs(y - (BUMP_1_Y_METERS - HALF_HEIGHT_METERS))
                    < acceptable_closeness) {
                return PROTECTION_MOVEMENT.GO_UP;
            } else {
                return PROTECTION_MOVEMENT
                        .YOU_CHILLIN_IN_THE_MIDDLE; // the robot is in the middle of the bump so it
                // can go either way
            }
        }
    }
}
