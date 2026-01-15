package igknighters.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {
    public static class HUB {
        public static final double HEIGHT_METERS = 72.0 * Conv.INCHES_TO_METERS;
        public static final Pose2d POSITION =
                new Pose2d(
                        158.84 * Conv.INCHES_TO_METERS,
                        181.56 * Conv.INCHES_TO_METERS,
                        new Rotation2d());
        public static final Pose3d POSE3D =
                new Pose3d(POSITION.getX(), POSITION.getY(), HEIGHT_METERS, new Rotation3d());
    }
    
    public static class PASS {

    }
}
