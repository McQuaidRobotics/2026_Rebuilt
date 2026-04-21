package igknighters.util.Vision.REAL;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import java.util.Optional;

public class AprilTagTracker {
    private final String cameraName;

    public AprilTagTracker(String cameraName) {
        this.cameraName = cameraName;
    }

    /**
     * Gets the location of a specific AprilTag relative to the robot.
     *
     * @param targetID The ID of the AprilTag to look for.
     * @return Optional Translation2d (X is forward, Y is left). Empty if the tag isn't visible or
     *     the ID doesn't match.
     */
    public Optional<Translation3d> getTranslationToTag(int targetID) {
        // 1. Check if we see the correct target
        if (!LimelightHelpers.getTV(cameraName)
                || (int) LimelightHelpers.getFiducialID(cameraName) != targetID) {
            return Optional.empty();
        }

        /* * 2. Get the Target Pose in Robot Space.
         * This returns where the TAG is relative to the ROBOT.
         * Limelight returns this as an array: [x, y, z, roll, pitch, yaw]
         */
        double[] poseArray = LimelightHelpers.getTargetPose_RobotSpace(cameraName);

        if (poseArray == null || poseArray.length < 6) {
            return Optional.empty();
        }

        // x = forward/back, y = left/right, z = up/down
        return Optional.of(new Translation3d(poseArray[0], poseArray[1], poseArray[2]));
    }

    /** Gets the angle (bearing) to the tag relative to the robot's front. */
    public Rotation2d getAngleToTag(int targetID) {
        return getTranslationToTag(targetID)
                .map(translation -> new Rotation2d(translation.getX(), translation.getY()))
                .orElse(new Rotation2d(0));
    }

    /** Gets the straight-line distance to the tag. */
    public double getDistanceToTag(int targetID) {
        return getTranslationToTag(targetID).map(Translation3d::getNorm).orElse(0.0);
    }
}
