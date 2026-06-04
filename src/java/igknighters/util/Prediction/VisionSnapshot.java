package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents a snapshot of vision measurements from multiple cameras. Holds the poses and
 * timestamps from each camera.
 */
public class VisionSnapshot {
    private final Pose2d[] pose;
    private final Double[] timestamp;

    public VisionSnapshot(Pose2d[] poses, Double[] timestamp) {
        this.pose = poses;
        this.timestamp = timestamp;
    }

    public Pose2d[] getPose() {
        return pose;
    }

    public Double[] getTimestamp() {
        return timestamp;
    }
}
