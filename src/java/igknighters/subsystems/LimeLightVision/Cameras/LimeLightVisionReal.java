package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionReal extends LimeLights {

    private final List<String> cameraNames;
    private double lastTimeStamp = 0.0;
    private final List<Integer> visibleTagIds = new ArrayList<>();

    public LimeLightVisionReal(String... cameraNames) {
        this.cameraNames = new ArrayList<>();
        for (String cameraName : cameraNames) {
            this.cameraNames.add(cameraName);
        }
    }

    /**
     * Returns a vision-based pose where translation comes from MT2 (reliable) and rotation comes
     * from MT1 (vision), ignoring MT1 translation entirely.
     */
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {

        List<Pose2d> poses = new ArrayList<>();
        double timestampSum = 0.0;
        visibleTagIds.clear();

        for (String cameraName : cameraNames) {

            // Feed gyro to Limelight (for MT2)
            LimelightHelpers.SetRobotOrientation(
                    cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);

            // Get both MT2 and MT1 estimates
            var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

            if (mt2Estimate != null && mt1Estimate != null && mt1Estimate.tagCount > 0) {

                // --- ROTATION SELECTION LOGIC ---
                Rotation2d rotationToUse;
                if (mt1Estimate.tagCount >= 2) {
                    rotationToUse = mt1Estimate.pose.getRotation(); // vision rotation
                } else {
                    rotationToUse = mt2Estimate.pose.getRotation(); // fallback gyro-based
                }

                // MT2 translation + selected rotation
                Pose2d rotationOnlyPose =
                        new Pose2d(mt2Estimate.pose.getTranslation(), rotationToUse);

                poses.add(rotationOnlyPose);

                // accumulate timestamp
                timestampSum += mt2Estimate.timestampSeconds;

                // collect visible tags
                for (var fiducial : mt2Estimate.rawFiducials) {
                    visibleTagIds.add(fiducial.id);
                }

                // Optional: log rotation source
                if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
                    Log.log(
                            "Subsystems/Vision/LimeLightVision/Source_" + cameraName,
                            (mt1Estimate.tagCount >= 2) ? "VISION_CORRECTION" : "ROBOT_GYRO_ONLY");
                }
            }

            double timestamp = !poses.isEmpty() ? timestampSum / poses.size() : 0.0;
            lastTimeStamp = timestamp;

            if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
                Log.log(
                        "ROBOT/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurements",
                        timestamp);
                Log.log(
                        "ROBOT/Subsystems/Vision/LimeLightVision/NumberOfTagsSeen",
                        visibleTagIds.size());
            }
        }

        return averagePose2ds(poses);
    }

    /** Returns a list of visible tag IDs in the current frame. */
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    /** Returns the last timestamp from vision measurements. */
    public double getLastTimeStamp() {
        return lastTimeStamp;
    }

    /** Averages a list of Pose2d objects (translation + rotation). */
    public Pose2d averagePose2ds(List<Pose2d> poses) {
        if (poses.isEmpty()) {
            if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
                Log.log("ROBOT/Subsystems/Vision/LimeLightVision/TagsSeen", "NO TAGS SEEN");
            }
            return null;
        }

        double xSum = 0.0, ySum = 0.0;
        double sinSum = 0.0, cosSum = 0.0;
        List<Double> rotations = new ArrayList<>();

        for (Pose2d pose : poses) {
            xSum += pose.getX();
            ySum += pose.getY();
            sinSum += Math.sin(pose.getRotation().getRadians());
            cosSum += Math.cos(pose.getRotation().getRadians());
            rotations.add(pose.getRotation().getDegrees());
        }

        int count = poses.size();
        double avgX = xSum / count;
        double avgY = ySum / count;
        Rotation2d avgRot = new Rotation2d(Math.atan2(sinSum / count, cosSum / count));

        if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/RotationList", rotations.toString());
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/Rotation", avgRot.getDegrees());
        }

        Pose2d averaged = new Pose2d(avgX, avgY, avgRot);
        if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/TagsSeen", averaged);
        }

        return averaged;
    }
}
