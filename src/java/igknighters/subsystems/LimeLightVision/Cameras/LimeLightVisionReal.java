package igknighters.subsystems.LimeLightVision.Cameras;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionReal extends LimeLights {
    private final List<String> cameraNames;
    private double lastTimeStamp = 0.0;

    public LimeLightVisionReal(String... cameraNames) {
        this.cameraNames = new ArrayList<>();
        for (String cameraName : cameraNames) {
            this.cameraNames.add(cameraName);
        }
    }

    private List<Integer> visibleTagIds = new ArrayList<>();

    // public Pose2d getRobotPoseFromVision(
    //         double yaw,
    //         double yawRate,
    //         double pitch,
    //         double pitchRate,
    //         double roll,
    //         double rollRate) {
    //     List<Pose2d> poses = new ArrayList<>();
    //     double timestamp = 0.0;
    //     visibleTagIds.clear();
    //     for (String cameraName : cameraNames) {
    //         LimelightHelpers.SetRobotOrientation(
    //                 cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    //         var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    //         var megaTag1Measurment = LimelightHelpers.getBotPose2d_wpiBlue(cameraName);

    //         if (llMeasurement != null && llMeasurement.tagCount > 0) {
    //             DogLog.log(
    //                     "Robot/Subsystems/Vision/LimeLightVision/RawPose_" + cameraName,
    //                     llMeasurement.pose);

    //             poses.add(llMeasurement.pose);
    //             timestamp += llMeasurement.timestampSeconds;
    //             for (var fiducial : llMeasurement.rawFiducials) {
    //                 visibleTagIds.add(fiducial.id);
    //             }
    //         }
    //     }
    //     if (!cameraNames.isEmpty()) {
    //         timestamp /= poses.size();
    //     }
    //     lastTimeStamp = timestamp;
    //     DogLog.log("Robot/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurments", timestamp);
    //     DogLog.log("Robot/Subsystems/Vision/LimeLightVision/NumberOfTagsSeen", poses.size());

    //     return averagePose2ds(poses);
    // }
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        List<Pose2d> poses = new ArrayList<>();
        double timestamp = 0.0;
        visibleTagIds.clear();

        for (String cameraName : cameraNames) {
            // Update Limelight with Gyro data for MT2
            LimelightHelpers.SetRobotOrientation(
                    cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);

            // Fetch both estimates
            var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

            if (mt2Estimate != null && mt2Estimate.tagCount > 0 && mt1Estimate != null) {
                // --- STABILITY LOGIC ---
                Rotation2d rotationToUse;

                // If we see 2+ tags, the vision rotation is stable. Use it to correct Gyro drift.
                if (mt1Estimate.tagCount >= 2) {
                    rotationToUse = mt1Estimate.pose.getRotation();
                }
                // If we only see 1 tag, vision rotation is jittery.
                // Fall back to MT2 (which is just your Gyro) to stay smooth.
                else {
                    rotationToUse = mt2Estimate.pose.getRotation();
                }

                // Create the Hybrid Pose: MT2 Translation + Selected Rotation
                Pose2d hybridPose = new Pose2d(mt2Estimate.pose.getTranslation(), rotationToUse);

                poses.add(hybridPose);

                // Use MT2 timestamp (primary source)
                timestamp += mt2Estimate.timestampSeconds;

                for (var fiducial : mt2Estimate.rawFiducials) {
                    visibleTagIds.add(fiducial.id);
                }

                // Optional: Log which rotation source we used for debugging
                DogLog.log(
                        "Subsystems/Vision/LimeLightVision/Source_" + cameraName,
                        (mt1Estimate.tagCount >= 2) ? "VISION_CORRECTION" : "ROBOT_GYRO_ONLY");
            }
        }

        if (!poses.isEmpty()) {
            timestamp /= poses.size();
        } else {
            timestamp = 0;
        }

        lastTimeStamp = timestamp;
        DogLog.log("Subsystems/Vision/LimeLightVision/TimeStampOfMeasurments", timestamp);
        DogLog.log("Subsystems/Vision/LimeLightVision/NumberOfTagsSeen", poses.size());

        return averagePose2ds(poses);
    }

    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    public Pose2d averagePose2ds(List<Pose2d> poses) {
        if (poses.isEmpty()) {
            DogLog.log("Subsystems/Vision/LimeLightVision/TagsSeen", "NO TAGS SEEN");
            return null; // safer than returning (0,0,0)
        } else {
            DogLog.log("Subsystems/Vision/LimeLightVision/TagsSeen", "Tag is seen we have a pose");
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
        DogLog.log("Subsystems/Vision/LimeLightVision/RotationList", rotations.toString());
        Rotation2d avgRot = new Rotation2d(Math.atan2(sinSum / count, cosSum / count));
        DogLog.log("Subsystems/Vision/LimeLightVision/Rotation", avgRot.getDegrees());
        Pose2d averaged = new Pose2d(avgX, avgY, avgRot);
        DogLog.log("Subsystems/Vision/LimeLightVision/TagsSeen", averaged);
        return averaged;
    }

    public double getLastTimeStamp() {
        return lastTimeStamp;
    }
}
