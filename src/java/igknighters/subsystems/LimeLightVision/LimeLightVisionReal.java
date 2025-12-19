package igknighters.subsystems.LimeLightVision;

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
            LimelightHelpers.SetRobotOrientation(
                    cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

            if (llMeasurement != null && llMeasurement.tagCount > 0) {
                DogLog.log(
                        "Robot/Subsystems/LimeLightVision/RawPose_" + cameraName,
                        llMeasurement.pose);
                poses.add(llMeasurement.pose);
                timestamp += llMeasurement.timestampSeconds;
                for (var fiducial : llMeasurement.rawFiducials) {
                    visibleTagIds.add(fiducial.id);
                }
            }
        }
        if (!cameraNames.isEmpty()) {
            timestamp /= poses.size();
        }
        lastTimeStamp = timestamp;
        DogLog.log("Robot/Subsystems/LimeLightVision/TimeStampOfMeasurments", timestamp);
        DogLog.log("Robot/Subsystems/LimeLightVision/NumberOfTagsSeen", poses.size());

        return averagePose2ds(poses);
    }

    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    public Pose2d averagePose2ds(List<Pose2d> poses) {
        if (poses.isEmpty()) {
            DogLog.log("Robot/Subsystems/LimeLightVision/TagsSeen", "NO TAGS SEEN");
            return null; // safer than returning (0,0,0)
        } else {
            DogLog.log("Robot/Subsystems/LimeLightVision/TagsSeen", "Tag is seen we have a pose");
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
        DogLog.log("Robot/Subsystems/LimeLightVision/RotationList", rotations.toString());
        Rotation2d avgRot = new Rotation2d(Math.atan2(sinSum / count, cosSum / count));
        DogLog.log("Robot/Subsystems/LimeLightVision/Rotation", avgRot.getDegrees());
        Pose2d averaged = new Pose2d(avgX, avgY, avgRot);
        DogLog.log("Robot/Subsystems/LimeLightVision/TagsSeen", averaged);
        return averaged;
    }

    public double getLastTimeStamp() {
        return lastTimeStamp;
    }
}
