package igknighters.util.Vision.REAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.util.Vision.LocalizationCamera;
import igknighters.util.log.Log;
import java.util.ArrayList;

public class StaticCamera extends LocalizationCamera {
    String name;
    double lastTimeStamp;

    ArrayList<Integer> visibleTagIds;

    public StaticCamera(String cameraName) {
        this.name = cameraName;
        this.lastTimeStamp = 0.0;
        this.visibleTagIds = new ArrayList<>();
    }

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle) {

        Pose2d robotPose2d = null;
        visibleTagIds.clear();

        // Feed gyro to Limelight (for MT2)
        LimelightHelpers.SetRobotOrientation(name, yaw, yawRate, pitch, pitchRate, roll, rollRate);

        // Get both MT2 and MT1 estimates
        var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (mt2Estimate != null && mt1Estimate != null && mt1Estimate.tagCount > 0) {

            // --- ROTATION SELECTION LOGIC ---
            Rotation2d rotationToUse;
            if (mt1Estimate.tagCount >= 2) {
                // only update when we have a good vision estimate > 2 tags
                rotationToUse = mt1Estimate.pose.getRotation(); // vision rotation
                lastTimeStamp = mt2Estimate.timestampSeconds;
            } else {
                rotationToUse = mt2Estimate.pose.getRotation(); // fallback gyro-based
            }

            // MT2 translation + selected rotation
            Pose2d rotationOnlyPose = new Pose2d(mt2Estimate.pose.getTranslation(), rotationToUse);

            for (var fiducial : mt2Estimate.rawFiducials) {
                visibleTagIds.add(fiducial.id);
            }

            robotPose2d = rotationOnlyPose;

            // Optional: log rotation source
            if (!Robot.consts.limelightVision().disableVisionLogs()) {
                Log.log(
                        "Subsystems/Vision/LimeLightVision/Source_" + name,
                        (mt1Estimate.tagCount >= 2) ? "VISION_CORRECTION" : "ROBOT_GYRO_ONLY");
            }
        }

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log(
                    "ROBOT/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurements",
                    lastTimeStamp);
        }

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log(
                    "ROBOT/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurements",
                    lastTimeStamp);
        }
        return robotPose2d;
    }

    public double getLastTimeStamp() {
        return lastTimeStamp;
    }

    public ArrayList<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }
}
