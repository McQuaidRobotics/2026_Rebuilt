package igknighters.util.Merging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.Robot;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.List;

public class PoseAverager {
    /** Averages a list of Pose2d objects (translation + rotation). */
    public static Pose2d averagePose2ds(List<Pose2d> poses) {
        if (poses.isEmpty()) {
            if (!Robot.consts.limelightVision().disableVisionLogs()) {
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

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/RotationList", rotations.toString());
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/Rotation", avgRot.getDegrees());
        }

        Pose2d averaged = new Pose2d(avgX, avgY, avgRot);
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/TagsSeen", averaged);
        }

        return averaged;
    }
}
