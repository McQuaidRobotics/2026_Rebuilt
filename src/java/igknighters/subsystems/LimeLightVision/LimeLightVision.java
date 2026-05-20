package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionSim;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights;
import igknighters.util.log.Log;
import java.util.List;

public class LimeLightVision extends SubsystemBase {
    private LimeLights vision;
    int iterations = 0;

    public LimeLightVision() {
        if (Robot.isReal()) {
            vision =
                    new LimeLightVisionReal(
                            SubsystemConstants.kLimelightVision.backCam,
                            SubsystemConstants.kLimelightVision.rightCam,
                            SubsystemConstants.kLimelightVision.turretCam,
                            SubsystemConstants.kLimelightVision.intakeCam);
        } else {
            vision = new LimeLightVisionSim("1", "2", "3", "4");
        }
    }

    public List<Integer> getVisibleTagIds() {
        return vision.getVisibleTagIds();
    }

    public void enableCameras(int IMU_MODE) {
        vision.enableCameras(IMU_MODE);
    }

    public void disableCameras() {
        vision.saveCameras();
    }

    public double getLastTimeStamp() {
        return vision.getLastTimeStamp();
    }

    public double timeSinceLastSample() {
        return vision.timeSinceLastSample();
    }

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vison/Limelight/ENABLED", true);
        }
        if (!Robot.ISDEMO) {
            return vision.getRobotPoseFromVision(yaw, yawRate, pitch, pitchRate, roll, rollRate);
        } else {
            return null;
        }
    }

    public Pose3d getRelativeTagPose(int tagId) {
        Log.log("ROBOT/Subsystems/Vision/Limelight/GETING TAG POSE", iterations++);
        Pose3d tagPose = vision.getRelativeTagPose(tagId);
        Log.log("ROBOT/Subsystems/Vision/Limelight/RELATIVE TAG POSE", tagPose);
        return tagPose;
    }
}
