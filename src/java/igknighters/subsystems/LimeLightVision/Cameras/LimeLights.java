package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;

public abstract class LimeLights {
    public abstract Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate);

    public abstract double getLastTimeStamp();

    public abstract void saveCameras();

    public abstract void enableCameras(int imu_mode);

    public abstract double timeSinceLastSample();

    public abstract List<Integer> getVisibleTagIds();

    public abstract Pose3d getRelativeTagPose(int tagId);
}
