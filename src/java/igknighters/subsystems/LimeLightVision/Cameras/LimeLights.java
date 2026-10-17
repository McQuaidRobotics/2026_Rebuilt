package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
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
}
