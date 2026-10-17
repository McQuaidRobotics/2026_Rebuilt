package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public abstract class LimeLights {
    public enum PoseMode {
        REGULAR,
        PURE_MT1,
        PURE_MT2
    }

    // Debug overrides driven by the operator vision debug controller
    protected PoseMode poseMode = PoseMode.REGULAR;
    // LL3 ("limelight", turret cam) starts off
    private static final boolean[] DEFAULT_DISABLED = {false, false, true, false};
    protected final boolean[] cameraDisabled = DEFAULT_DISABLED.clone();

    public void setPoseMode(PoseMode mode) {
        poseMode = mode;
    }

    public PoseMode getPoseMode() {
        return poseMode;
    }

    /** index is camera constructor order (0 = LL1) */
    public void toggleCamera(int index) {
        cameraDisabled[index] = !cameraDisabled[index];
    }

    public boolean[] getCameraDisabled() {
        return cameraDisabled.clone();
    }

    public void resetDebug() {
        poseMode = PoseMode.REGULAR;
        System.arraycopy(DEFAULT_DISABLED, 0, cameraDisabled, 0, cameraDisabled.length);
    }

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
