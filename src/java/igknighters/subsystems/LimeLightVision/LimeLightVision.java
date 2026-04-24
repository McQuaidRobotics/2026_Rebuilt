package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionSim;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights;
import igknighters.util.log.Log;
import java.util.List;

/**
 * Subsystem for managing Limelight-based vision. This class provides a high-level interface for
 * obtaining robot pose and target information from multiple Limelight cameras.
 *
 * <p>It automatically switches between real hardware ({@link LimeLightVisionReal}) and simulation
 * ({@link LimeLightVisionSim}) based on the robot's execution environment.
 */
public class LimeLightVision extends SubsystemBase {
    /** The underlying vision implementation (Real or Sim). */
    private LimeLights vision;

    /**
     * Constructs the LimeLightVision subsystem and initializes the appropriate vision backend based
     * on whether the robot is real or simulated.
     */
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

    /**
     * Returns a list of AprilTag IDs currently visible to the cameras.
     *
     * @return List of visible tag IDs.
     */
    public List<Integer> getVisibleTagIds() {
        return vision.getVisibleTagIds();
    }

    /**
     * Enables the vision cameras and sets the IMU integration mode.
     *
     * @param IMU_MODE The IMU mode to use (e.g., for rotation compensation).
     */
    public void enableCameras(int IMU_MODE) {
        vision.enableCameras(IMU_MODE);
    }

    /** Disables the vision cameras to save power or bandwidth. */
    public void disableCameras() {
        vision.saveCameras();
    }

    /**
     * Returns the FPGA timestamp of the last vision measurement.
     *
     * @return Timestamp in seconds.
     */
    public double getLastTimeStamp() {
        return vision.getLastTimeStamp();
    }

    /**
     * Returns the time elapsed since the last vision sample was received.
     *
     * @return Time in seconds.
     */
    public double timeSinceLastSample() {
        return vision.timeSinceLastSample();
    }

    /**
     * Estimates the robot's pose on the field based on visible AprilTags.
     *
     * @param yaw The robot's current yaw from the IMU.
     * @param yawRate The robot's current yaw rate.
     * @param pitch The robot's current pitch.
     * @param pitchRate The robot's current pitch rate.
     * @param roll The robot's current roll.
     * @param rollRate The robot's current roll rate.
     * @return The estimated {@link Pose2d}, or null if no tags are visible.
     */
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
            Log.log("ROBOT/Subsystems/Vision/Limelight/ENABLED", true);
        }
        return vision.getRobotPoseFromVision(yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }
}
