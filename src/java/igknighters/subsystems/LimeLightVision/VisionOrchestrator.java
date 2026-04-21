package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import igknighters.util.Merging.PoseAverager;
import igknighters.util.Vision.LocalizationCamera;
import java.util.ArrayList;
import java.util.List;

public class VisionOrchestrator {
    private final List<LocalizationCamera> cameras;

    private final ArrayList<Integer> visibleTagIds;
    // used for localization
    double timestampSum = 0.0;
    double usedCameras = 0.0;
    // used for rumble
    double currentGreatestDoubleTagTimeStamp = 0.0;
    double lastGreatestDoubleTagTimeStamp = 0.0;
    public VisionOrchestrator(List<LocalizationCamera> cameras) {
        this.cameras = cameras;
        this.visibleTagIds = new ArrayList<>();
    }

    public Pose2d getPose(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle) {
        ArrayList<Pose2d> poses = new ArrayList<>();
        visibleTagIds.clear();
        timestampSum = 0.0;
        usedCameras = 0.0;
        lastGreatestDoubleTagTimeStamp = currentGreatestDoubleTagTimeStamp;
        currentGreatestDoubleTagTimeStamp = 0.0;

        for (var camera : cameras) {
            Pose2d pose =
                    camera.getRobotPoseFromVision(
                            yaw, yawRate, pitch, pitchRate, roll, rollRate, turretAngle);
            if (pose != null) {
                poses.add(pose);
                timestampSum += camera.getLastTimeStamp();
                currentGreatestDoubleTagTimeStamp =
                        Math.max(currentGreatestDoubleTagTimeStamp, camera.getLastDoubleTagTimeStamp());
                usedCameras++;
                visibleTagIds.addAll(camera.getVisibleTagIds());
            }
        }

        return PoseAverager.averagePose2ds(poses);
    }

    public double getTimeStamp() {
        if (usedCameras == 0) {
            return 0.0;
        }
        return timestampSum / usedCameras;
    }

    public double getTimeSinceLastUpdate() {
        return currentGreatestDoubleTagTimeStamp - lastGreatestDoubleTagTimeStamp;
    }

    public ArrayList<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    public void saveCameras() {}

    public void enableCameras(int IMU_MODE) {}
}
