package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.subsystems.LimeLightVision.Helpers.VisionSimulator;
import igknighters.util.Merging.PoseAverager;

import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionSim extends LimeLights {
    List<VisionSimulator> visionSimulators;
    List<Pose2d> estimatedPoses;
    List<Double[]> cameraRotations;
    List<Integer> visibleTagIds = new ArrayList<>();
    double timesum = 0.0;
    double camerasThatSeeStuff = 0.0;

    double cameraFOV = 60;

    public LimeLightVisionSim(String... cameraNames) {
    }

    @Override
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        estimatedPoses.clear();
        visibleTagIds.clear();
        timesum = 0.0;
        camerasThatSeeStuff = 0.0;
        for (VisionSimulator simulator : visionSimulators) {
            Pose2d estimatedPose = simulator.getEstimatedPose();
            if (estimatedPose != null) {
                estimatedPoses.add(estimatedPose);
                timesum += simulator.getTime();
                camerasThatSeeStuff++;
                visibleTagIds.addAll(simulator.getVisibleTagIds());
            }
        }
        return PoseAverager.averagePose2ds(estimatedPoses);
    }

    @Override
    public double getLastTimeStamp() {
        // In simulation, we can get the timestamp from the latest results
        return 0.0;
    }

    @Override
    public double timeSinceLastSample() {
        return 0.0;
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }
}
