package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.util.Merging.PoseAverager;

import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionSim extends LimeLights {

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
        return null;
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
        List<Integer> demoTags = new ArrayList<>();
        demoTags.add(15);
        return demoTags;
    }
}
