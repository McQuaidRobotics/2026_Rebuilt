package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionSim extends LimeLights {

    public LimeLightVisionSim() {}

    @Override
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        // In simulation, we can directly get the botpose from LimelightHelpers
        // This assumes that LimelightHelpersSimTools is populating the NetworkTables
        // with appropriate simulation data
        return null;
    }

    @Override
    public double getLastTimeStamp() {
        // In simulation, we can get the timestamp from the latest results
        return 0.0;
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public List<Integer> getVisibleTagIds() {
        List<Integer> demoTags = new ArrayList<>();
        demoTags.add(15);
        return demoTags;
    }
}
