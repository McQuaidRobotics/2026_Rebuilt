package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import java.util.List;

public abstract class LimeLights implements SharedSubsystem {
    public abstract Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate);

    public abstract double getLastTimeStamp();

    public abstract List<Integer> getVisibleTagIds();
}
