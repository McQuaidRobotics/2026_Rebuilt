package igknighters.util.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;

public abstract class LocalizationCamera {
    public abstract Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle);

    public abstract double getLastTimeStamp();

    public abstract ArrayList<Integer> getVisibleTagIds();
}
