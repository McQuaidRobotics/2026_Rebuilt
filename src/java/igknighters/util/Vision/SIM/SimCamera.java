package igknighters.util.Vision.SIM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import igknighters.util.Vision.LocalizationCamera;
import java.util.ArrayList;
import java.util.List;

/**
 * A simulated version of a Limelight camera using VisionSimulator. Mimics the structure of
 * StaticCamera for use in simulation environments.
 */
public class SimCamera extends LocalizationCamera {
    private final VisionSimulator simulator;
    private final String name;
    private double lastTimeStamp;
    private double lastDoubleTagTimeStamp;
    private ArrayList<Integer> visibleTagIds;

    /**
     * @param cameraName The name of the camera (for logging/ID)
     * @param minAngle The minimum FOV angle for this camera
     * @param maxAngle The maximum FOV angle for this camera
     */
    public SimCamera(String cameraName, double minAngle, double maxAngle) {
        this.name = cameraName;
        this.visibleTagIds = new ArrayList<>();
        this.lastDoubleTagTimeStamp = 0.0;

        // Initializing the simulator with the provided FOV constraints
        // Values based on your original LimeLightVisionSim defaults
        this.simulator =
                new VisionSimulator(
                        minAngle,
                        maxAngle,
                        5, // Max tags visible
                        0.0001, // stdDev
                        0.0, // latency
                        0.1 // trust value
                        );
    }

    @Override
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle) {

        // Get the simulated estimate
        Pose2d estimatedPose = simulator.getEstimatedPose();
        visibleTagIds.clear();

        if (estimatedPose != null) {
            lastTimeStamp = simulator.getTime();

            // Convert List to ArrayList to match abstract method signature
            List<Integer> tags = simulator.getVisibleTagIds();
            if (tags != null) {
                visibleTagIds.addAll(tags);
                if (tags.size() >= 2) {
                    lastDoubleTagTimeStamp = lastTimeStamp;
                }
            }

            return estimatedPose;
        }

        return null;
    }

    @Override
    public double getLastTimeStamp() {
        return lastTimeStamp;
    }

    @Override
    public double getLastDoubleTagTimeStamp() {
        return lastDoubleTagTimeStamp;
    }

    @Override
    public ArrayList<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    public String getName() {
        return name;
    }
}
