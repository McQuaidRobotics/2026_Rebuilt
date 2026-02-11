package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

/**
 * FieldVisualizer provides methods to update specific objects on the field (Field2d) independently
 * of the main telemetry loop. This allows for updating vision targets, detected objects, and other
 * field elements only when necessary.
 */
public class FieldVisualizer {

    private FieldVisualizer() {}

    private static class SingletonHelper {
        private static final FieldVisualizer INSTANCE = new FieldVisualizer();
    }

    public static FieldVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");

    private final DoubleArrayPublisher shootingTargetPub =
            table.getDoubleArrayTopic("shootingTargetPose").publish();

    private final DoubleArrayPublisher drivingTargetPub =
            table.getDoubleArrayTopic("drivingTargetPose").publish();

    private final DoubleArrayPublisher detectedObjectsPub =
            table.getDoubleArrayTopic("detectedObjects").publish();

    /**
     * Updates the shooting target pose on the field.
     *
     * @param target The pose of the shooting target, or null to clear.
     */
    public void updateShootingTarget(Pose2d target) {
        if (target == null) {
            shootingTargetPub.set(new double[0]);
            return;
        }
        shootingTargetPub.set(
                new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
    }

    /**
     * Updates the driving target pose on the field.
     *
     * @param target The pose of the driving target, or null to clear.
     */
    public void updateDrivingTarget(Pose2d target) {
        if (target == null) {
            drivingTargetPub.set(new double[0]);
            return;
        }
        drivingTargetPub.set(
                new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
    }

    /**
     * Updates the list of detected objects on the field.
     *
     * @param objects A list of poses for detected objects, or null/empty to clear.
     */
    public void updateDetectedObjects(List<Pose2d> objects) {
        if (objects == null || objects.isEmpty()) {
            detectedObjectsPub.set(new double[0]);
            return;
        }
        double[] array = new double[objects.size() * 3];
        for (int i = 0; i < objects.size(); i++) {
            Pose2d pose = objects.get(i);
            array[i * 3] = pose.getX();
            array[i * 3 + 1] = pose.getY();
            array[i * 3 + 2] = pose.getRotation().getDegrees();
        }
        detectedObjectsPub.set(array);
    }

    /**
     * Updates all vision-related targets on the field in one call.
     *
     * @param detectedObjects List of detected object poses.
     * @param shootingTarget Shooting target pose.
     * @param drivingTarget Driving target pose.
     */
    public void updateVisionTargets(
            List<Pose2d> detectedObjects, Pose2d shootingTarget, Pose2d drivingTarget) {
        updateDetectedObjects(detectedObjects);
        updateShootingTarget(shootingTarget);
        updateDrivingTarget(drivingTarget);
    }
}
