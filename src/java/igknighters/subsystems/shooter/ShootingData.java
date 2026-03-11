package igknighters.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;

public class ShootingData {
    public double MAX_HEIGHT_METERS;
    public double MIN_HEIGHT_METERS;
    public Pose3d TARGET_POSE;

    public ShootingData(double MAX_HEIGHT_METERS, double MIN_HEIGHT_METERS, Pose3d TARGET_POSE) {
        this.MAX_HEIGHT_METERS = MAX_HEIGHT_METERS;
        this.MIN_HEIGHT_METERS = MIN_HEIGHT_METERS;
        this.TARGET_POSE = TARGET_POSE;
    }
}
