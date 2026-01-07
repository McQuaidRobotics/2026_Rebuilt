package igknighters.subsystems.Luma.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class CameraReal extends Camera {
    PhotonCamera camera;
    double cameraHeightMeters;
    double cameraPitchRadians;

    public CameraReal(String cameraName, double cameraHeightMeters, double cameraPitchRadians) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRadians = cameraPitchRadians;
    }

    public CameraReal(String cameraName) {
        this(cameraName, 0.5, 0.0); // Default placeholder values
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public Translation2d getGamePieceOffset() {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return new Translation2d();
        }
        var result = results.get(results.size() - 1);

        if (!result.hasTargets()) {
            return new Translation2d();
        }
        var latest = result.getBestTarget();

        double distance =
                PhotonUtils.calculateDistanceToTargetMeters(
                        cameraHeightMeters,
                        0.0, // Target height is 0 (ground)
                        cameraPitchRadians,
                        Units.degreesToRadians(latest.getPitch()));

        double yaw = Units.degreesToRadians(latest.getYaw());
        return new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));
    }

    @Override
    public Pose2d getRobotPose() {
        return new Pose2d();
    }

    @Override
    public String getName() {
        return "CameraReal-" + camera.getName();
    }
}
