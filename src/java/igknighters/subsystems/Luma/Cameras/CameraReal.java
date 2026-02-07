package igknighters.subsystems.Luma.Cameras;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraReal extends Camera {
    PhotonCamera camera;
    String name;
    double cameraHeightMeters;
    double cameraPitchRadians;
    Translation2d robotToCameraTranslation;
    List<PhotonPipelineResult> results = new ArrayList<>();
    boolean noObjects = false;

    public CameraReal(
            String cameraName, double cameraHeightMeters, Translation2d robotToCameraTranslation) {
        this.camera = new PhotonCamera(cameraName);
        DogLog.log(cameraName, true);
        this.name = cameraName;
        DogLog.log("Subsystems/Vision/" + cameraName + "/Status", "ENABLED");

        camera.setPipelineIndex(0);
        this.cameraHeightMeters = cameraHeightMeters;
        this.robotToCameraTranslation = robotToCameraTranslation;
    }

    public CameraReal(String cameraName) {
        this(cameraName, 0.5, new Translation2d()); // Default placeholder values
    }

    @Override
    public void periodic() {

        DogLog.log("Subsystems/Vision/" + name + "/Connected", camera.isConnected());
        List<PhotonPipelineResult> potentialResults = new ArrayList<>();
        potentialResults = camera.getAllUnreadResults();
        //this stops the robot from using an empty list if it is the first cycle of empty
        if (!potentialResults.isEmpty()) {
            results = potentialResults;
            noObjects = false;
        } else if (potentialResults.isEmpty() && noObjects == true) {
            results = potentialResults;
            noObjects = true;
        } else {
            noObjects = true;
        }
        DogLog.log("Subsystems/Vision/Physical RESULTS IN PERIODIC", potentialResults.size());
        DogLog.log("Subsystems/Vision/RESULTS IN PERIODIC", results.size());
    }

    @Override
    public void simulationPeriodic() {}

    public Translation2d getGamePieceOffsetFromTargetList(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            throw new IllegalArgumentException(
                    "Target list is empty in getGamePieceOffsetFromTargetList");
        }
        targets.sort(Comparator.comparingDouble(t -> t.getArea()));

        var bestTarget = targets.get(targets.size() - 1);
        double distance =
                PhotonUtils.calculateDistanceToTargetMeters(
                        cameraHeightMeters,
                        0.0, // Target height is 0 (ground)
                        cameraPitchRadians,
                        Units.degreesToRadians(bestTarget.getPitch()));

        double yaw = Units.degreesToRadians(bestTarget.getYaw());
        return new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw))
                .plus(robotToCameraTranslation);
    }

    @Override
    public Translation2d getGamePieceOffset() {
        DogLog.log("Subsystems/Vision/Getting Offset", true);
        if (results.isEmpty()) {
            DogLog.log("Subsystems/Vision/ObjectDetection/Camera Results", false);
            return new Translation2d();
        } else {
            DogLog.log("Subsystems/Vision/ObjectDetection/Camera Results", true);
        }
        var result = results.get(results.size() - 1);
        if (!result.hasTargets()) {
            DogLog.log("Subsystems/Vision/ObjectDetection/Camera Has Target", false);
            return new Translation2d();
        } else {
            DogLog.log("Subsystems/Vision/ObjectDetection/Camera Has Target", true);
        }
        var targets = result.getTargets();

        targets.sort(Comparator.comparingDouble(t -> t.getYaw()));

        List<PhotonTrackedTarget> bestCluster = new ArrayList<>();
        List<PhotonTrackedTarget> currentCluster = new ArrayList<>();

        double clusterThresholdDeg = 10.0;

        for (var t : targets) {
            if (currentCluster.isEmpty()) {
                currentCluster.add(t);
                continue;
            }

            var last = currentCluster.get(currentCluster.size() - 1);

            if (Math.abs(t.getYaw() - last.getYaw()) <= clusterThresholdDeg) {
                currentCluster.add(t);
            } else {
                if (currentCluster.size() > bestCluster.size()) {
                    bestCluster = new ArrayList<>(currentCluster);
                }
                currentCluster.clear();
                currentCluster.add(t);
            }
        }

        if (currentCluster.size() > bestCluster.size()) {
            bestCluster = currentCluster;
        }

        DogLog.log("Subsystems/Vision/ObjectDetection/Camera Cluster Size", bestCluster.size());

        return getGamePieceOffsetFromTargetList(bestCluster);
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
