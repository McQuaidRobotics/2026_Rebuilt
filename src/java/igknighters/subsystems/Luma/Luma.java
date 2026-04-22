package igknighters.subsystems.Luma;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.subsystems.Luma.Cameras.Camera;
import igknighters.subsystems.Luma.Cameras.CameraReal;
import igknighters.subsystems.Luma.Cameras.CameraSim;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Luma extends SubsystemBase {
    private final List<Camera> cameras;

    public Luma(boolean disabled, String... cameraNames) {
        this.cameras = new ArrayList<>();
        boolean isReal = Robot.isReal();
        for (String name : cameraNames) {
            if (isReal && !disabled) {
                this.cameras.add(new CameraReal(name));
            } else {
                this.cameras.add(new CameraSim(name));
            }
        }
    }

    public Luma(Camera... cameras) {
        this.cameras = Arrays.asList(cameras);
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            camera.periodic();
        }
    }

    @Override
    public void simulationPeriodic() {
        for (Camera camera : cameras) {
            camera.simulationPeriodic();
        }
    }

    public Translation2d getClosestGamePiece() {
        Translation2d closest = null;
        double closestDist = Double.MAX_VALUE;

        for (Camera camera : cameras) {
            Translation2d offset = camera.getGamePieceOffset();
            // Assuming (0,0) means no target found (as implemented in CameraReal)
            double dist = offset.getNorm();
            if (dist > 0.01 && dist < closestDist) {
                closestDist = dist;
                closest = offset;
            }
        }
        return closest != null ? closest : new Translation2d();
    }

    @Override
    public String getName() {
        return "Luma";
    }
}
