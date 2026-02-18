package igknighters.subsystems.Luma.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public abstract class Camera {
    public abstract void periodic();

    public abstract void simulationPeriodic();

    public abstract String getName();

    public abstract Translation2d getGamePieceOffset();

    public abstract Pose2d getRobotPose();

    public abstract List<Translation2d> getTargetTranslations();
}
