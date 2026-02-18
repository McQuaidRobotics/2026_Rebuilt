package igknighters.subsystems.Luma.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class CameraSim extends Camera {
    private final String name;

    public CameraSim(String name) {
        this.name = name;
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public Translation2d getGamePieceOffset() {
        return new Translation2d();
    }

    @Override
    public Pose2d getRobotPose() {
        return new Pose2d();
    }

    @Override
    public String getName() {
        return "CameraSim-" + name;
    }

    @Override
    public List<Translation2d> getTargetTranslations() {
        List<Translation2d> translations = new ArrayList<Translation2d>();
        return translations;
    }
}
