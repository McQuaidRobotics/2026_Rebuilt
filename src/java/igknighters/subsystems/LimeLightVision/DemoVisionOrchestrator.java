package igknighters.subsystems.LimeLightVision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import igknighters.util.Merging.PoseAverager;
import igknighters.util.Merging.TranslationMerger;
import igknighters.util.Vision.LocalizationCamera;
import igknighters.util.Vision.RelativeTracking;

public class DemoVisionOrchestrator {
    private final List<RelativeTracking> cameras;

    private final ArrayList<Integer> visibleTagIds;
    // used for localization
    double timestampSum = 0.0;
    double usedCameras = 0.0;
    // used for rumble
    double currentGreatestDoubleTagTimeStamp = 0.0;
    double lastGreatestDoubleTagTimeStamp = 0.0;

    public DemoVisionOrchestrator(List<RelativeTracking> cameras) {
        this.cameras = cameras;
        this.visibleTagIds = new ArrayList<>();
    }
    /**
     * Gets the merged translation in 3D space based on the turret angle.
     * @param turretAngle
     * @return the translation3d of the tag relative to the robot. A result of 0,0,0 if no valid translations are available.
     */
    public Translation3d getTranslation3d(Angle turretAngle) {
        timestampSum = 0.0;
        usedCameras = 0.0;
        lastGreatestDoubleTagTimeStamp = currentGreatestDoubleTagTimeStamp;
        currentGreatestDoubleTagTimeStamp = 0.0;
        ArrayList<Translation3d> translations = new ArrayList<>();
        for (var camera : cameras) {
            Translation3d translation = camera.getTranslation(turretAngle, 1); // Replace 1 with actual target ID if needed

            if (translation.getX() != 0 || translation.getY() != 0 || translation.getZ() != 0) {
                translations.add(translation);
            }
        }
        if (translations.isEmpty()) {
            return new Translation3d(0, 0, 0); // default value for no valid translations
        }
        return TranslationMerger.mergeTranslations(translations);
    }
}
