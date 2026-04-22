package igknighters.util.Merging;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;

public class TranslationMerger {
    public static Translation3d mergeTranslations(ArrayList<Translation3d> translations) {
        double xSum = 0, ySum = 0, zSum = 0;
        for (Translation3d translation : translations) {
            xSum += translation.getX();
            ySum += translation.getY();
            zSum += translation.getZ();
        }
        return new Translation3d(xSum, ySum, zSum);
    }
}
