package igknighters.util.Vision;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public abstract class RelativeTracking {
    public abstract Translation3d getTranslation(Angle robotAngle, int targetID);
     
}
