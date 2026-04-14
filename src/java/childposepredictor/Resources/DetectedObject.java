package childposepredictor.Resources;
import edu.wpi.first.math.geometry.Translation3d;

public class DetectedObject {
    public final int id;
    public final Translation3d position;
    public final double confidence;

    public DetectedObject(int id, Translation3d position, double confidence) {
        this.id = id;
        this.position = position;
        this.confidence = confidence;
    }
}
