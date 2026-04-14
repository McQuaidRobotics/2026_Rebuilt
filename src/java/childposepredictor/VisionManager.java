package childposepredictor;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;

import childposepredictor.Resources.BallTracker;
import childposepredictor.Resources.StereoVisionProcessor;

public class VisionManager {
    private final StereoVisionProcessor stereo = new StereoVisionProcessor();
    private final BallTracker tracker = new BallTracker();

    public void processFrame(double[][] leftCamData, double[][] rightCamData, double timestamp) {
        // leftCamData: Array of [id, x, y]
        for (int i = 0; i < leftCamData.length; i++) {
            int id = (int) leftCamData[i][0];
            double xL = leftCamData[i][1];
            double yL = leftCamData[i][2];
            double xR = rightCamData[i][1]; // Simplification: assumes IDs match

            Translation3d pos3d = stereo.calculateStereoTranslation(xL, xR, yL);
            tracker.updateBall(id, pos3d, timestamp);
        }
    }
}