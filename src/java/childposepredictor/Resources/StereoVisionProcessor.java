package childposepredictor.Resources;

import edu.wpi.first.math.geometry.Translation3d;

public class StereoVisionProcessor {
    // Distance between the two cameras (Baseline)
    private final double BASELINE_METERS = 0.5; 
    // Focal length in pixels (depends on your camera resolution/FOV)
    private final double FOCAL_LENGTH_PX = 640; 

    public Translation3d calculateStereoTranslation(double xLeftPx, double xRightPx, double yPx) {
        // Disparity is the difference in horizontal position between the two cameras
        double disparity = Math.abs(xLeftPx - xRightPx);
        
        if (disparity == 0) return new Translation3d();

        // Standard Stereo depth formula: Z = (Baseline * FocalLength) / Disparity
        double z = (BASELINE_METERS * FOCAL_LENGTH_PX) / disparity;
        
        // Calculate X and Y relative to the center of the camera rig
        double x = (xLeftPx * z) / FOCAL_LENGTH_PX;
        double y = (yPx * z) / FOCAL_LENGTH_PX;

        return new Translation3d(z, -x, -y); // Adjusted for standard robot coordinate frames
    }
}