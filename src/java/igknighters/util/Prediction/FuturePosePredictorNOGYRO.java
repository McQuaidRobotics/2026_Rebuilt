package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.LinkedList;

public class FuturePosePredictorNOGYRO {

    private static class StateSample {
        public final Pose2d pose;
        public final ChassisSpeeds speeds;
        public final double timestamp;

        public StateSample(Pose2d pose, ChassisSpeeds speeds, double timestamp) {
            this.pose = pose;
            this.speeds = speeds;
            this.timestamp = timestamp;
        }
    }

    // --- CONFIGURABLE PARAMETERS ---
    private final double HISTORY_WINDOW_SECONDS = 0.25; // Track past 250ms of robot movement
    private final double ACCELERATION_DAMPENING = 0.85;  // Attenuate high-order projections 

    private final LinkedList<StateSample> history = new LinkedList<>();

    /**
     * Ticks the predictor. Call this every loop cycle
     * with the latest telemetry.
     */
    public synchronized void update(Pose2d currentPose, ChassisSpeeds currentSpeeds, double timestamp) {
        history.addLast(new StateSample(currentPose, currentSpeeds, timestamp));

        // Evict samples that are older than sliding window threshold
        while (!history.isEmpty() && (timestamp - history.getFirst().timestamp) > HISTORY_WINDOW_SECONDS) {
            history.removeFirst();
        }
    }

    /**
     * Predicts where the robot will be in the future.
     * * @param lookaheadTimeSeconds How far into the future to guess (e.g., 0.120 for 120ms)
     * @return The predicted Pose2d of the robot
     */
    public synchronized Pose2d predict_pose(double lookaheadTimeSeconds) {
        if (history.size() < 2) {
            // Not enough history to calculate patterns; fallback to baseline odometry
            return history.isEmpty() ? new Pose2d() : history.getLast().pose;
        }

        StateSample current = history.getLast();
        StateSample older = history.get(Math.max(0, history.size() - 4)); // Look back ~4 frames (~80ms)
        StateSample oldest = history.getFirst(); // Look back to the start of the window

        double dt1 = current.timestamp - older.timestamp;
        double dt2 = older.timestamp - oldest.timestamp;

        if (dt1 <= 0 || dt2 <= 0) {
            return current.pose; // Prevent division by zero on strange clock cycles
        }

        // 1. Current Velocities
        double vx = current.speeds.vxMetersPerSecond;
        double vy = current.speeds.vyMetersPerSecond;
        double omega = current.speeds.omegaRadiansPerSecond;

        // 2. Compute Accelerations (Slopes of Velocities)
        double ax = (current.speeds.vxMetersPerSecond - older.speeds.vxMetersPerSecond) / dt1;
        double ay = (current.speeds.vyMetersPerSecond - older.speeds.vyMetersPerSecond) / dt1;
        double alpha = (current.speeds.omegaRadiansPerSecond - older.speeds.omegaRadiansPerSecond) / dt1;

        // 3. Compute Jerk (The rate of acceleration changes / "Exponentialness")
        double axOlder = (older.speeds.vxMetersPerSecond - oldest.speeds.vxMetersPerSecond) / dt2;
        double ayOlder = (older.speeds.vyMetersPerSecond - oldest.speeds.vyMetersPerSecond) / dt2;
        double alphaOlder = (older.speeds.omegaRadiansPerSecond - oldest.speeds.omegaRadiansPerSecond) / dt2;

        double jx = (ax - axOlder) / ((current.timestamp - oldest.timestamp) / 2.0);
        double jy = (ay - ayOlder) / ((current.timestamp - oldest.timestamp) / 2.0);
        double zeta = (alpha - alphaOlder) / ((current.timestamp - oldest.timestamp) / 2.0);

        // 4. Apply Lookahead with Kinematic Taylor Integration & Centrifugal Decay
        // Dampen higher order terms over time to simulate traction/motor limitations safely
        double t = lookaheadTimeSeconds;
        double t2 = Math.pow(t, 2) * ACCELERATION_DAMPENING;
        double t3 = Math.pow(t, 3) * Math.pow(ACCELERATION_DAMPENING, 2);

        double deltaX = (vx * t) + (0.5 * ax * t2) + ((1.0 / 6.0) * jx * t3);
        double deltaY = (vy * t) + (0.5 * ay * t2) + ((1.0 / 6.0) * jy * t3);
        double deltaTheta = (omega * t) + (0.5 * alpha * t2) + ((1.0 / 6.0) * zeta * t3);

        // 5. Project Twist exponentially onto the Lie Manifold
        // WPILib's exp() integrates translation and rotation together, tracking curves perfectly!
        Twist2d effectiveTwist = new Twist2d(deltaX, deltaY, deltaTheta);
        return current.pose.exp(effectiveTwist);
    }
}