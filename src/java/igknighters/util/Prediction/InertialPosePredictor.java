package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.LinkedList;

public class InertialPosePredictor {

    private static class InertialSample {
        public final Pose2d pose;
        public final ChassisSpeeds speeds;
        public final double gyroOmega; // Raw angular velocity
        public final double fusedAx;
        public final double fusedAy;
        public final double alpha; // Rotational acceleration
        public final double timestamp;

        public InertialSample(
                Pose2d pose,
                ChassisSpeeds speeds,
                double gyroOmega,
                double fusedAx,
                double fusedAy,
                double alpha,
                double timestamp) {
            this.pose = pose;
            this.speeds = speeds;
            this.gyroOmega = gyroOmega;
            this.fusedAx = fusedAx;
            this.fusedAy = fusedAy;
            this.alpha = alpha;
            this.timestamp = timestamp;
        }
    }

    // --- TUNABLE PARAMETERS ---
    private final double HISTORY_WINDOW_SECONDS = 0.25;
    private final double IMU_TRUST_FACTOR = 0.70; // 70% IMU, 30% Encoders for linear acceleration
    private final double ACCELERATION_DAMPENING = 0.85; // Attenuate high-order projections

    private final LinkedList<InertialSample> history = new LinkedList<>();

    /**
     * Ticks the predictor with full inertial and kinematic data.
     *
     * @param currentPose Global pose from your vision/odometry fuser
     * @param robotSpeeds Robot-centric chassis speeds (from swerve kinematics)
     * @param imuAccelX Raw X accelerometer reading in meters/sec^2
     * @param imuAccelY Raw Y accelerometer reading in meters/sec^2
     * @param gyroOmega Raw angular velocity in radians/sec
     * @param timestamp System timestamp (seconds)
     */
    public synchronized void update(
            Pose2d currentPose,
            ChassisSpeeds robotSpeeds,
            double imuAccelX,
            double imuAccelY,
            double gyroOmega,
            double timestamp) {
        if (history.isEmpty()) {
            history.addLast(
                    new InertialSample(currentPose, robotSpeeds, gyroOmega, 0, 0, 0, timestamp));
            return;
        }

        InertialSample last = history.getLast();
        double dt = timestamp - last.timestamp;
        if (dt <= 0) return;

        // 1. CORIOLIS CORRECTION
        // Strip out the centripetal forces caused by the robot spinning while moving
        double trueImuAx = imuAccelX + (gyroOmega * robotSpeeds.vyMetersPerSecond);
        double trueImuAy = imuAccelY - (gyroOmega * robotSpeeds.vxMetersPerSecond);

        // 2. KINEMATIC LINEAR ACCELERATION
        double encoderAx = (robotSpeeds.vxMetersPerSecond - last.speeds.vxMetersPerSecond) / dt;
        double encoderAy = (robotSpeeds.vyMetersPerSecond - last.speeds.vyMetersPerSecond) / dt;

        // 3. SENSOR FUSION (Linear)
        double fusedAx = (trueImuAx * IMU_TRUST_FACTOR) + (encoderAx * (1.0 - IMU_TRUST_FACTOR));
        double fusedAy = (trueImuAy * IMU_TRUST_FACTOR) + (encoderAy * (1.0 - IMU_TRUST_FACTOR));

        // 4. ROTATIONAL ACCELERATION
        // Gyros are pristine, so we take the derivative of the hardware rate directly
        double alpha = (gyroOmega - last.gyroOmega) / dt;

        history.addLast(
                new InertialSample(
                        currentPose, robotSpeeds, gyroOmega, fusedAx, fusedAy, alpha, timestamp));

        while (!history.isEmpty()
                && (timestamp - history.getFirst().timestamp) > HISTORY_WINDOW_SECONDS) {
            history.removeFirst();
        }
    }

    /**
     * Predicts the robot-centric ChassisSpeeds at a future timestamp using fused acceleration and
     * jerk vectors.
     *
     * @param lookaheadTimeSeconds How far into the future to project velocity.
     * @return The predicted robot-centric ChassisSpeeds.
     */
    public synchronized ChassisSpeeds predictVelocity(double lookaheadTimeSeconds) {
        if (history.size() < 5) {
            return history.isEmpty() ? new ChassisSpeeds() : history.getLast().speeds;
        }

        InertialSample current = history.getLast();
        InertialSample older = history.get(history.size() - 3);

        double dt = current.timestamp - older.timestamp;
        if (dt <= 0) return current.speeds;

        // 1. Core Current Velocities
        double vx = current.speeds.vxMetersPerSecond;
        double vy = current.speeds.vyMetersPerSecond;
        double omega = current.gyroOmega;

        // 2. Fused Current Accelerations
        double ax = current.fusedAx;
        double ay = current.fusedAy;
        double alpha = current.alpha;

        // 3. Calculate Jerk
        double jx = (ax - older.fusedAx) / dt;
        double jy = (ay - older.fusedAy) / dt;
        double zeta = (alpha - older.alpha) / dt;

        // 4. Taylor Series Expansion for Velocity (First derivative of position)
        double t = lookaheadTimeSeconds;
        // We apply the dampening modifier here as well to protect against extreme spikes
        double t2 = Math.pow(t, 2) * ACCELERATION_DAMPENING;

        double futureVx = vx + (ax * t) + (0.5 * jx * t2);
        double futureVy = vy + (ay * t) + (0.5 * jy * t2);
        double futureOmega = omega + (alpha * t) + (0.5 * zeta * t2);

        return new ChassisSpeeds(futureVx, futureVy, futureOmega);
    }

    /** Predicts future pose using 3rd-order Taylor Expansion projected via Exponential Mapping. */
    public synchronized Pose2d predict(double lookaheadTimeSeconds) {
        // Ensure we have enough history for a stable 4-frame lookback on a 20ms loop
        if (history.size() < 5) {
            return history.isEmpty() ? new Pose2d() : history.getLast().pose;
        }

        InertialSample current = history.getLast();
        InertialSample older =
                history.get(history.size() - 3); // Look back ~40ms for Jerk calculation

        double dt = current.timestamp - older.timestamp;
        if (dt <= 0) return current.pose;

        // 1. Core Velocities (Linear from encoders, Rotational from hardware Gyro)
        double vx = current.speeds.vxMetersPerSecond;
        double vy = current.speeds.vyMetersPerSecond;
        double omega = current.gyroOmega;

        // 2. Fused Accelerations
        double ax = current.fusedAx;
        double ay = current.fusedAy;
        double alpha = current.alpha;

        // 3. Calculate Jerk (Derivative of our already-fused acceleration)
        double jx = (ax - older.fusedAx) / dt;
        double jy = (ay - older.fusedAy) / dt;
        double zeta = (alpha - older.alpha) / dt;

        // 4. Apply Lookahead with Kinematic Taylor Integration & Centrifugal Decay
        double t = lookaheadTimeSeconds;
        double t2 = Math.pow(t, 2) * ACCELERATION_DAMPENING;
        double t3 = Math.pow(t, 3) * Math.pow(ACCELERATION_DAMPENING, 2);

        double deltaX = (vx * t) + (0.5 * ax * t2) + ((1.0 / 6.0) * jx * t3);
        double deltaY = (vy * t) + (0.5 * ay * t2) + ((1.0 / 6.0) * jy * t3);
        double deltaTheta = (omega * t) + (0.5 * alpha * t2) + ((1.0 / 6.0) * zeta * t3);

        // 5. Exponential Map Projection onto the Lie Manifold
        Twist2d futureTwist = new Twist2d(deltaX, deltaY, deltaTheta);
        return current.pose.exp(futureTwist);
    }
}
