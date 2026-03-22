package igknighters.subsystems.shooter.solvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.LerpTable;

public class LerpSolveShot {

    private static final LerpTable rpmLerp = new LerpTable();
    private static final LerpTable hoodLerp = new LerpTable();
    private static final LerpTable timeOfFlightLerp = new LerpTable();

    

    private record FullShooterParams(double rpm, double hoodAngle, double timeOfFlight) {}
    public ShooterState calculateBoth(double distance, double requiredVelocity) {
        FullShooterParams baseline = SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;
        double velocityRatio = requiredVelocity / baselineVelocity;

        // Split the correction: sqrt gives equal "contribution" from each
        double rpmFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply RPM scaling
        double adjustedRpm = baseline.rpm * rpmFactor;

        // Apply hood adjustment (changes horizontal component)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        return new ShooterCommand(adjustedRpm, adjustedHood);
    }
}
