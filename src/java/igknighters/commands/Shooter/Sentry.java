package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants.kLimelightVision;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.IndexerState;
import igknighters.subsystems.shooter.Shooter;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Demo sentry: tracks people holding AprilTags (id >= MIN_TAG_ID) with the fixed Limelights, hops
 * between them at random, and fires foam in bursts. Never aims or fires outside the front arc.
 */
public class Sentry extends Command {
    /** Turret angle (deg, robot-relative, CCW+) that points out the robot's front. VERIFY FIRST. */
    public static final double FRONT_DEG = 0.0;

    /** Never aim or fire more than this from FRONT_DEG (just under 90, operators stand behind). */
    public static final double ARC_HALF_DEG = 85.0;

    /** Field tags are 1-32, so kid tags start at 33 and pose estimation ignores them. */
    public static final int MIN_TAG_ID = 33;

    /** Flip to -1 if the turret points mirror-image away from the tag (Limelight yaw sign). */
    static final double YAW_SIGN = 1.0;

    static final double LOCK_DEG = 4.0;

    static final String[] CAMS = {
        kLimelightVision.turretCam,
        kLimelightVision.intakeCam,
        kLimelightVision.backCam,
        kLimelightVision.rightCam
    };

    private final Shooter shooter;
    private final Indexer indexer;
    private final DoubleSupplier pickStick; // right = +, picks rightmost/leftmost kid
    private final BooleanSupplier nextTarget;

    private final TunableDouble rpm = TunableValues.getDouble("Sentry/RPM", 2000);
    private final TunableDouble hood =
            TunableValues.getDouble(
                    "Sentry/HoodDeg", Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
    private final TunableDouble onlyTag = TunableValues.getDouble("Sentry/OnlyTagId", 0);

    private final Random rng = new Random();
    private final double[] camYaw = new double[CAMS.length];
    private int currentId = -1;
    private double switchAt, burstUntil, restUntil;
    private boolean lastNext;

    public Sentry(
            Shooter shooter,
            Indexer indexer,
            DoubleSupplier pickStick,
            BooleanSupplier nextTarget) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.pickStick = pickStick;
        this.nextTarget = nextTarget;
        addRequirements(shooter, indexer);
        setName("Sentry");
    }

    @Override
    public void initialize() {
        // Mount yaw comes from each Limelight's own robot-space camera pose (3D tab config)
        for (int i = 0; i < CAMS.length; i++) {
            camYaw[i] =
                    YAW_SIGN
                            * Math.toDegrees(
                                    LimelightHelpers.getCameraPose3d_RobotSpace(CAMS[i])
                                            .getRotation()
                                            .getZ());
            Log.log("Sentry/CamYawDeg/" + CAMS[i], camYaw[i]);
        }
        currentId = -1;
        switchAt = burstUntil = restUntil = 0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        Map<Integer, Double> seen = visibleTargets();

        boolean next = nextTarget.getAsBoolean();
        boolean nextPressed = next && !lastNext;
        lastNext = next;

        double stick = pickStick.getAsDouble();
        if (Math.abs(stick) > 0.5 && !seen.isEmpty()) {
            currentId = pickSide(seen, stick);
        } else if (nextPressed || now > switchAt || !seen.containsKey(currentId)) {
            currentId = pickOther(seen, currentId, rng);
            switchAt = now + 2.0 + 3.0 * rng.nextDouble();
        }

        Double bearing = seen.get(currentId);
        double want = bearing == null ? FRONT_DEG : FRONT_DEG + bearing;
        double goal = clampToArc(want);
        shooter.targetState(RPM.of(rpm.value()), Degrees.of(goal), Degrees.of(hood.value()));

        double turret = shooter.getTurretAngleDegrees();
        boolean locked =
                bearing != null
                        && inArc(want)
                        && inArc(turret)
                        && Math.abs(turret - goal) < LOCK_DEG
                        && ShootInformation.getInstance().getAtTarget();

        if (now > restUntil) { // new burst: 0.3-1.0s of fire, then 0.4-1.5s rest
            burstUntil = now + 0.3 + 0.7 * rng.nextDouble();
            restUntil = burstUntil + 0.4 + 1.1 * rng.nextDouble();
        }
        boolean fire = locked && now < burstUntil;
        indexer.goToState(fire ? IndexerState.DISPENSE_BALL : IndexerState.STOP);

        Log.log("Sentry/TargetId", currentId);
        Log.log("Sentry/GoalDeg", goal);
        Log.log("Sentry/Locked", locked);
        Log.log("Sentry/Firing", fire);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.goToState(IndexerState.STOP);
    }

    /** tag id -> robot-relative bearing (deg, CCW+) from every Limelight. */
    private Map<Integer, Double> visibleTargets() {
        int only = (int) onlyTag.value();
        Map<Integer, Double> seen = new HashMap<>();
        for (int i = 0; i < CAMS.length; i++) {
            for (var f : LimelightHelpers.getRawFiducials(CAMS[i])) {
                if (f.id < MIN_TAG_ID || (only != 0 && f.id != only)) continue;
                // ponytail: bearing from robot center, ignores camera/turret offset parallax
                // (~5deg at 3m); add tag-in-robot-frame math if shots land wide up close
                seen.put(f.id, MathUtil.inputModulus(camYaw[i] - f.txnc, -180, 180));
            }
        }
        return seen;
    }

    static double clampToArc(double turretDeg) {
        return MathUtil.clamp(turretDeg, FRONT_DEG - ARC_HALF_DEG, FRONT_DEG + ARC_HALF_DEG);
    }

    static boolean inArc(double turretDeg) {
        return Math.abs(turretDeg - FRONT_DEG) <= ARC_HALF_DEG;
    }

    /** stick right (+) -> rightmost kid (smallest CCW bearing), left -> leftmost. */
    static int pickSide(Map<Integer, Double> seen, double stick) {
        int best = -1;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var e : seen.entrySet()) {
            double score = -Math.signum(stick) * e.getValue();
            if (score > bestScore) {
                bestScore = score;
                best = e.getKey();
            }
        }
        return best;
    }

    /** random visible kid other than current; keeps current if they're the only one. */
    static int pickOther(Map<Integer, Double> seen, int current, Random rng) {
        List<Integer> others = new ArrayList<>(seen.keySet());
        others.remove(Integer.valueOf(current));
        if (others.isEmpty()) return seen.containsKey(current) ? current : -1;
        return others.get(rng.nextInt(others.size()));
    }
}
