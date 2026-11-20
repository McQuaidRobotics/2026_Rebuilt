package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.commands.Shooter.Sentry;
import igknighters.commands.Shooter.Sentry.CamCfg;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.util.log.Log;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Sim-only demo world for {@link Sentry}: kids holding AprilTags wander around the robot, every
 * Limelight publishes the rawfiducials it would see (using the same pulled configs Sentry reads),
 * and FuelSim balls that reach a kid count as hits. No field tags involved, like a real demo.
 *
 * <p>Robot spawns mid-field facing +Y (open floor between the hubs, so balls don't hit field
 * pieces). 5 kids in front, 2 behind; the 2 behind must never get hit.
 */
public class SentrySim {
    static final double HFOV_DEG = 80, VFOV_DEG = 56; // LL4-ish, change for LL3 (63 x 50)
    static final double MAX_RANGE = 8.0, TAG_HEIGHT = 1.2, TAG_SIZE = 0.165;
    static final double KID_RADIUS = 0.3, KID_HEIGHT = 1.6, WANDER = 0.8, KID_SPEED = 0.6;
    static final double MIN_HIT_HEIGHT = 0.3;
    static final Pose2d ROBOT_START = new Pose2d(8.27, 2.0, Rotation2d.kCCW_90deg);

    /**
     * Kid spawn spots relative to the robot: (forward m, left m). Last two are BEHIND the robot.
     */
    static final double[][] SPAWNS = {
        {3.0, 0.0}, {4.0, 2.0}, {2.5, -2.0}, {5.0, -1.0}, {2.0, 2.5}, {-1.5, 1.0}, {-1.5, -1.0}
    };

    static class Kid {
        final int id;
        final Translation2d home;
        Translation2d pos, waypoint;
        int hits;
        final boolean behind;

        Kid(int id, Translation2d home, boolean behind) {
            this.id = id;
            this.home = home;
            this.pos = this.waypoint = home;
            this.behind = behind;
        }
    }

    private final FuelSim fuelSim;
    private final Supplier<Pose2d> robotPose;
    private final Consumer<Pose2d> resetRobot;
    private final Random rng = new Random();
    private final List<Kid> kids = new ArrayList<>();
    private final CamCfg[] cams = new CamCfg[Sentry.CAMS.length];
    private boolean moving = true;

    public SentrySim(FuelSim fuelSim, Supplier<Pose2d> robotPose, Consumer<Pose2d> resetRobot) {
        this.fuelSim = fuelSim;
        this.robotPose = robotPose;
        this.resetRobot = resetRobot;
        File dir = new File(Filesystem.getDeployDirectory(), Sentry.CONFIG_DIR);
        for (int i = 0; i < cams.length; i++) cams[i] = Sentry.loadCam(dir, Sentry.CAMS[i], 0);
        reset();
        SmartDashboard.putData(
                "Sentry Sim/Reset Demo", Commands.runOnce(this::reset).ignoringDisable(true));
        SmartDashboard.putData(
                "Sentry Sim/Toggle Kids Moving",
                Commands.runOnce(() -> moving = !moving).ignoringDisable(true));
    }

    /** Put the robot back at the start and respawn the kids around it. */
    public void reset() {
        resetRobot.accept(ROBOT_START);
        fuelSim.clearFuel();
        kids.clear();
        for (int i = 0; i < SPAWNS.length; i++) {
            Translation2d rel = new Translation2d(SPAWNS[i][0], SPAWNS[i][1]);
            Translation2d home =
                    ROBOT_START.getTranslation().plus(rel.rotateBy(ROBOT_START.getRotation()));
            kids.add(new Kid(Sentry.MIN_TAG_ID + i, home, SPAWNS[i][0] < 0));
        }
    }

    public void update() {
        if (moving) wander();
        publishCameras();
        countHits();
        logKids();
    }

    private void wander() {
        for (Kid k : kids) {
            Translation2d to = k.waypoint.minus(k.pos);
            if (to.getNorm() < 0.05) {
                k.waypoint =
                        k.home.plus(
                                new Translation2d(
                                        WANDER * (2 * rng.nextDouble() - 1),
                                        WANDER * (2 * rng.nextDouble() - 1)));
            } else {
                k.pos = k.pos.plus(to.times(Math.min(1, KID_SPEED * 0.02 / to.getNorm())));
            }
        }
    }

    /** Write what each Limelight would report to its rawfiducials NT entry. */
    private void publishCameras() {
        Pose3d robot = new Pose3d(robotPose.get());
        for (int c = 0; c < cams.length; c++) {
            if (cams[c] == null) continue;
            Pose3d cam = robot.transformBy(new Transform3d(cams[c].pos(), cams[c].rot()));
            List<Double> raw = new ArrayList<>();
            for (Kid k : kids) {
                double[] obs =
                        observe(cam, new Translation3d(k.pos.getX(), k.pos.getY(), TAG_HEIGHT));
                if (obs == null) continue;
                double distRobot = k.pos.getDistance(robot.getTranslation().toTranslation2d());
                // id, txnc, tync, ta, distToCamera, distToRobot, ambiguity
                raw.addAll(
                        Arrays.asList(
                                (double) k.id, obs[0], obs[1], obs[2], obs[3], distRobot, 0.1));
            }
            LimelightHelpers.getLimelightNTTableEntry(Sentry.CAMS[c], "rawfiducials")
                    .setDoubleArray(raw.stream().mapToDouble(Double::doubleValue).toArray());
        }
    }

    /** tx, ty, ta, dist for a tag at fieldPos seen by cam, or null if out of view. */
    static double[] observe(Pose3d cam, Translation3d fieldPos) {
        Translation3d v = new Pose3d(fieldPos, Rotation3d.kZero).relativeTo(cam).getTranslation();
        double dist = v.getNorm();
        if (v.getX() <= 0 || dist > MAX_RANGE) return null;
        double tx = -Math.toDegrees(Math.atan2(v.getY(), v.getX())); // Limelight: right +
        double ty = Math.toDegrees(Math.atan2(v.getZ(), v.getX()));
        if (Math.abs(tx) > HFOV_DEG / 2 || Math.abs(ty) > VFOV_DEG / 2) return null;
        double imageArea =
                4 * Math.tan(Math.toRadians(HFOV_DEG / 2)) * Math.tan(Math.toRadians(VFOV_DEG / 2));
        double ta = 100 * TAG_SIZE * TAG_SIZE / (dist * dist) / imageArea;
        return new double[] {tx, ty, ta, dist};
    }

    private void countHits() {
        fuelSim.fuels.removeIf(
                f -> {
                    // airborne only: misses bounce off the field walls and roll back along the
                    // floor, which would "hit" ankles behind the robot without being a shot
                    if (f.pos.getZ() > KID_HEIGHT || f.pos.getZ() < MIN_HIT_HEIGHT) return false;
                    for (Kid k : kids) {
                        if (f.pos.toTranslation2d().getDistance(k.pos) < KID_RADIUS) {
                            k.hits++;
                            return true;
                        }
                    }
                    return false;
                });
    }

    /** Hits on kids in front (behind=false) or behind the robot (behind=true). */
    public int hits(boolean behind) {
        return kids.stream().filter(k -> k.behind == behind).mapToInt(k -> k.hits).sum();
    }

    private void logKids() {
        Pose3d[] poses = new Pose3d[kids.size()];
        int total = 0, behindHits = 0;
        for (int i = 0; i < kids.size(); i++) {
            Kid k = kids.get(i);
            poses[i] = new Pose3d(k.pos.getX(), k.pos.getY(), KID_HEIGHT / 2, Rotation3d.kZero);
            Log.log("Sentry/Sim/Hits/" + k.id + (k.behind ? " (BEHIND)" : ""), k.hits);
            total += k.hits;
            if (k.behind) behindHits += k.hits;
        }
        Log.log("Sentry/Sim/Kids", poses);
        Log.log("Sentry/Sim/TotalHits", total);
        Log.log("Sentry/Sim/BehindHits (must be 0)", behindHits);
    }
}
