package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.subsystems.YamShooter.Shooter;
import igknighters.subsystems.YamShooter.ShooterState;
import igknighters.subsystems.YamShooter.solvers.Math.LerpSolveShot;
import igknighters.util.TunableValues;
import java.util.function.Supplier;

public class ShooterCommands {

    public static TunableValues.TunableDouble shootRPM =
            TunableValues.getDouble("Shooter/ShootRPM", 3000);
    public static TunableValues.TunableDouble shootHoodAngle =
            TunableValues.getDouble(
                    "Shooter/ShootHoodAngle", Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());

    public static Command shootAtSpeed(Shooter shooter, double speed) {
        return Commands.run(() -> shooter.targetState(RPM.of(speed), Degrees.of(0), Degrees.of(0)))
                .withName("shoot at speed: " + speed);
    }

    public static Command targetNetworkTablesValues(Shooter shooter) {
        return Commands.run(
                () ->
                        shooter.targetState(
                                RPM.of(shootRPM.value()),
                                Degrees.of(0),
                                Degrees.of(shootHoodAngle.value())));
    }

    public static Command targetState(Shooter shooter, ShooterState state) {
        return Commands.run(
                () -> shooter.targetState(state.flywheelSpeed, state.turretAngle, state.hoodAngle));
    }

    public static Command stopShooting(Shooter shooter) {
        return Commands.runOnce(() -> shooter.setRollerVoltage(0)).withName("stop shooting");
    }

    public static Command aimTurretAtAngle(
            Shooter shooter, double turretAngleDegrees, double hoodAngleDegrees) {
        return Commands.run(
                        () ->
                                shooter.targetState(
                                        RPM.of(0),
                                        Degrees.of(turretAngleDegrees),
                                        Degrees.of(hoodAngleDegrees)))
                .withName("aiming turret + hood");
    }

    public static Command idle(Shooter shooter) {
        return Commands.run(() -> shooter.targetState(RPM.of(3000), Degrees.of(0), Degrees.of(0)))
                .withName("Idle Shooter");
    }

    public static Command targetState(
            Shooter shooter, double speed, double turretAngleDegrees, double hoodAngleDegrees) {
        return Commands.run(
                        () ->
                                shooter.targetState(
                                        RPM.of(speed),
                                        Degrees.of(turretAngleDegrees),
                                        Degrees.of(hoodAngleDegrees)))
                .withName("Target Shooter State");
    }

    public static Command homeHood(Shooter shooter) {
        // return Commands.run(() -> shooter.setHoodVoltage(-1)).until(()
        // ->shooter.isHoodSensorHit());
        if (shooter.isHoodSensorTripped()) {
            return Commands.runOnce(
                    () ->
                            shooter.hood.zeroAt(
                                    Degrees.of(
                                            Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES())));
        }
        return shooter.hood
                .run(() -> shooter.hood.setVoltage(-1))
                .until(() -> shooter.isHoodSensorTripped())
                .withTimeout(3.0)
                .withName("DRIVE DOWN HAS NOT HIT THE SENSOR YET HOME HOOD") // failsafe, can be
                // deleted if needed,
                // might be conflicting
                // with the
                // below code, needs testing on robot otherwise.
                .andThen(
                        Commands.runOnce(
                                () -> {
                                    shooter.hood.setVoltage(0);
                                    shooter.hood.zeroAt(
                                            Degrees.of(
                                                    Robot.consts
                                                            .shooter()
                                                            .kHood()
                                                            .MIN_ANGLE_DEGREES()));
                                }))
                .withName("HOOD IS DOWN ON SENSOR");
    }

    public static enum shotType {
        PASS,
        SHOT
    }

    public static Command shootAt(Shooter shooter, Supplier<Pose2d> targetPoseSupplier) {
        return Commands.run(
                        () -> {
                            Pose2d targetPose = targetPoseSupplier.get();
                            Pose3d targetPose3d = new Pose3d(targetPose);

                            ShooterState targetingData =
                                    LerpSolveShot.solve(
                                            targetPose3d,
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            0.02);

                            if (targetingData.flywheelSpeed.in(RPM) > 0.1) {
                                shooter.targetState(
                                        targetingData.flywheelSpeed,
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle);
                            } else {
                                // Fallback: Spin up to a safe mid-range RPM and keep turret pointed
                                // at target
                                shooter.targetState(
                                        RPM.of(3000.0),
                                        targetingData.turretAngle,
                                        Degrees.of(
                                                Robot.consts
                                                        .shooter()
                                                        .kHood()
                                                        .MIN_ANGLE_DEGREES()));
                            }
                        })
                .withName("Aiming at auto chosen target with look ahead");
    }
}
