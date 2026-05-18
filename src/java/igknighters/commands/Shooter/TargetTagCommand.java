package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.solvers.Math.LerpSolveShot;
import igknighters.subsystems.swerve.Swerve;

public class TargetTagCommand extends Command {
    private final Shooter shooter;
    private final LimeLightVision vision;
    private final Swerve swerve;
    private final int tagId;

    public TargetTagCommand(Shooter shooter, LimeLightVision vision, Swerve swerve, int tagId) {
        this.shooter = shooter;
        this.vision = vision;
        this.swerve = swerve;
        this.tagId = tagId;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose3d tagPoseRelative = vision.getRelativeTagPose(tagId);
        if (tagPoseRelative != null) {
            // Convert robot-relative tagPose to field-relative
            Pose3d robotPose = new Pose3d(swerve.getState().Pose);
            Pose3d fieldRelativeTagPose =
                    robotPose.plus(new Transform3d(new Pose3d(), tagPoseRelative));

            ShooterState state =
                    LerpSolveShot.solve(
                            fieldRelativeTagPose,
                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                            0.02);

            if (state.flywheelSpeed.in(RPM) != 0) {
                shooter.targetState(state);
            } else {
                // If shot is impossible, target with idle RPM but keep aiming
                shooter.targetState(
                        RPM.of(3000),
                        state.turretAngle,
                        Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
            }
        } else {
            // If tag not seen, idle or hold last position
            shooter.targetState(
                    RPM.of(3000),
                    Degrees.of(shooter.getTurretAngleDegrees()),
                    Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
        }
    }
}
