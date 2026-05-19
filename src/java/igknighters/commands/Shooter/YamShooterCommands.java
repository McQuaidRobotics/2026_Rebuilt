package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.YamShooter.hood.Hood;
import igknighters.subsystems.YamShooter.turret.Turret;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.ShootingData;
import igknighters.subsystems.shooter.solvers.Math.LerpSolveShot;

public class YamShooterCommands {
    
    public static Command idleHood(Hood hood) {
        return hood.targetAngle(Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
    }

    public static ShooterState getTargetData(AngularVelocity speed) {
        Pose3d targetPose = ShootInformation.getInstance().getTargetPose();
        return LerpSolveShot.solve(targetPose, speed.in(RPM), 0);
    }

    public static Command idleTurret(Turret turret) {
        return turret.run(
            () -> {
                ShooterState targetingData = getTargetData(RPM.of(0.0))
            }
        );
    }
}
