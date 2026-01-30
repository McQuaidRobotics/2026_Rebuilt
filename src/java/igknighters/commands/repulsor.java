package igknighters.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.ForceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;

public class repulsor {
    

    public static double getComponentX(Pose2d currentPose, Pose2d obstacle, double strength) {
        double dist = Math.hypot(obstacle.getX()-currentPose.getX(), obstacle.getX()-obstacle.getY());
        double cos = obstacle.getX()-currentPose.getX()/dist;
        double xRepelForce = cos*repelForce(currentPose, obstacle, dist, strength);
        return xRepelForce;
    }

    public static Command moveWithRepulsor(CommandSwerveDrivetrain swerve, Pose2d targetPose, Pose2d obstacle, double strength) {

        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 1.0)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 1.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        final PIDController xController =
                new PIDController(0.1, 0.0, 0.0); // Adjust gains as necessary
        final PIDController yController = new PIDController(0.1, 0.0, 0.0);
        final PIDController thetaController = new PIDController(0.1, 0.0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        for (obs : obstacles) {
            double obstacleForcesX = obs
        }

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(
                                            xController.calculate(
                                                    currentPose.getX(), targetPose.getX())+obstacleForcesX)
                                    .withVelocityY(
                                            yController.calculate(
                                                    currentPose.getY(), targetPose.getY())+obstacleForcesY)
                                    .withRotationalRate(
                                            thetaController.calculate(
                                                    currentPose.getRotation().getRadians(),
                                                    targetPose.getRotation().getRadians())));
                });
    }
}
