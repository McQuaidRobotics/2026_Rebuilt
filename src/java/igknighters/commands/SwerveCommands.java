package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import wpilibExt.AllianceSymmetry;

public class SwerveCommands {

    public static Command zeroGyro(CommandSwerveDrivetrain swerve) {
        return Commands.either(
                Commands.runOnce(
                        () ->
                                swerve.resetPose(
                                        new Pose2d(
                                                swerve.getState().Pose.getX(),
                                                swerve.getState().Pose.getY(),
                                                new Rotation2d(0.0)))),
                Commands.runOnce(
                        () ->
                                swerve.resetPose(
                                        new Pose2d(
                                                swerve.getState().Pose.getX(),
                                                swerve.getState().Pose.getY(),
                                                new Rotation2d(Math.PI)))),
                AllianceSymmetry::isBlue);
    }

    public static Pose2d getPose(CommandSwerveDrivetrain swerve) {
        return swerve.getState().Pose;
    }

    public static Command stopDriving(CommandSwerveDrivetrain swerve) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 1.0)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 1.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        return swerve.run(
                () -> {
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(0.0)
                                    .withVelocityY(0.0)
                                    .withRotationalRate(0.0));
                });
    }

    public static Command moveToSimple(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
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

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(
                                            xController.calculate(
                                                    currentPose.getX(), targetPose.getX()))
                                    .withVelocityY(
                                            yController.calculate(
                                                    currentPose.getY(), targetPose.getY()))
                                    .withRotationalRate(
                                            thetaController.calculate(
                                                    currentPose.getRotation().getRadians(),
                                                    targetPose.getRotation().getRadians())));
                });
    }
}
