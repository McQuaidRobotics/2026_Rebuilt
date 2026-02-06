package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import java.util.function.BooleanSupplier;

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
                () -> Robot.isBlue());
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
                        })
                .withTimeout(.5);
    }

    public static BooleanSupplier isAt(
            CommandSwerveDrivetrain swerve,
            Pose2d targetPose,
            double positionToleranceMeters,
            double angleToleranceRadians) {
        return () -> {
            Pose2d currentPose = swerve.getState().Pose;

            // 1. Calculate linear distance (Hypotenuse)
            double positionError =
                    currentPose.getTranslation().getDistance(targetPose.getTranslation());

            // 2. Calculate angular difference
            double angleError =
                    Math.atan2(
                            Math.sin(currentPose.getRotation().getRadians())
                                    - Math.sin(targetPose.getRotation().getRadians()),
                            Math.cos(currentPose.getRotation().getRadians())
                                    - Math.cos(targetPose.getRotation().getRadians()));

            boolean isAt =
                    positionError <= positionToleranceMeters && angleError <= angleToleranceRadians;

            DogLog.log("Commands/Swerve/IsAt/PositionError", positionError);
            DogLog.log("Commands/Swerve/IsAt/AngleError", angleError);
            DogLog.log("Commands/Swerve/IsAt/Reached Target", isAt);

            return isAt;
        };
    }

    public static Command moveToSimple(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        final PIDController xController =
                new PIDController(0.5, 0.2, 0.0); // Adjust gains as necessary
        xController.setTolerance(0.0);
        final PIDController yController = new PIDController(0.5, 0.02, 0.0);
        yController.setTolerance(0.0);
        final PIDController thetaController = new PIDController(0.2, 0.01, 0.0);
        thetaController.setTolerance(0.0);
        thetaController.enableContinuousInput(0, 2 * Math.PI);

        return swerve.run(
                () -> {
                    // System.out.println(
                    //         "STARTING AUTO ALIGNMENT TO POSE: X: "
                    //                 + targetPose.getX()
                    //                 + " Y: "
                    //                 + targetPose.getY());
                    Pose2d currentPose = swerve.getState().Pose;
                    final double vx = xController.calculate(currentPose.getX(), targetPose.getX());
                    final double vy = yController.calculate(currentPose.getY(), targetPose.getY());
                    final double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());
                    DogLog.log("Commands/Swerve/MoveToSimple/VX", vx);
                    DogLog.log("Commands/Swerve/MoveToSimple/VY", vy);
                    DogLog.log("Commands/Swerve/MoveToSimple/Omega", omega);
                    DogLog.log(
                            "Commands/Swerve/MoveToSimple/dx",
                            targetPose.getX() - currentPose.getX());
                    DogLog.log(
                            "Commands/Swerve/MoveToSimple/dy",
                            targetPose.getY() - currentPose.getY());
                    DogLog.log(
                            "Commands/Swerve/MoveToSimple/dtheta",
                            targetPose.getRotation().getRadians()
                                    - currentPose.getRotation().getRadians());
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(-vx)
                                    .withVelocityY(-vy)
                                    .withRotationalRate(-omega));
                });
    }

    public static Command moveToSimpleWithVelocityControl(
            CommandSwerveDrivetrain swerve, Pose2d targetPose, Pose2d maxVelocities) {
        final PIDController xController =
                new PIDController(.1, 0.0, 0.0); // Adjust gains as necessary
        final PIDController yController = new PIDController(.1, 0.0, 0.0);
        final PIDController thetaController = new PIDController(.1, 0.0, 0.0);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        xController.setTolerance(0.0);
        yController.setTolerance(0.0);
        thetaController.setTolerance(0.0);
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * .01)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .01)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    final double vx = xController.calculate(currentPose.getX(), targetPose.getX());
                    final double vy = yController.calculate(currentPose.getY(), targetPose.getY());
                    final double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());

                    // Clamp speeds to max velocities
                    double clampedVx =
                            Math.max(Math.min(-vx, maxVelocities.getX()), -maxVelocities.getX());
                    double clampedVy =
                            Math.max(Math.min(-vy, maxVelocities.getY()), -maxVelocities.getY());
                    double clampedOmega =
                            Math.max(
                                    Math.min(-omega, maxVelocities.getRotation().getRadians()),
                                    -maxVelocities.getRotation().getRadians());

                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedVX", clampedVx);
                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedVY", clampedVy);
                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedOmega",
                            clampedOmega);

                    DogLog.log("Commands/Swerve/MoveToSimpleWithVelocityControl/VX", vx);
                    DogLog.log("Commands/Swerve/MoveToSimpleWithVelocityControl/VY", vy);
                    DogLog.log("Commands/Swerve/MoveToSimpleWithVelocityControl/Omega", omega);
                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/dx",
                            targetPose.getX() - currentPose.getX());
                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/dy",
                            targetPose.getY() - currentPose.getY());
                    DogLog.log(
                            "Commands/Swerve/MoveToSimpleWithVelocityControl/dtheta",
                            targetPose.getRotation().getRadians()
                                    - currentPose.getRotation().getRadians());

                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(clampedVx)
                                    .withVelocityY(clampedVy)
                                    .withRotationalRate(clampedOmega));
                });
    }
}
