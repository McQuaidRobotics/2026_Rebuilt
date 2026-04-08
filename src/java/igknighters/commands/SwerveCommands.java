package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import igknighters.util.log.Log;
import java.util.function.BooleanSupplier;

public class SwerveCommands {

    public static Command zeroGyro(Swerve swerve) {
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

    public static Pose2d getPose(Swerve swerve) {
        return swerve.getState().Pose;
    }

    /**
     * Checks if the swerve is at the target velocity and not at the start pose. This was made so
     * that we can see if velocity is 0 but not when we start. Because at the start of climb
     * sequence velocity is 0.
     *
     * @param swerve The swerve subsystem
     * @param targetSpeeds The target chassis speeds
     * @param tolerance The tolerance for each chassis speed component
     * @param startPose The starting pose to compare against
     * @param positionToleranceMeters The position away from start in meters
     * @param angleToleranceRadians The angle difference in radians
     * @return A BooleanSupplier that returns true if the swerve is at the target velocity and not
     *     at the start pose
     */
    public static BooleanSupplier isAtVelocityAndNotAtStart(
            Swerve swerve,
            ChassisSpeeds targetSpeeds,
            ChassisSpeeds tolerance,
            Pose2d startPose,
            double positionToleranceMeters,
            double angleToleranceRadians) {
        return () -> {
            boolean isAtVel = isAtVelocity(swerve, targetSpeeds, tolerance).getAsBoolean();
            boolean isNotAtStart =
                    !isAt(swerve, startPose, positionToleranceMeters, angleToleranceRadians)
                            .getAsBoolean();
            return isAtVel && isNotAtStart;
        };
    }

    public static BooleanSupplier isAtVelocity(
            Swerve swerve, ChassisSpeeds targetSpeeds, ChassisSpeeds tolerance) {
        return () -> {
            ChassisSpeeds currentSpeeds = swerve.getFieldRelativeSpeeds();
            boolean isAt =
                    Math.abs(currentSpeeds.vxMetersPerSecond - targetSpeeds.vxMetersPerSecond)
                                    <= tolerance.vxMetersPerSecond
                            && Math.abs(
                                            currentSpeeds.vyMetersPerSecond
                                                    - targetSpeeds.vyMetersPerSecond)
                                    <= tolerance.vyMetersPerSecond
                            && Math.abs(
                                            currentSpeeds.omegaRadiansPerSecond
                                                    - targetSpeeds.omegaRadiansPerSecond)
                                    <= tolerance.omegaRadiansPerSecond;
            return isAt;
        };
    }

    public static Command stopDriving(Swerve swerve) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        return swerve.runOnce(
                        () -> {
                            swerve.setControl(
                                    m_driveRequest
                                            .withVelocityX(0.0)
                                            .withVelocityY(0.0)
                                            .withRotationalRate(0.0));
                        })
                .withName("Stop Driving");
    }

    public static BooleanSupplier isAt(
            Swerve swerve,
            Pose2d targetPose,
            double positionToleranceMeters,
            double angleToleranceRadians) {
        return () -> {
            Pose2d currentPose = swerve.getState().Pose;
            if (!Robot.consts.disableAllLogs()) {
                FieldVisualizer.getInstance().updateDrivingTarget(targetPose);
            }

            // 1. Calculate linear distance (Hypotenuse)
            double positionError =
                    currentPose.getTranslation().getDistance(targetPose.getTranslation());

            // 2. Calculate angular difference
            double currentHeading = currentPose.getRotation().getRadians();
            double targetHeading = targetPose.getRotation().getRadians();

            // Calculate raw error (Target - Current is the standard way to calculate error)
            double rawError = targetHeading - currentHeading;

            // Wrap the error to be within -PI to PI
            double angleError = Math.abs(Math.atan2(Math.sin(rawError), Math.cos(rawError)));

            boolean isAt =
                    positionError <= positionToleranceMeters && angleError <= angleToleranceRadians;

            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Commands/Swerve/IsAt/PositionError", positionError);
                Log.log("ROBOT/Commands/Swerve/IsAt/AngleError", angleError);
                Log.log("ROBOT/Commands/Swerve/IsAt/Reached Target", isAt);
            }

            return isAt;
        };
    }

    @SuppressWarnings("resource")
    public static Command moveToSimple(Swerve swerve, Pose2d targetPose) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        final PIDController xController =
                new PIDController(1, 0.2, 0.0); // Adjust gains as necessary
        xController.setTolerance(0.0);
        final PIDController yController = new PIDController(2, 0.02, 0.0);
        yController.setTolerance(0.0);
        final PIDController thetaController = new PIDController(1, 0.01, 0.0);
        thetaController.setTolerance(0.0);
        thetaController.enableContinuousInput(Math.PI, -Math.PI);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    final double vx = xController.calculate(currentPose.getX(), targetPose.getX());
                    final double vy = yController.calculate(currentPose.getY(), targetPose.getY());
                    final double omega =
                            thetaController.calculate(
                                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                                    MathUtil.angleModulus(targetPose.getRotation().getRadians()));
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/VX", vx);
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/VY", vy);
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/Omega", omega);
                        Log.log(
                                "Commands/Swerve/MoveToSimple/dx",
                                targetPose.getX() - currentPose.getX());
                        Log.log(
                                "Commands/Swerve/MoveToSimple/dy",
                                targetPose.getY() - currentPose.getY());
                        Log.log(
                                "Commands/Swerve/MoveToSimple/dtheta",
                                targetPose.getRotation().getRadians()
                                        - currentPose.getRotation().getRadians());
                    }
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(-vx)
                                    .withVelocityY(-vy)
                                    .withRotationalRate(omega));
                });
    }

    @SuppressWarnings("resource")
    public static Command moveToSimpleWithVelocityControl(
            Swerve swerve, Pose2d targetPose, Pose2d maxVelocities) {
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
                    if (!Robot.consts.disableAllLogs()) {

                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedVX",
                                clampedVx);
                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedVY",
                                clampedVy);
                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/ClampedOmega",
                                clampedOmega);

                        Log.log("ROBOT/Commands/Swerve/MoveToSimpleWithVelocityControl/VX", vx);
                        Log.log("ROBOT/Commands/Swerve/MoveToSimpleWithVelocityControl/VY", vy);
                        Log.log(
                                "ROBOT/Commands/Swerve/MoveToSimpleWithVelocityControl/Omega",
                                omega);
                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/dx",
                                targetPose.getX() - currentPose.getX());
                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/dy",
                                targetPose.getY() - currentPose.getY());
                        Log.log(
                                "Commands/Swerve/MoveToSimpleWithVelocityControl/dtheta",
                                targetPose.getRotation().getRadians()
                                        - currentPose.getRotation().getRadians());
                    }

                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(clampedVx)
                                    .withVelocityY(clampedVy)
                                    .withRotationalRate(clampedOmega));
                });
    }
}
