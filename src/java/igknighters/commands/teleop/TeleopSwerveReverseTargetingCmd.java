package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;

public class TeleopSwerveReverseTargetingCmd extends TeleopSwerveBaseCmd {

    private final Pose2d targetPose;
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond()
                                    * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final PIDController rotationController;

    public TeleopSwerveReverseTargetingCmd(
            Swerve swerve,
            DriverController controller,
            Pose2d targetPose,
            double kP,
            double kI,
            double kD) {
        super(swerve, controller);
        this.targetPose = targetPose;
        addRequirements(swerve);
        rotationController =
                new PIDController(kP * 180.0 / Math.PI, kI * 180.0 / Math.PI, kD * 180.0 / Math.PI);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(.0001);
    }

    private double wrapAngleRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public void execute() {
        final var currentPose = swerve.getState().Pose;

        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        double desiredAngleRad = Math.atan2(dy, dx);
        double currentAngleRad = currentPose.getRotation().getRadians();

        // Wrap both angles explicitly
        desiredAngleRad = wrapAngleRadians(desiredAngleRad);
        currentAngleRad = wrapAngleRadians(currentAngleRad - Math.PI);

        double error = wrapAngleRadians(desiredAngleRad - currentAngleRad);

        if (!Robot.consts.disableAllLogs()) {
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveReverseTargetingCmd/Desired Angle (deg)",
                    Math.toDegrees(desiredAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveReverseTargetingCmd/Current Angle (deg)",
                    Math.toDegrees(currentAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveReverseTargetingCmd/Wrapped Error (deg)",
                    Math.toDegrees(error));
        }

        double omega = rotationController.calculate(currentAngleRad, desiredAngleRad);

        Translation2d vt = translationStick();
        double allianceFlipper = 1.0;
        // if (AllianceSymmetry.isBlue()) {
        //     allianceFlipper = 1.0;
        // } else {
        //     allianceFlipper = -1.0;
        // }

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(
                                vt.getX()
                                        * Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond()
                                        * allianceFlipper)
                        .withVelocityY(
                                vt.getY()
                                        * Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond()
                                        * allianceFlipper)
                        .withRotationalRate(omega));
    }
}
