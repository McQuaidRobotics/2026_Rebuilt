package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
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
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import igknighters.util.log.Log;

public class TeleopSwerveForwardTargetingCmd extends TeleopSwerveBaseCmd {

    private final Pose2d targetPose;
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final PIDController rotationController;

    public TeleopSwerveForwardTargetingCmd(
            Swerve swerve,
            DriverController controller,
            Pose2d targetPose,
            double kP,
            double kI,
            double kD) {

        super(swerve, controller);
        rotationController =
                new PIDController(kP * 180.0 / Math.PI, kI * 180.0 / Math.PI, kD * 180.0 / Math.PI);
        this.targetPose = targetPose;
        addRequirements(swerve);

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
        currentAngleRad = wrapAngleRadians(currentAngleRad);

        double error = wrapAngleRadians(desiredAngleRad - currentAngleRad);

        if (!Robot.consts.disableAllLogs()) {
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveForwardTargetingCmd/Desired Angle (deg)",
                    Math.toDegrees(desiredAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveForwardTargetingCmd/Current Angle (deg)",
                    Math.toDegrees(currentAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveForwardTargetingCmd/Wrapped Error (deg)",
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
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * allianceFlipper)
                        .withVelocityY(
                                vt.getY()
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * allianceFlipper)
                        .withRotationalRate(omega));
    }
}
