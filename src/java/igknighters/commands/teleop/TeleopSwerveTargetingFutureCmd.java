package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import igknighters.util.log.Log;

public class TeleopSwerveTargetingFutureCmd extends TeleopSwerveBaseCmd {
    private final Pose2d targetPose;
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final PIDController rotationController;

    private final double lookaheadTimeSeconds; // Lookahead time in seconds

    // Make the PID controller handle wraparound automatically

    public TeleopSwerveTargetingFutureCmd(
            Swerve swerve,
            DriverController controller,
            Pose2d targetPose,
            double lookaheadTimeSeconds,
            double kP,
            double kI,
            double kD) {
        super(swerve, controller);
        this.targetPose = targetPose;
        this.lookaheadTimeSeconds = lookaheadTimeSeconds;
        rotationController =
                new PIDController(kP * 180.0 / Math.PI, kI * 180.0 / Math.PI, kD * 180.0 / Math.PI);
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
        // x y r
        final var currentPose = swerve.getState().Pose;
        final var fieldSpeeds = swerve.getFieldRelativeSpeeds();
        final Pose2d futurePose2d =
                new Pose2d(
                        currentPose.getX() + (fieldSpeeds.vxMetersPerSecond * lookaheadTimeSeconds),
                        currentPose.getY() + (fieldSpeeds.vyMetersPerSecond * lookaheadTimeSeconds),
                        new Rotation2d(
                                currentPose.getRotation().getRadians()
                                        + (fieldSpeeds.omegaRadiansPerSecond
                                                * lookaheadTimeSeconds)));

        double dx = targetPose.getX() - futurePose2d.getX();
        double dy = targetPose.getY() - futurePose2d.getY();

        double desiredAngleRad = Math.atan2(dy, dx);
        double futureAngleRad = futurePose2d.getRotation().getRadians();

        // Wrap both angles explicitly
        desiredAngleRad = wrapAngleRadians(desiredAngleRad);
        futureAngleRad = wrapAngleRadians(futureAngleRad);

        double error = wrapAngleRadians(desiredAngleRad - futureAngleRad);

        if (!Robot.consts.disableAllLogs()) {
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveTargetingFutureCmd/Desired Angle (deg)",
                    Math.toDegrees(desiredAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveTargetingFutureCmd/Current Angle (deg)",
                    Math.toDegrees(futureAngleRad));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveTargetingFutureCmd/Wrapped Error (deg)",
                    Math.toDegrees(error));
        }

        double omega = rotationController.calculate(futureAngleRad, desiredAngleRad);

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
