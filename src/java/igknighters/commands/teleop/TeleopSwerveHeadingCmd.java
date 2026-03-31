package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import igknighters.util.log.Log;

public class TeleopSwerveHeadingCmd extends TeleopSwerveBaseCmd {
    private final double heading;
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final PIDController rotationController;

    public TeleopSwerveHeadingCmd(
            Swerve swerve,
            DriverController controller,
            double heading,
            double kP,
            double kI,
            double kD) {
        super(swerve, controller);
        rotationController = new PIDController(kP, kI, kD);
        this.heading = heading;
        rotationController.enableContinuousInput(-180, 180);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double omega =
                rotationController.calculate(
                        swerve.getState().Pose.getRotation().getDegrees(), heading);
        if (!Robot.consts.disableAllLogs()) {
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveHeadingCmd/Swerve Heading: ",
                    (swerve.getState().Pose.getRotation().getDegrees()));
            Log.log(
                    "ROBOT/Commands/Swerve/TeleopSwerveHeadingCmd/error: ",
                    (swerve.getState().Pose.getRotation().getDegrees() - heading));
            Log.log("ROBOT/Commands/Swerve/TeleopSwerveHeadingCmd/PID CALCULATION: ", omega);
        }
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
