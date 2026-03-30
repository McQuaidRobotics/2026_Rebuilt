package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import igknighters.util.log.Log;

public class TeleopSwerveWithDetune extends TeleopSwerveBaseCmd {
    private double detune;
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public TeleopSwerveWithDetune(Swerve swerve, DriverController controller, double detune) {
        super(swerve, controller);
        this.detune = detune;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        super.execute();
        detune = DrivingSharedState.getInstance().detune;
        Translation2d vt = translationStick();

        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Subsystems/Swerve/DETUNE", detune);
        }

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(
                                vt.getX()
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * detune)
                        .withVelocityY(
                                vt.getY()
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * detune)
                        .withRotationalRate(
                                detune
                                        * RotationsPerSecond.of(0.75).in(RadiansPerSecond)
                                        * rotationStick().getX()));
    }
}
