package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;

public class AutoRotateOnBump extends TeleopSwerveBaseCmd {
    private double detune;
    private final PIDController thetaController = new PIDController(4.0, 0, 0);
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public AutoRotateOnBump(Swerve swerve, DriverController controller) {
        super(swerve, controller);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    public double getBestAngleToTarget(double currentAngleDegrees) {
        double multipleOf45 = Math.floor(currentAngleDegrees / 45);

        if (multipleOf45 % 2 == 0) {
            return (multipleOf45 + 1) * 45;
        } else {
            return (multipleOf45) * 45;
        }
    }

    @Override
    public void execute() {
        super.execute();
        detune = DrivingSharedState.getInstance().detune;
        Translation2d vt = translationStick();

        double targetAngle =
                Math.toRadians(
                        getBestAngleToTarget(swerve.getState().Pose.getRotation().getDegrees()));
        double currentAngle = swerve.getState().Pose.getRotation().getRadians();

        double rotationRate = thetaController.calculate(currentAngle, targetAngle);

        DogLog.log("Commands/AutoRotateOnBump/Active", true);
        DogLog.log("Commands/AutoRotateOnBump/CurrentAngle", currentAngle);
        DogLog.log("Commands/AutoRotateOnBump/TargetAngle", targetAngle);
        DogLog.log("Commands/AutoRotateOnBump/RotationRate", rotationRate);

        FieldConstants.BUMP.PROTECTION_MOVEMENT directionToMove =
                FieldConstants.BUMP.getProtectionMovement(swerve.getState().Pose, 0.5);

        // Force a smaller speed on the bump as requested
        double bumpSpeedMultiplier = 0.5;
        double maxSpeed =
                knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                        * detune
                        * bumpSpeedMultiplier;

        // Logic for Y velocity based on bump protection
        double vy = vt.getY() * maxSpeed;

        // Adjust vy based on directionToMove if necessary
        if (directionToMove == FieldConstants.BUMP.PROTECTION_MOVEMENT.GO_UP) {
            // For example, force a positive Y velocity or keep current stick if already moving left
            vy = Math.min(vy, -0.5 * maxSpeed);
        } else if (directionToMove == FieldConstants.BUMP.PROTECTION_MOVEMENT.GO_DOWN) {
            vy = Math.max(vy, +0.5 * maxSpeed);
        }

        DogLog.log("Commands/AutoRotateOnBump/DirectionToMove", directionToMove);

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(vt.getX() * maxSpeed)
                        .withVelocityY(vy)
                        .withRotationalRate(rotationRate));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        DogLog.log("Commands/AutoRotateOnBump/Active", false);
    }
}
