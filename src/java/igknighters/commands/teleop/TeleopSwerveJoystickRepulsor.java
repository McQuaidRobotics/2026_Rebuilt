package igknighters.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.commands.Repulsor;
import igknighters.commands.Repulsor.obstacle;
import igknighters.constants.FieldConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.ControllerConstants;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import monologue.ProceduralStructGenerator;

public class TeleopSwerveJoystickRepulsor extends Command {
    protected final Swerve swerve;

    private final DoubleSupplier rawTranslationXSup;
    private final DoubleSupplier rawTranslationYSup;
    private final DoubleSupplier rawRotationXSup;
    private final DoubleSupplier rawRotationYSup;

    private final TunableDouble translationMod;
    private final TunableDouble rotationMod;
    private static final boolean demo = false;

    public TeleopSwerveJoystickRepulsor(Swerve swerve, DriverController controller) {
        this.swerve = swerve;

        this.rawTranslationXSup = controller.leftStickX();
        this.rawTranslationYSup = controller.leftStickY();
        this.rawRotationXSup = controller.rightStickX();
        this.rawRotationYSup = controller.rightStickY();

        if (demo) {
            translationMod = TunableValues.getDouble("DemoSwerveTranslationModifier", 0.8);
            rotationMod = TunableValues.getDouble("DemoSwerveRotationalModifier", 0.8);
        } else {
            translationMod = null;
            rotationMod = null;
        }
    }

    private double solveJoystickDiagonalDelta(double x, double y) {
        double absX = Math.abs(x);
        double absY = Math.abs(y);
        double diffPercent = 1.0 - (Math.abs(absX - absY) / Math.max(absX, absY));
        double out = Math.max(Math.hypot(x, y) - (0.12 * diffPercent), 0.0);
        if (!Double.isFinite(out)) return 0.0;
        return out;
    }

    protected Translation2d translationStick() {
        double repulseMod = .05;
        ArrayList<obstacle> obstacles = FieldConstants.OBSTACLES.ALL_OBSTACLES;
        double rawX = rawTranslationXSup.getAsDouble();
        double rawY = rawTranslationYSup.getAsDouble();
        double angle = Math.atan2(rawY, rawX);
        double rawMagnitude = solveJoystickDiagonalDelta(rawX, rawY);
        double XRepulse = Repulsor.getXRepulse(swerve.getState().Pose, obstacles) * repulseMod;
        double YRepulse = Repulsor.getYRepulse(swerve.getState().Pose, obstacles) * repulseMod;
        rawMagnitude = MathUtil.clamp(rawMagnitude, -1, 1);
        double magnitude =
                ControllerConstants.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
        if (demo) magnitude *= translationMod.value();
        double processedX = magnitude * Math.cos(angle);
        double processedY = magnitude * Math.sin(angle);
        double repulseProcessedX = processedX;
        double repulseProcessedY = processedY;
        if (XRepulse != 0 && YRepulse != 0) {
            repulseProcessedX += XRepulse;
        }
        if (YRepulse != 0) {
            repulseProcessedY += YRepulse;
        }
        if (!Robot.consts.disableAllLogs()) {
            Log.log("Commands/repulsor/Teleop/TeleopXRepulse", XRepulse);
            Log.log("Commands/repulsor/Teleop/XForce", repulseProcessedX);
            Log.log("Commands/repulsor/Teleop/TeleopYRepulse", YRepulse);
            Log.log("Commands/repulsor/Teleop/YForce", repulseProcessedY);
        }
        if (Robot.isBlue()) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("TeleopSwerveBaseCmd", "Blue Alliance - No Inversion");
            }
            return new Translation2d(-repulseProcessedY, repulseProcessedX);
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("TeleopSwerveBaseCmd", "Red Alliance - Inversion");
            }
            return new Translation2d(repulseProcessedY, -repulseProcessedX);
        }
    }

    protected Translation2d rotationStick() {
        double rawX = rawRotationXSup.getAsDouble();
        double rawY = rawRotationYSup.getAsDouble();
        double angle = Math.atan2(rawY, rawX);
        double rawMagnitude = Math.hypot(rawX, rawY);
        rawMagnitude = MathUtil.clamp(rawMagnitude, -1, 1);
        double magnitude =
                ControllerConstants.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
        if (demo) magnitude *= rotationMod.value();
        double processedX = magnitude * Math.cos(angle);
        double processedY = magnitude * Math.sin(angle);
        return new Translation2d(processedX, processedY);
    }

    @Override
    public void execute() {
        summarize();
    }

    @Override
    public void end(boolean interrupted) {
        if (!Robot.consts.disableAllLogs()) {
            Log.log("Commands/Teleop/teleopCommand", "ENDED");
        }
    }

    protected record TeleopSwerveJoystickRepulsorCommandSummary(
            double rawTranslationX,
            double translationX,
            double rawTranslationY,
            double translationY,
            double rawRotationX,
            double rotationX,
            double rawRotationY,
            double rotationY)
            implements StructSerializable {
        public static final Struct<TeleopSwerveJoystickRepulsorCommandSummary> struct =
                ProceduralStructGenerator.genRecord(
                        TeleopSwerveJoystickRepulsorCommandSummary.class);

        public static final TeleopSwerveJoystickRepulsorCommandSummary kZero =
                new TeleopSwerveJoystickRepulsorCommandSummary(0, 0, 0, 0, 0, 0, 0, 0);
    }

    protected TeleopSwerveJoystickRepulsorCommandSummary summarize() {
        final Translation2d translation = translationStick();
        final Translation2d rotation = rotationStick();
        // return new TeleopSwerveCommandSummary(
        //         rawTranslationXSup.getAsDouble(),
        //         translation.getX(),
        //         rawTranslationYSup.getAsDouble(),
        //         translation.getY(),
        //         rawRotationXSup.getAsDouble(),
        //         rotation.getX(),
        //         rawRotationYSup.getAsDouble(),
        //         rotation.getY());
        if (!Robot.consts.disableAllLogs()) {
            Log.log("Commands/teleop/repulsor/rawTranslationX", rawTranslationXSup.getAsDouble());
            Log.log("Commands/teleop/repulsor/translationX", translation.getX());
            Log.log("Commands/teleop/repulsor/rawTranslationY", rawTranslationYSup.getAsDouble());
            Log.log("Commands/teleop/repulsor/translationY", translation.getY());
            Log.log("Commands/teleop/repulsor/rawRotationX", rawRotationXSup.getAsDouble());
            Log.log("Commands/teleop/repulsor/rotationX", rotation.getX());
            Log.log("Commands/teleop/repulsor/rawRotationY", rawRotationYSup.getAsDouble());
            Log.log("Commands/teleop/repulsor/rotationY", rotation.getY());
        }
        return null;
    }
}
