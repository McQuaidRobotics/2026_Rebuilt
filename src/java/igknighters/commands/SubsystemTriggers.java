package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.teleop.AutoRotateOnBump;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Manages high-level robot triggers and mode-based command bindings. This class is responsible for
 * wiring up sensor-based triggers (like bumper collisions), mode changes (Teleop, Auto, Disabled),
 * and visual feedback via LEDs.
 *
 * <p>It provides a centralized place to define how the robot should react to different states and
 * inputs throughout its lifecycle.
 */
public class SubsystemTriggers {
    // Standard WPILib mode triggers
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger autonomous = RobotModeTriggers.autonomous();
    private final Trigger teleop = RobotModeTriggers.teleop();

    private Trigger shouldRumble;
    private final NetworkTable dashboardTable =
            NetworkTableInstance.getDefault().getTable("dashboard");

    /**
     * Creates a trigger that returns true only once after it is checked. Useful for one-shot
     * actions that should only happen the first time a condition is met.
     *
     * @return A one-shot {@link Trigger}.
     */
    public static Trigger falseOnce() {
        return new Trigger(
                new BooleanSupplier() {
                    boolean ret = false;

                    public boolean getAsBoolean() {
                        try {
                            return ret;
                        } finally {
                            ret = true;
                        }
                    }
                });
    }

    /**
     * Parses a 3D pose from NetworkTables for use in dashboard-driven commands.
     *
     * @param path The base path in the dashboard table.
     * @return The parsed {@link Pose3d}.
     */
    public Pose3d getPoseFromString(String path) {
        double x = dashboardTable.getEntry(path + "X").getDouble(0.0) * Conv.FEET_TO_METERS;
        double y = dashboardTable.getEntry(path + "Y").getDouble(0.0) * Conv.FEET_TO_METERS;
        double theta = dashboardTable.getEntry(path + "Theta").getDouble(0.0);
        return new Pose3d(x, y, 0, new Rotation3d(0, 0, theta));
    }

    /**
     * Sets up triggers for operator-specific dashboard controls.
     *
     * @param subsystems The robot subsystems.
     */
    public void SetupOperatorController(Subsystems subsystems) {
        Swerve swerve = subsystems.swerve;

        Trigger moveToTrigger =
                new Trigger(() -> dashboardTable.getEntry("robot/moveTrigger").getBoolean(false));

        // When the move trigger is pressed on the dashboard, drive to the specified waypoint.
        moveToTrigger.whileTrue(
                Wayfinder.driveToTarget(
                        swerve, getPoseFromString("robot/moveWaypoint").toPose2d()));
    }

    /**
     * Returns the appropriate LED command based on the current robot mode.
     *
     * @param led The LED subsystem.
     * @return A command for the current mode's LED pattern.
     */
    public Command getLEDCommandByMode(Led led) {
        return Commands.either(
                teleopLED(led), Commands.either(disabledLED(led), autoLED(led), disabled), teleop);
    }

    private Command teleopLED(Led led) {
        return LEDCommands.run(led, LEDPattern.solid(Color.kGreen))
                .ignoringDisable(true)
                .withName("TeleopGreen");
    }

    private Command autoLED(Led led) {
        return LEDCommands.run(led, LedUtil.makeRainbow(255, 126))
                .ignoringDisable(true)
                .withName("AutoRainbow");
    }

    private Command disabledLED(Led led) {
        return LEDCommands.run(led, LEDPattern.solid(Color.kRed))
                .ignoringDisable(true)
                .withName("DisabledRed");
    }

    /**
     * Initializes all robot triggers, combining subsystem states, mode changes, and driver inputs.
     *
     * @param subsystems The robot subsystems.
     * @param driverController The driver's controller.
     * @param poseSupplier A supplier for the robot's current pose.
     */
    public void SetupTriggers(
            Subsystems subsystems,
            DriverController driverController,
            Supplier<Pose2d> poseSupplier) {
        Led led = subsystems.led;
        Swerve swerve = subsystems.swerve;

        // Triggered when the robot is inside a "bump" zone on the field.
        Trigger onBump = new Trigger(() -> FieldConstants.BUMP.isInside(swerve.getState().Pose));

        SetupOperatorController(subsystems);

        // When on a bump during teleop, automatically adjust the robot's rotation.
        onBump.and(teleop)
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(
                                        () -> DrivingSharedState.getInstance().setOnBump(true)),
                                new AutoRotateOnBump(swerve, driverController)));

        // Clear the bump state when leaving the zone.
        onBump.onFalse(Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(false)));

        // Mode-based LED patterns
        falseOnce().and(disabled).whileTrue(disabledLED(led));
        autonomous.onTrue(autoLED(led));
        teleop.whileTrue(teleopLED(led));

        // Provide haptic feedback (rumble) when a vision target is successfully tracked.
        shouldRumble =
                new Trigger(() -> subsystems.vision.timeSinceLastSample() < 0.1)
                        .and(falseOnce())
                        .and(teleop)
                        .whileTrue(
                                Commands.startEnd(
                                                () -> driverController.rumble(.30),
                                                () -> driverController.rumble(0.0))
                                        .ignoringDisable(true)
                                        .withName("RumbleForTag"));
    }
}
