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
import igknighters.constants.Conv;
import igknighters.controllers.DriverController;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SubsystemTriggers {
    private final Trigger disabled = RobotModeTriggers.disabled();
    private final Trigger autonomous = RobotModeTriggers.autonomous();
    private final Trigger teleop = RobotModeTriggers.teleop();

    private Trigger shouldRumble;
    private final NetworkTable dashboardTable =
            NetworkTableInstance.getDefault().getTable("dashboard");

    Command disabledLED;

    Command autoLED;

    Command teleopLED;

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
                    ;
                });
    }

    public Pose3d getPoseFromString(String path) {
        double x = dashboardTable.getEntry(path + "X").getDouble(0.0) * Conv.FEET_TO_METERS;
        double y = dashboardTable.getEntry(path + "Y").getDouble(0.0) * Conv.FEET_TO_METERS;
        double theta = dashboardTable.getEntry(path + "Theta").getDouble(0.0);
        return new Pose3d(x, y, 0, new Rotation3d(0, 0, theta));
    }

    public void SetupOperatorController(Subsystems subsystems) {
        // put whatever god awfull op controller stuff here
    }

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

    public void SetupTriggers(
            Subsystems subsystems,
            DriverController driverController,
            Supplier<Pose2d> poseSupplier) {
        Led led = subsystems.led;
        Swerve swerve = subsystems.swerve;
        SetupOperatorController(subsystems);

        // onBump.and(teleop)
        //        .whileTrue(
        //                Commands.runOnce(() -> DrivingSharedState.getInstance().setOnBump(true))
        //                        .andThen(new AutoRotateOnBump(swerve, driverController)));
        // onBump.onFalse(Commands.runOnce(() ->
        // DrivingSharedState.getInstance().setOnBump(false)));

        falseOnce().and(disabled).whileTrue(disabledLED(led));

        autonomous.onTrue(autoLED(led));

        teleop.whileTrue(teleopLED(led));

        // rumble
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
